#include "dsp.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define COV_ALPHA        0.001f  // 10 s at 100 Hz
#define NOTCH_Q          8.0f    // narrow: a wide notch removes noise too and pushes the statistic negative
#define BREATH_LO        0.1f    // Hz, 6-42 breaths/min
#define BREATH_HI        0.7f
#define BREATH_PROM      12.0f   // ponytail: peak/median power; set so a quiet room with a fan stays silent
#define TONE_LO          0.8f
#define TONE_HI          45.0f
#define TONE_PROM        20.0f   // 10 s window (vs 30 s on the Pi) gives a sinusoid ~3x less prominence
#define NFFT             1024

static float fre[NFFT], fim[NFFT], fpow[NFFT / 2 + 1], tmp[NFFT];  // one caller (radar task)

static bq_t lowpass(float fc, float fs, float q) {
    float w = 2 * (float)M_PI * fc / fs, c = cosf(w), al = sinf(w) / (2 * q), a0 = 1 + al;
    return (bq_t){ (1 - c) / 2 / a0, (1 - c) / a0, (1 - c) / 2 / a0, -2 * c / a0, (1 - al) / a0 };
}

static bq_t notch(float f, float fs) {
    float w = 2 * (float)M_PI * f / fs, c = cosf(w), al = sinf(w) / (2 * NOTCH_Q), a0 = 1 + al;
    return (bq_t){ 1 / a0, -2 * c / a0, 1 / a0, -2 * c / a0, (1 - al) / a0 };
}

static inline float bq(const bq_t *f, float *s, float x) {  // direct form I, s = x1 x2 y1 y2
    float y = f->b0 * x + f->b1 * s[0] + f->b2 * s[1] - f->a1 * s[2] - f->a2 * s[3];
    s[1] = s[0]; s[0] = x; s[3] = s[2]; s[2] = y;
    return y;
}

static void fft(float *re, float *im, int n) {
    for (int i = 1, j = 0; i < n; i++) {
        int bit = n >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) { float t = re[i]; re[i] = re[j]; re[j] = t; t = im[i]; im[i] = im[j]; im[j] = t; }
    }
    for (int len = 2; len <= n; len <<= 1) {
        float a = -2 * (float)M_PI / len, wr = cosf(a), wi = sinf(a);
        for (int i = 0; i < n; i += len) {
            float cr = 1, ci = 0;
            for (int k = 0; k < len / 2; k++) {
                int p = i + k, q = p + len / 2;
                float tr = re[q] * cr - im[q] * ci, ti = re[q] * ci + im[q] * cr;
                re[q] = re[p] - tr; im[q] = im[p] - ti; re[p] += tr; im[p] += ti;
                float t = cr * wr - ci * wi; ci = cr * wi + ci * wr; cr = t;
            }
        }
    }
}

// x (n samples, chronological) -> fpow[0..NFFT/2], zero-padded; linear detrend + Hann
static void power(const float *x, int n) {
    float sx = 0, sxy = 0, mt = (n - 1) / 2.0f, stt = 0;
    for (int i = 0; i < n; i++) { sx += x[i]; sxy += (i - mt) * x[i]; stt += (i - mt) * (i - mt); }
    float mean = sx / n, slope = sxy / stt;
    for (int i = 0; i < NFFT; i++) {
        fre[i] = i < n ? (x[i] - mean - slope * (i - mt)) * (0.5f - 0.5f * cosf(2 * (float)M_PI * i / (n - 1))) : 0;
        fim[i] = 0;
    }
    fft(fre, fim, NFFT);
    for (int i = 0; i <= NFFT / 2; i++) fpow[i] = fre[i] * fre[i] + fim[i] * fim[i];
}

static int cmpf(const void *a, const void *b) { float x = *(const float *)a, y = *(const float *)b; return (x > y) - (x < y); }

static float median(const float *p, int lo, int hi) {
    int n = hi - lo + 1;
    memcpy(tmp, p + lo, n * sizeof(float));
    qsort(tmp, n, sizeof(float), cmpf);
    return tmp[n / 2] + 1e-12f;
}

void dsp_init(dsp_t *d) {
    memset(d, 0, sizeof *d);
    for (int k = 0; k < NPC; k++) d->pc[k][k] = d->brpc[k][k] = 1;
    d->lp[0] = lowpass(2.0f, 100, 0.5412f);  // 4th-order Butterworth as two biquads; a 9.3 Hz tone (aliases to 0.7) is -45 dB
    d->lp[1] = lowpass(2.0f, 100, 1.3066f);
}

void dsp_amplitudes(const int8_t iq[2 * NSUB], float na[NSUB]) {
    float sum = 0;
    for (int s = 0; s < NSUB; s++) sum += na[s] = hypotf(iq[2 * s], iq[2 * s + 1]);
    float k = 1 / (sum / NSUB + 1e-6f);  // per-packet normalization removes AGC gain steps
    for (int s = 0; s < NSUB; s++) na[s] *= k;
}

void dsp_push(dsp_t *d, const float na[NSUB]) {
    if (!d->n) memcpy(d->mu, na, sizeof d->mu);
    float x[NSUB];
    for (int s = 0; s < NSUB; s++) { x[s] = na[s] - d->mu[s]; d->mu[s] += COV_ALPHA * x[s]; }
    for (int i = 0; i < NSUB; i++)
        for (int j = 0; j < NSUB; j++) d->cov[i][j] += COV_ALPHA * (x[i] * x[j] - d->cov[i][j]);

    for (int k = 0; k < NPC; k++) {
        float p = 0;
        for (int s = 0; s < NSUB; s++) p += d->pc[k][s] * x[s];
        if (!k) d->tone_buf[d->n % TONE_N] = p;
    }
    int16_t *b = d->br[d->nbr % BR_N];
    for (int s = 0; s < NSUB; s++) {
        float v = bq(&d->lp[1], d->lpst[s][1], bq(&d->lp[0], d->lpst[s][0], na[s] - 1)) * BR_SCALE;
        b[s] = v > 32767 ? 32767 : v < -32767 ? -32767 : (int16_t)v;
    }
    if (d->n % BR_DECIM == 0) d->nbr++;

    int16_t *m = d->mring[d->n % MWIN];
    for (int s = 0; s < NSUB; s++) {
        float v = na[s];
        for (int i = 0; i < d->nnotch; i++) {
            float *st = d->nst[i][s];
            if (st[0] == 0 && st[2] == 0) st[0] = st[1] = st[2] = st[3] = v;  // fresh notch: start settled, no transient
            v = bq(&d->notch[i], st, v);
        }
        v = (v - 1) * BR_SCALE;
        m[s] = v > 32767 ? 32767 : v < -32767 ? -32767 : (int16_t)v;
    }
    d->n++;
}

float dsp_motion(const dsp_t *d) {
    if (d->n < MWIN) return 0;
    int head = d->n % MWIN, used = 0;
    float total = 0;
    for (int s = 0; s < NSUB; s++) {
        float mean = 0;
        for (int i = 0; i < MWIN; i++) mean += d->mring[i][s];
        mean /= MWIN;
        float den = 0, num = 0, prev = 0;
        for (int i = 0; i < MWIN; i++) {
            float v = d->mring[(head + i) % MWIN][s] - mean;
            den += v * v;
            if (i) num += v * prev;
            prev = v;
        }
        if (den > 1e-9f) { total += num / den; used++; }
    }
    return used ? total / used : 0;
}

// top NPC eigenvectors of cov into pc (warm-started from pc's previous value)
static void pca(float cov[NSUB][NSUB], float pc[NPC][NSUB]) {
    for (int k = 0; k < NPC; k++) {
        float v[NSUB], w[NSUB];
        memcpy(v, pc[k], sizeof v);
        for (int it = 0; it < 20; it++) {
            for (int i = 0; i < NSUB; i++) {
                w[i] = 0;
                for (int j = 0; j < NSUB; j++) w[i] += cov[i][j] * v[j];
            }
            for (int j = 0; j < k; j++) {  // deflate: stay orthogonal to the stronger components
                float dot = 0;
                for (int i = 0; i < NSUB; i++) dot += pc[j][i] * w[i];
                for (int i = 0; i < NSUB; i++) w[i] -= dot * pc[j][i];
            }
            float nrm = 0;
            for (int i = 0; i < NSUB; i++) nrm += w[i] * w[i];
            if (nrm < 1e-20f) return;
            nrm = 1 / sqrtf(nrm);
            for (int i = 0; i < NSUB; i++) v[i] = w[i] * nrm;
        }
        float dot = 0;
        for (int i = 0; i < NSUB; i++) dot += v[i] * pc[k][i];
        for (int i = 0; i < NSUB; i++) pc[k][i] = dot < 0 ? -v[i] : v[i];  // keep the sign so the PC series stays continuous
    }
}

static void tones(dsp_t *d, float fs) {
    static float x[TONE_N];
    for (int i = 0; i < TONE_N; i++) x[i] = d->tone_buf[(d->n + i) % TONE_N];
    power(x, TONE_N);
    int lo = (int)ceilf(TONE_LO * NFFT / fs), hi = (int)(TONE_HI * NFFT / fs);
    if (hi > NFFT / 2 - 1) hi = NFFT / 2 - 1;
    float med = median(fpow, lo, hi);
    int nt = 0;
    float f[MAX_TONES], p[MAX_TONES];
    for (; nt < MAX_TONES; nt++) {
        int best = lo;
        for (int i = lo; i <= hi; i++) if (fpow[i] > fpow[best]) best = i;
        if (fpow[best] / med < TONE_PROM) break;
        f[nt] = best * fs / NFFT; p[nt] = fpow[best] / med;
        for (int i = best - 5; i <= best + 5; i++) if (i >= lo && i <= hi) fpow[i] = 0;  // rest of this peak
    }
    int same = nt == d->nnotch;
    for (int i = 0; same && i < nt; i++) same = fabsf(f[i] - d->notch_f[i]) < 0.15f;
    if (!same) {
        d->nnotch = nt;
        memset(d->nst, 0, sizeof d->nst);
        for (int i = 0; i < nt; i++) { d->notch_f[i] = f[i]; d->notch[i] = notch(f[i], fs); }
    }
    d->ntones = nt;
    memcpy(d->tone_f, f, sizeof f);
    memcpy(d->tone_p, p, sizeof p);
}

static void breathing(dsp_t *d, float fs) {
    // detrended window: x[i][s] = br[i][s] - (mean[s] + slope[s] * (i - mt))
    static float cov[NSUB][NSUB];
    float mean[NSUB] = {0}, slope[NSUB] = {0}, mt = (BR_N - 1) / 2.0f, stt = 0, x[BR_N];
    for (int i = 0; i < BR_N; i++) {
        const int16_t *r = d->br[(d->nbr + i) % BR_N];
        stt += (i - mt) * (i - mt);
        for (int s = 0; s < NSUB; s++) { mean[s] += r[s]; slope[s] += (i - mt) * r[s]; }
    }
    for (int s = 0; s < NSUB; s++) { mean[s] /= BR_N; slope[s] /= stt; }
    memset(cov, 0, sizeof cov);
    for (int i = 0; i < BR_N; i++) {
        const int16_t *r = d->br[(d->nbr + i) % BR_N];
        float v[NSUB];
        for (int s = 0; s < NSUB; s++) v[s] = r[s] - mean[s] - slope[s] * (i - mt);
        for (int a = 0; a < NSUB; a++) for (int b = a; b < NSUB; b++) cov[a][b] += v[a] * v[b];
    }
    for (int a = 0; a < NSUB; a++) for (int b = 0; b < a; b++) cov[a][b] = cov[b][a];
    pca(cov, d->brpc);

    float fb = fs / BR_DECIM, best_f = 0, best_p = 0;
    int lo = (int)ceilf(BREATH_LO * NFFT / fb), hi = (int)(BREATH_HI * NFFT / fb);
    int rlo = (int)ceilf(0.05f * NFFT / fb), rhi = (int)(1.5f * NFFT / fb);  // reference band stays below the low-pass knee
    if (rhi > NFFT / 2) rhi = NFFT / 2;
    for (int k = 0; k < NPC; k++) {
        for (int i = 0; i < BR_N; i++) {
            const int16_t *r = d->br[(d->nbr + i) % BR_N];
            x[i] = 0;
            for (int s = 0; s < NSUB; s++) x[i] += d->brpc[k][s] * r[s];
        }
        power(x, BR_N);
        int bi = lo;
        for (int i = lo; i <= hi; i++) if (fpow[i] > fpow[bi]) bi = i;
        float prom = fpow[bi] / median(fpow, rlo, rhi);
        if (prom > best_p) { best_p = prom; best_f = bi * fb / NFFT; }
    }
    d->bprom = best_p;
    d->bpm = best_p >= BREATH_PROM ? best_f * 60 : 0;
}

void dsp_second(dsp_t *d, float fs) {
    if (fs < 20) return;
    pca(d->cov, d->pc);
    if (d->n >= TONE_N) tones(d, fs);
    if (d->nbr >= BR_N) breathing(d, fs);
}

float dsp_tune(float e99, float w20, int *separable) {
    *separable = e99 < w20;
    return *separable ? (e99 + w20) / 2 : e99 * 1.1f;  // overlap: favour no false alarms
}
