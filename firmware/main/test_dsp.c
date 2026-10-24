// Host check for dsp.c: cc -O2 -o /tmp/t test_dsp.c dsp.c -lm && /tmp/t [recording.npy]
#include "dsp.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static float randn(void) {
    float u = (rand() + 1.0f) / (RAND_MAX + 2.0f), v = (rand() + 1.0f) / (RAND_MAX + 2.0f);
    return sqrtf(-2 * logf(u)) * cosf(2 * (float)M_PI * v);
}

static dsp_t d;
static float r1[NSUB], r2[NSUB];

// 1 + noise + fan (5.33 Hz) + breathing (0.25 Hz), at 100 Hz
static void feed(int secs, float fan, float breath, float walk) {
    for (int i = 0; i < secs * 100; i++) {
        float t = d.n / 100.0f, na[NSUB];
        for (int s = 0; s < NSUB; s++)
            na[s] = 1 + 0.02f * randn() + fan * sinf(2 * (float)M_PI * 5.33f * t) * r1[s]
                  + breath * sinf(2 * (float)M_PI * 0.25f * t) * r2[s] + walk * sinf(t * 6) * r2[s];
        dsp_push(&d, na);
        if (d.n % 100 == 0) dsp_second(&d, 100);
    }
}

int main(int argc, char **argv) {
    setvbuf(stdout, NULL, _IONBF, 0);
    srand(1);
    for (int s = 0; s < NSUB; s++) { r1[s] = randn(); r2[s] = randn(); }

    dsp_init(&d); feed(2, 0, 0, 0);
    assert(dsp_motion(&d) < 0.15f);
    dsp_init(&d); feed(2, 0, 0, 0.2f);
    assert(dsp_motion(&d) > 0.6f);

    dsp_init(&d); feed(1, 0.05f, 0, 0);
    float raw = dsp_motion(&d);  // no notch yet
    feed(60, 0.05f, 0, 0);
    printf("fan: ms %.3f -> %.3f with notch, tones", raw, dsp_motion(&d));
    for (int i = 0; i < d.ntones; i++) printf(" %.2f Hz (x%.0f)", d.tone_f[i], d.tone_p[i]);
    printf(", bpm %.1f (prom %.1f)\n", d.bpm, d.bprom);
    assert(raw > 0.3f && dsp_motion(&d) < 0.15f);
    assert(d.ntones && fabsf(d.tone_f[0] - 5.33f) < 0.1f);
    assert(d.bpm == 0);

    dsp_init(&d); feed(60, 0.05f, 0.01f, 0);
    printf("breathing 15/min with fan: bpm %.1f (prom %.1f)\n", d.bpm, d.bprom);
    assert(fabsf(d.bpm - 15) < 1);

    int sep;
    float thr = dsp_tune(0.1f, 0.3f, &sep);
    assert(sep && thr > 0.1f && thr < 0.3f);

    if (argc > 1) {  // a recording from the old Pi server: .npy of {u32 ms, i8 rssi, i8 iq[104]}
        FILE *f = fopen(argv[1], "rb");
        unsigned char h[10];
        fread(h, 1, 10, f);
        fseek(f, 10 + h[8] + 256 * h[9], SEEK_SET);
        unsigned char rec[109];
        static float ms[1000];
        int nms = 0;
        dsp_init(&d);
        while (fread(rec, 1, sizeof rec, f) == sizeof rec) {
            float na[NSUB];
            dsp_amplitudes((int8_t *)rec + 5, na);
            dsp_push(&d, na);
            if (d.n % 10 == 0 && d.n >= MWIN && nms < 1000) ms[nms++] = dsp_motion(&d);
            if (d.n % 100 == 0) dsp_second(&d, 100);
        }
        float s[1000];
        memcpy(s, ms, nms * sizeof(float));
        for (int i = 1; i < nms; i++) for (int j = i; j && s[j - 1] > s[j]; j--) { float t = s[j]; s[j] = s[j - 1]; s[j - 1] = t; }
        printf("recording: %u packets, ms p50 %.3f p99 %.3f, %d tones, bpm %.1f (prom %.1f)\n",
               d.n, s[nms / 2], s[nms * 99 / 100], d.ntones, d.bpm, d.bprom);
    }
    puts("dsp self-check ok");
}
