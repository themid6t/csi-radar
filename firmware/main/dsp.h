// CSI feature extraction, plain C (no ESP-IDF), so it also builds on the host for test_dsp.c.
//
// motion: WiDetect motion statistic, the mean lag-1 autocorrelation of each subcarrier's normalized
//   amplitude over 1 s. Static channel = white noise (~0), movement correlates packets (-> 1).
//   Detected fan tones are notched out first: a 5 Hz rotation at 100 Hz has lag-1 autocorrelation 0.95.
// PCA: streaming covariance (10 s time constant), top 3 eigenvectors by power iteration once a second.
// tones: FFT of PC1 over 10 s at the full packet rate.
// breathing: each subcarrier low-passed (4th order, 2 Hz) before decimating to 10 Hz, so fan harmonics
//   can't alias into 0.1-0.7 Hz; PCA of that 30 s window, then FFT. (Projecting first and filtering
//   after doesn't work: the once-a-second PC updates mix a 5.33 Hz fan down to 0.67 Hz.)
#pragma once
#include <stdint.h>

#define NSUB      52
#define MWIN      100    // motion window, ~1 s
#define TONE_N    1024   // ~10 s at 100 Hz
#define BR_N      300    // 30 s at 10 Hz
#define BR_DECIM  10
#define BR_SCALE  8192.0f
#define NPC       3
#define MAX_TONES 3

typedef struct { float b0, b1, b2, a1, a2; } bq_t;

typedef struct {
    uint32_t n;                          // packets pushed
    float mu[NSUB], cov[NSUB][NSUB], pc[NPC][NSUB];
    int16_t mring[MWIN][NSUB];           // notched amplitudes, (a - 1) * BR_SCALE, ring
    int nnotch; float notch_f[MAX_TONES]; bq_t notch[MAX_TONES]; float nst[MAX_TONES][NSUB][4];
    float tone_buf[TONE_N];              // PC1, ring
    bq_t lp[2]; float lpst[NSUB][2][4];
    int16_t br[BR_N][NSUB]; uint32_t nbr; // low-passed amplitudes at 10 Hz, (a - 1) * BR_SCALE, ring
    float brpc[NPC][NSUB];
    // outputs of dsp_second()
    int ntones; float tone_f[MAX_TONES], tone_p[MAX_TONES];
    float bpm, bprom;                    // bpm = 0 when no breathing peak
} dsp_t;

void  dsp_init(dsp_t *d);
void  dsp_amplitudes(const int8_t iq[2 * NSUB], float na[NSUB]);   // (imag, real) pairs -> normalized amplitude
void  dsp_push(dsp_t *d, const float na[NSUB]);
float dsp_motion(const dsp_t *d);
void  dsp_second(dsp_t *d, float fs);   // fs = measured packet rate
float dsp_tune(float empty_p99, float walking_p20, int *separable);
