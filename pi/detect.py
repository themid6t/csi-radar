"""CSI feature extraction. Pure numpy, no state, so it can run on recordings offline too.

motion_stat: WiDetect-style motion statistic, the mean lag-1 autocorrelation of each subcarrier's
  amplitude over ~1 s. A static channel is white noise (~0); movement makes consecutive packets
  correlated (-> 1). It doesn't depend on the room, so a fixed threshold mostly works.
spectrum: top principal components of ~30 s of amplitudes at the packet rate (~100 Hz), FFT'd. Used for
  breathing (dominant 0.1-0.7 Hz peak, only trusted when it towers over the band; the person has to be
  near the link) and periodic interference (narrow tones such as a ceiling fan's rotation/blade rate).
"""
import numpy as np

FS = 100.0                 # resample at the packet rate: decimating aliases fan tones into the breathing band
BREATH_BAND = (0.1, 0.7)   # Hz, 6-42 breaths/min
BREATH_PROMINENCE = 12.0   # ponytail: peak/median power; set so a quiet room with a fan running stays silent
TONE_BAND = (0.8, 45.0)
TONE_PROMINENCE = 60.0


def amplitudes(iq):
    """iq: (n, 52, 2) int8 (imag, real) -> per-packet normalized amplitude (n, 52). Removes AGC gain steps."""
    a = np.hypot(iq[..., 0].astype(np.float32), iq[..., 1].astype(np.float32))
    return a / (a.mean(axis=1, keepdims=True) + 1e-6)


def motion_stat(na):
    """na: (W, 52) normalized amplitudes, consecutive packets. Returns a value in about [-0.1, 1]."""
    x = na - na.mean(axis=0)
    den = (x * x).sum(axis=0)
    ok = den > 1e-9
    if not ok.any():
        return 0.0
    acf = (x[1:] * x[:-1]).sum(axis=0) / (den + 1e-9)
    return float(acf[ok].mean())


def spectrum(t, na, npc=3):
    """t: (n,) seconds, na: (n, 52). Returns (freqs, [power per principal component]) or None if < 20 s."""
    if len(t) < 2 or t[-1] - t[0] < 20:
        return None
    grid = np.arange(t[0], t[-1], 1 / FS)
    x = na[np.clip(np.searchsorted(t, grid), 0, len(t) - 1)]
    n = len(x)
    coef = np.polyfit(np.arange(n), x, 1)          # detrend each subcarrier
    x = x - (coef[0] * np.arange(n)[:, None] + coef[1])
    _, _, vt = np.linalg.svd(x, full_matrices=False)
    nfft = 1 << int(np.ceil(np.log2(n * 4)))
    win = np.hanning(n)
    return np.fft.rfftfreq(nfft, 1 / FS), [np.abs(np.fft.rfft(pc * win, nfft)) ** 2 for pc in (x @ vt[:npc].T).T]


def breathing(spec):
    """-> (bpm or None, prominence)."""
    if spec is None:
        return None, 0.0
    f, ps = spec
    band = (f >= BREATH_BAND[0]) & (f <= BREATH_BAND[1])
    ref = (f >= 0.05) & (f <= 2.0)
    best = (0.0, 0.0)
    for p in ps:
        i = np.argmax(np.where(band, p, 0))
        prom = p[i] / (np.median(p[ref]) + 1e-12)
        if prom > best[1]:
            best = (f[i] * 60, prom)
    return (round(float(best[0]), 1) if best[1] >= BREATH_PROMINENCE else None), float(best[1])


def tones(spec, top=3):
    """Strong narrowband interference in the first component -> [(Hz, prominence)], strongest first."""
    if spec is None:
        return []
    f, ps = spec
    p = ps[0]
    band = (f >= TONE_BAND[0]) & (f <= TONE_BAND[1])
    med = np.median(p[band]) + 1e-12
    out, q = [], np.where(band, p, 0)
    for _ in range(top):
        i = int(np.argmax(q))
        if q[i] / med < TONE_PROMINENCE:
            break
        out.append((round(float(f[i]), 2), round(float(q[i] / med))))
        q[max(0, i - 20):i + 20] = 0              # suppress the rest of this peak
    return out


def tune(empty, walking):
    """Threshold from labelled motion_stat samples. Returns (thr, separable)."""
    e99, w20 = np.percentile(empty, 99), np.percentile(walking, 20)
    if e99 < w20:
        return float((e99 + w20) / 2), True
    return float(e99 * 1.1), False  # overlap: favour no false alarms


if __name__ == "__main__":
    rng = np.random.default_rng(0)
    still = 1 + 0.02 * rng.standard_normal((100, 52))
    walk = 1 + 0.02 * rng.standard_normal((100, 52)) + 0.2 * np.sin(np.linspace(0, 6, 100))[:, None] * rng.standard_normal(52)
    assert motion_stat(still) < 0.15 < 0.6 < motion_stat(walk), (motion_stat(still), motion_stat(walk))

    t = np.arange(0, 30, 0.01)
    base = 1 + 0.02 * rng.standard_normal((len(t), 52))
    fan = 0.05 * np.sin(2 * np.pi * 5.33 * t)[:, None] * rng.standard_normal(52)
    breath = base + fan + 0.01 * np.sin(2 * np.pi * 0.25 * t)[:, None] * rng.standard_normal(52)
    bpm, _ = breathing(spectrum(t, breath))
    assert bpm is not None and abs(bpm - 15) < 1, bpm
    assert breathing(spectrum(t, base + fan))[0] is None
    tn = tones(spectrum(t, base + fan))
    assert tn and abs(tn[0][0] - 5.33) < 0.05, tn

    thr, ok = tune(rng.uniform(0, 0.1, 300), rng.uniform(0.3, 0.9, 300))
    assert ok and 0.1 < thr < 0.4, thr
    print("detect.py self-check ok")
