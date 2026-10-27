#!/usr/bin/env python3
"""AM depth by the definition am_probe/README.md uses: 20*log10(env.max/env.min).

Envelope by complex demodulation at the *output* carrier frequency, low-passed
with a Hann-windowed sinc FIR. Two things this gets right that the obvious
implementations do not:

  * The low-pass is a designed FIR with a 200 Hz cutoff, so the image at 2f is
    rejected by >80 dB whatever Fs/f happens to be. A moving average over an
    integer number of tone periods only nulls that image when Fs/f is an
    integer -- at 12345 Hz (Fs/f = 3.888) the leaked image is larger than the
    modulation being measured.

  * The carrier is located from the data, not assumed. The reader advances at
    1 + tilt/65536 samples per output frame, so the output tone sits above the
    input tone by that ratio; demodulating at the nominal frequency leaves a
    slow phase ramp that shows up as spurious envelope tilt.

Reports depth, AM rate, and the expected rate |tilt| * 48000 / 65536 for
comparison.
"""

# MIT License (see LICENSE at repository root) applies to this file.
import sys
import numpy as np

FS = 48000.0


def lowpass(cut_hz, ntaps=4001):
    n = np.arange(ntaps) - (ntaps - 1) / 2
    h = np.sinc(2 * cut_hz / FS * n) * np.hanning(ntaps)
    return h / h.sum()


def measure(path, f_nom, tilt=None, skip_s=2.0, cut=200.0):
    x = np.fromfile(path, dtype=np.float32).astype(np.float64)
    x = x[int(skip_s * FS):]

    # locate the actual output carrier: parabolic interpolation on the
    # Hann-windowed magnitude peak near the nominal tone
    X = np.abs(np.fft.rfft(x * np.hanning(len(x))))
    fr = np.fft.rfftfreq(len(x), d=1.0 / FS)
    lo = np.searchsorted(fr, f_nom - 100)
    k = lo + int(np.argmax(X[lo:np.searchsorted(fr, f_nom + 100)]))
    a, b, c = np.log(X[k - 1]), np.log(X[k]), np.log(X[k + 1])
    delta = 0.5 * (a - c) / (a - 2 * b + c)
    f_car = (k + delta) * fr[1]

    n = np.arange(len(x))
    z = x * np.exp(-2j * np.pi * f_car * n / FS)

    h = lowpass(cut)
    zf = np.convolve(z, h, mode='valid')
    env = np.abs(zf)

    depth = 20.0 * np.log10(env.max() / env.min())

    e = env - env.mean()
    E = np.abs(np.fft.rfft(e))
    ef = np.fft.rfftfreq(len(e), d=1.0 / FS)
    i0 = np.searchsorted(ef, 0.2)
    rate = ef[i0 + int(np.argmax(E[i0:]))]

    out = dict(depth=depth, rate=rate, f_car=f_car,
               ripple_pp=(env.max() - env.min()) / env.mean())
    if tilt is not None:
        out['rate_expected'] = abs(tilt) * FS / 65536.0
    return out


if __name__ == '__main__':
    path, f = sys.argv[1], float(sys.argv[2])
    tilt = int(sys.argv[3]) if len(sys.argv) > 3 else None
    r = measure(path, f, tilt)
    exp = f"  expected {r['rate_expected']:.3f}" if tilt is not None else ""
    print(f"{r['depth']:9.5f} dB   rate {r['rate']:6.3f} Hz{exp}   "
          f"carrier {r['f_car']:.3f} Hz")

    # --max-depth turns this from a reporter into a check. It is opt-in
    # because the script is also used by hand on arbitrary captures, where
    # there is no expected value to hold it to. CI passes it; a person
    # exploring does not have to.
    #
    # The AM rate is checked whenever a tilt was given, because there the
    # expected value is not a convention but arithmetic: tilt/65536 * Fs.
    # A mismatch there means the reader is not advancing at the rate it was
    # told to, which is a different and worse fault than a depth regression.
    fail = False
    if '--max-depth' in sys.argv:
        limit = float(sys.argv[sys.argv.index('--max-depth') + 1])
        if r['depth'] > limit:
            print(f"FAIL: depth {r['depth']:.5f} dB exceeds {limit} dB")
            fail = True
    if tilt is not None and abs(r['rate'] - r['rate_expected']) > 0.05:
        print(f"FAIL: AM rate {r['rate']:.3f} Hz is not the "
              f"{r['rate_expected']:.3f} Hz the tilt implies")
        fail = True
    sys.exit(1 if fail else 0)
