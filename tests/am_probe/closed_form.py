#!/usr/bin/env python3
"""Closed-form AM depth for the shipped polyphase table.

Independent of am_probe: reads fd_sinc_table.h, rebuilds the exact coefficient
set the reader uses at each Q16 fractional position (row p blended linearly
with row p+1), evaluates |H(w)| at the tone frequency, and reports the
peak-to-trough spread as the tone's fractional delay sweeps a full cycle.

That spread *is* the AM depth: as the drift corrector walks the read position
through one whole sample, the reader's gain at the tone frequency traces this
curve once, which is what the envelope in am_probe measures.

The same calculation is applied to the Catmull-Rom kernel as a control -- if it
reproduces the published Catmull-Rom column, the method is sound and the sinc
figure it produces can be trusted too.
"""

# MIT License (see LICENSE at repository root) applies to this file.
import os
import re
import sys
import numpy as np

FS = 48000.0


def load_table(path=None):
    if path is None:
        here = os.path.dirname(os.path.abspath(__file__))
        for cand in ('fd_sinc_table.h',
                     os.path.join(here, 'fd_sinc_table.h'),
                     os.path.join(here, '..', '..', 'fd_sinc_table.h')):
            if os.path.exists(cand):
                path = cand
                break
        else:
            path = 'fd_sinc_table.h'
    src = open(path).read()
    taps = int(re.search(r'taps per phase \(N\)\s*:\s*(\d+)', src).group(1))
    phases = int(re.search(r'phase rows \(P\)\s*:\s*(\d+)', src).group(1))
    body = src[src.index('{'):]
    vals = [float(v.rstrip('fF'))
            for v in re.findall(r'-?\d+\.\d+(?:[eE][-+]?\d+)?[fF]?', body)]
    need = taps * (phases + 1)
    assert len(vals) >= need, (len(vals), need)
    tab = np.array(vals[:need]).reshape(phases + 1, taps)
    return tab, taps, phases


def sinc_response(f, tab, taps, phases, nfrac=4096):
    w = 2 * np.pi * f / FS
    k = np.arange(taps)
    e = np.exp(-1j * w * k)
    mags = []
    for i in range(nfrac):
        frac = i * 65536.0 / nfrac
        sub_bits = 16 - int(np.log2(phases))
        p = int(frac) >> sub_bits
        sub = int(frac) & ((1 << sub_bits) - 1)
        fr = sub / (1 << sub_bits)
        co = tab[p] + fr * (tab[p + 1] - tab[p])
        mags.append(abs(co @ e))
    return np.array(mags)


def cr_response(f, nfrac=4096):
    w = 2 * np.pi * f / FS
    mags = []
    for i in range(nfrac):
        d = i / nfrac
        c1 = np.array([-0.5, 0.0, 0.5, 0.0])
        c2 = np.array([1.0, -2.5, 2.0, -0.5])
        c3 = np.array([-0.5, 1.5, -1.5, 0.5])
        c0 = np.array([0.0, 1.0, 0.0, 0.0])
        co = ((c3 * d + c2) * d + c1) * d + c0     # taps for P, A, B, C
        e = np.exp(-1j * w * np.arange(4))
        mags.append(abs(co @ e))
    return np.array(mags)


# Bounds, one per tone: (Catmull-Rom max dB, polyphase sinc max dB).
#
# The Catmull-Rom figures come from the second table in
# docs/INTERPOLATION.md with ~15% slack; they are a reference point, not a
# target, since nothing ships on that interpolator.
#
# The sinc bound is a ceiling rather than an expected value. Below about
# 0.001 dB the closed form and direct simulation disagree by 3-9x
# because both sit near the floor of what float32 and a 24-bit clamp
# resolve -- INTERPOLATION.md says to read those as "not measurable"
# rather than as values, so asserting equality there would be asserting
# noise. 0.01 dB is far above the floor and far below anything audible,
# which makes it a real regression trip without pretending to a precision
# the measurement does not have.
BOUNDS = {
    12000.0: (1.30, 0.01),
    12345.0: (1.45, 0.01),
    15000.0: (3.00, 0.01),
}


if __name__ == '__main__':
    tab, taps, phases = load_table()
    print(f"table: {taps} taps x {phases} phases (+1 guard)\n")
    print(f"{'tone':>8}  {'Catmull-Rom':>14}  {'polyphase sinc':>16}")
    fail = False
    for f in (12000.0, 12345.0, 15000.0):
        mc = cr_response(f)
        ms = sinc_response(f, tab, taps, phases)
        dc = 20 * np.log10(mc.max() / mc.min())
        ds = 20 * np.log10(ms.max() / ms.min())
        lim_c, lim_s = BOUNDS[f]
        bad = dc > lim_c or ds > lim_s
        fail = fail or bad
        print(f"{f:8.0f}  {dc:11.4f} dB  {ds:13.5f} dB  "
              f"{'FAIL' if bad else 'OK'}")
    print("\nRESULT: " + ("FAIL" if fail else "PASS"))
    sys.exit(1 if fail else 0)
