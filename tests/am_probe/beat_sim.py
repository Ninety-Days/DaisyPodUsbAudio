#!/usr/bin/env python3
"""Burst-level simulation of the record-side drift loop.

Explains the ~17.18 s / ~44-frame sawtooth in rec_avail:
both sides of the ring move in bursts of one 48-frame block, on two different
~1 kHz clocks. Their relative phase slips one whole block period every
1/(offset x packet_rate) seconds; at that instant one extra block lands.
"""

# MIT License (see LICENSE at repository root) applies to this file.
import sys

import numpy as np

def sim(eps, secs=700.0, RATE_DIV=8, TARGET=256, BLK=48, burst=True):
    f_r, f_w = 1000.0, 1000.0*(1.0+eps)
    read_pos, avail_log, tilt_log = 0.0, [], []
    for m in range(int(f_r*secs)):
        t = m/f_r
        W = BLK*int(np.floor(t*f_w)) + TARGET if burst else BLK*(t*f_w) + TARGET
        avail = int(np.floor(W - read_pos))
        tilt  = max(-32, min(32, int((avail - TARGET)/RATE_DIV)))
        read_pos += BLK*(1.0 + tilt/65536.0)
        avail_log.append(avail); tilt_log.append(tilt)
    return np.array(avail_log), np.array(tilt_log)

if __name__ == "__main__":
    EPS = 58.2e-6
    a, t = sim(EPS); a, t = a[100000:], t[100000:]
    jumps = np.where(np.abs(np.diff(a)) > 20)[0]
    per = np.diff(jumps).mean()/1000.0
    print("simulated vs logged (600 s run)")
    print(f"  period        {per:6.2f} s   logged 17.18 s   closed form {1/(EPS*1000):.2f} s")
    print(f"  cycles/600 s  {600/per:6.1f}      logged 34")
    print(f"  excursion     {a.max()-a.min():4d} fr    logged 44 fr")
    print(f"  tilt range    {t.min()}..{t.max()}       mean {t.mean():.2f} -> {t.mean()*1e6/65536:.1f} ppm")
    print()
    # The period is the one number here with an independent reference: the
    # closed form 1/(EPS*1000) and the 17.18 s seen in a logged 600 s run.
    # If the simulation stops reproducing it, either the model or the
    # constants it is built from have moved, and INTERPOLATION.md's account
    # of the sawtooth stops being backed by anything.
    fail = abs(per - 17.18) > 0.5

    print("audibility of the swing")
    print(f"  pitch    {(t.max()-t.min())*1e6/65536:.0f} ppm = {(t.max()-t.min())*1e6/65536*1731e-6:.3f} cent   (JND ~5-10 cent)")
    print(f"  AM rate  {t.min()*48000/65536:.2f} .. {t.max()*48000/65536:.2f} Hz, sweeping once per cycle")
    print(f"  AM depth 0.001 dB on sinc / 1.07 dB on Catmull-Rom at 12 kHz (see am_probe.c)")
    print()
    print("RESULT: " + ("FAIL" if fail else "PASS"))
    sys.exit(1 if fail else 0)
