# `am_probe` — AM-envelope measurement of the fractional-delay reader

Answers, without hardware: **how much amplitude modulation does the drift
corrector's fractional reader put on a steady tone, and at what rate?**

The AM lives entirely in the reader, so the reader is driven directly.
`am_probe.c` contains a verbatim copy of `fd_read_frame()` (the same copy
`tests/interp/interp_check.c` uses) and runs it at a fixed integer `tilt`
against a nominal 1:1 writer, dumping float32 output on stdout.

## Build and run

```sh
cc -O2 -I../.. -DFD_USE_SINC=1 -o am_sinc am_probe.c -lm
cc -O2 -I../.. -DFD_USE_SINC=0 -o am_cr   am_probe.c -lm

./am_cr 12000 4 12 > out.f32          # tone Hz, tilt Q16, seconds
python3 demod.py out.f32 12000 4      # -> depth dB, AM rate Hz

python3 closed_form.py                # analytic control, no simulation
```

`-I../..` points the compiler at `fd_sinc_table.h` in the repository root;
`closed_form.py` finds it by itself. Neither needs the table copied here.

`demod.py` is the measurement half and ships with the probe deliberately —
see *How to demodulate, and how not to* below. `closed_form.py` computes the
same depths analytically from `fd_sinc_table.h` with no simulation at all, as
an independent control.

## Results (48 kHz, −12 dBFS tone, tilt = 4)

Measured with `demod.py`; closed-form values from `closed_form.py` in
parentheses.

| tone | linear | Catmull-Rom | polyphase sinc |
|---|---|---|---|
| 12 000 Hz | 3.01 dB | 1.072 dB (1.072) | 0.00002 dB (0.00018) |
| 12 345 Hz | — | 1.195 dB (1.195) | 0.00005 dB (0.00022) |
| 15 000 Hz | 5.11 dB | 2.526 dB (2.526) | 0.00007 dB (0.00032) |
| 18 000 Hz | 8.34 dB | 5.256 dB (5.256) | 0.00018 dB (0.00054) |

**The sinc column is smaller than this file used to claim.** It previously
read 0.001 / 0.011 / 0.001 dB. Two things were wrong with that. The values
are 5–50× too large — simulation and closed form agree the real figures are
0.0002 dB and below. And the 12 345 Hz outlier was not real: the closed form
puts 12 345 Hz (0.00022) essentially level with 12 000 Hz (0.00018), and
makes 15 000 Hz the deeper of the three. The old numbers came from a
demodulator that leaked; see below.

The Catmull-Rom and linear columns were correct and are unchanged. Catmull-Rom
now reproduces to three decimals by two independent methods.

**AM rate = `|tilt| × 48000 / 65536` = `|tilt| × 0.7324 Hz`.** Measured
2.177 Hz at `tilt`=3 (formula: 2.197) and 2.924 Hz at `tilt`=4 (formula:
2.930), independent of tone frequency; the residual is envelope-FFT bin width.
An earlier version of this file quoted 2.959 Hz for `tilt`=4, which disagrees
with its own formula by 1%. `tilt` is quantised to integers, so the AM rate
moves in 0.73 Hz steps as the control loop hunts — read the live value and its
min..max range with `read_transport_diagnostics.py`.

## How to demodulate, and how not to

This section exists because the sinc column was wrong for a mechanical reason
that is easy to repeat.

The obvious implementation — multiply by `exp(-j*2*pi*f*n/Fs)`, then smooth
with a moving average over a whole number of tone periods — nulls the image at
2f only when `Fs/f` is an integer. At 12 kHz (`Fs/f` = 4) it does. At 12 345 Hz
(`Fs/f` = 3.888) it does not, and the leaked image is *larger than the
modulation being measured*. That is where 0.011 dB came from: it is the
demodulator's leakage, not the reader's AM.

A hard band mask in the frequency domain fails differently — it rings at its
edges, and the ringing again dominates a 0.0002 dB signal.

`demod.py` avoids both: a designed Hann-windowed sinc FIR with a 200 Hz cutoff
(>80 dB rejection at 2f regardless of `Fs/f`), and the carrier located from the
data rather than assumed, since the reader advances at `1 + tilt/65536` samples
per output frame and the output tone sits slightly above the input tone.

**The tell in all three failure modes is the same:** the reported AM rate comes
out nowhere near `|tilt| × 0.7324 Hz`. `demod.py` prints the measured rate next
to the expected one for exactly this reason. If they disagree, the depth figure
is an artefact — do not record it.

**Sanity check before trusting any change here:** run the Catmull-Rom column.
It is large enough to be immune to all of these problems, and it has a known
answer to three decimals. A method that gets 1.072 / 1.195 / 2.526 is measuring
the reader; a method that does not is measuring itself.

## Notes on test design

- **Do not use exactly 12 000 Hz** as the only probe: it is `Fs/4`, a
  degenerate 4-sample period. 12 345 Hz reads 0.123 dB deeper on Catmull-Rom
  (closed form: 1.1947 vs 1.0721).
- Keep the tone near −12 dBFS. Near full scale the 24-bit clamp fires and adds
  its own artefact. Peak `Σ|h|` verified against the shipped table: 2.2357 for
  sinc, 1.2500 for cubic.
- The linear column is from the closed-form model (`|H(ω,d)|` swept over `d`),
  since the shipped reader no longer offers a linear path. Verified: 3.01 dB at
  12 kHz and 5.11 dB at 15 kHz are exact.
- That model reproduces the Catmull-Rom column to 0.001 dB. It does **not**
  agree with the sinc column to any useful relative tolerance — closed form and
  simulation differ by 3–9× on a quantity of 0.0002 dB, both being near the
  floor of what float32 output and a 24-bit clamp can resolve. The honest
  statement about sinc is an upper bound, not a value.

## `beat_sim.py`

Separate from the AM measurement: a burst-level simulation of the record-side
drift loop, explaining the ~17.18 s sawtooth in `rec_avail`. Run it with no
arguments.

## Bounds

`closed_form.py` holds each tone to a ceiling in its own `BOUNDS` table and
exits non-zero when one is exceeded. `demod.py` reports by default and checks
only when given `--max-depth`, because it is also run by hand on arbitrary
captures where there is no expected value; when a tilt argument is supplied it
always checks the AM rate against the `tilt/65536 * Fs` the tilt implies.
`beat_sim.py` checks its sawtooth period against the 17.18 s from a logged
600 s run.

The sinc ceiling is 0.01 dB rather than a measured value. Below roughly
0.001 dB the closed form and direct simulation disagree by 3–9x, both
being near the floor of what float32 output and a 24-bit clamp resolve, so a
tighter bound would be asserting noise.
