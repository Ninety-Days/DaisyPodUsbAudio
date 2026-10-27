# q16-drift-corrector

A single-header, dependency-free reference implementation of a fractional
(Q16) clock-drift corrector for streaming ring buffers — the kind of
problem you have whenever two nominally-identical clocks (a USB host's
audio clock and a device's crystal, an audio interface and an app's mixer,
two independently-clocked digital audio devices) turn out to differ by a
small, slowly-varying offset, typically tens of parts-per-million.

## What it does

Replaces the classic "vary the packet size by ±1 frame" drift correction
with a fractional read position that advances by slightly more or less
than 1.0 per output sample, interpolating between the ring entries around
that position. The output packet/block size never changes. The correction is spread across every sample as a fraction of a
sample instead of being concentrated into one dropped or repeated sample,
which is what makes the ±1-frame approach audible as a click.

## Which interpolator

Three readers are provided and they are **not** equivalent.

The read phase does not sit still — it sweeps 0→1 continuously at the
clock offset, so whatever the interpolator does to high frequencies
arrives as slow **amplitude modulation**, not as a static roll-off. At
30 ppm the sweep period is 0.69 s, about 1.4 Hz of AM.

AM depth as the phase sweeps its range, at 48 kHz:

| | linear | Catmull-Rom | polyphase sinc |
|---|---|---|---|
| 5 kHz | 0.47 dB | 0.04 dB | — |
| 10 kHz | 2.01 dB | 0.53 dB | — |
| 12 kHz | 3.01 dB | 1.07 dB | **0.0002 dB** |
| 15 kHz | 5.11 dB | 2.53 dB | **0.0003 dB** |
| 20 kHz | 11.74 dB | 8.41 dB | **0.0023 dB** |

Linear puts roughly 2 dB of ~1.4 Hz AM on everything near 10 kHz —
audible on cymbals and other HF-dense material. It is kept as the minimal
illustration of the idea and is adequate only where the signal has little
energy above a few kHz.

**Use `q16_read_sinc_i32()` unless the table is too expensive for you.**
It needs the companion `q16_sinc_table.h` (16 KB of `const float`,
regenerate with `gen_table.py`) and costs 32 taps per channel plus one row
blend per frame. `Q16_NO_SINC` compiles it out.

`q16_read_cubic_i32()` is the fallback when that table will not fit: no
table, four taps, and roughly 8 dB of AM at 20 kHz.

Two figures are easy to confuse. Worst-case *arithmetic* error of the
cubic against a double-precision reference is about 7 LSB of a 24-bit
sample (≈ −122 dBFS) — that is float32 rounding, **not** interpolator
transparency. The *interpolation* error of the same cubic is ≈ −89 dBFS at
1 kHz and ≈ −26 dBFS at 10 kHz, some 30–95 dB larger, and it is what
limits quality. Don't quote the arithmetic figure as if it were the
quality figure.

Read `q16_drift_corrector.h`'s header comment first — it is the actual
documentation, including the control-loop math, the tuning knobs, a
worked-through explanation of the standing-offset behavior (with numbers),
and why an SOF-derived / feedforward approach is not a substitute for the
closed-loop version here.

## Where this came from

Extracted from [DaisyPodUsbAudio](../), where it corrects drift on a
USB Audio Class 1 full-duplex stream on an STM32H750. This header has no
dependency on that project, on USB, on any HAL, on a fixed channel count,
or on a fixed sample rate — it operates on a caller-owned ring of
interleaved `int32_t` frames and nothing else. It should outlive both the
USB class version and the MCU it was first built for.

"Extracted" means the algorithm, not the source file. The sinc reader here
and `fd_read_frame` in the parent firmware share a convolution core — same
table layout, same guard row, same row blend — but they have different
contracts. This one owns and advances the read position, masks its own ring
index, and clamps to a configurable sample range; the firmware's takes the
index and fraction as arguments, clamps to 24 bits, and carries its
Catmull-Rom fallback inside the same function.

Nothing keeps the two in step. The parent tree's `check_reader_copies.py`
verifies its three textual copies of `fd_read_frame` against each other, and
it does not cover this file — a text comparison could not, given the
differing contracts. A fix to one does not reach the other. If you track the
parent project, diff the two by hand.

## Using it

Copy `q16_drift_corrector.h` into your project. It's meant to be read,
adapted, and vendored — not linked as a library. The three calls you need:

```c
q16_corrector_t rec;
q16_init(&rec);

/* producer side, once per frame written into your ring: */
q16_advance_write(&rec);

/* consumer side, once per output callback, before reading: */
uint32_t available = q16_update_tilt(&rec);
/* ... your own pre-fill / underrun handling using `available` ...
   NOTE: each reader needs its own headroom around idx -- the sinc reader
   touches idx-15 .. idx+16, the cubic idx-1 .. idx+2, the linear
   idx .. idx+1. See the header. */
q16_read_sinc_i32(&rec, your_ring, channels, out_frame);
```

Override `Q16_RING_SIZE`, `Q16_TARGET`, `Q16_RATE_DIV`, `Q16_RATE_MAX`, or
`Q16_FRAC_BITS` with a `-D` before including the header, or just edit the
`#define`s directly.

**`Q16_RING_SIZE` must be a power of two if you use the sinc reader** — it
indexes the ring by masking rather than by modulo, and the header stops the
build with an `#error` if that does not hold. The linear and cubic readers use
`%` and take any ring size, so under `-DQ16_NO_SINC` the size is free.

## Testing it

`test_q16.c` is a standalone numeric test — no hardware, no mocks. It
simulates a producer running at a real measured offset (58.5 ppm, from the
parent project's hardware) against a fixed-rate consumer, and checks the
two things that actually matter for a rate corrector:

1. **No discontinuity.** The interpolated output's sample-to-sample delta
   never exceeds what the input signal's own slew rate allows — i.e. no
   click, checked on the *output samples*, not on the ring index (which is
   allowed to step by more than one slot at a time; that's not audible on
   its own and testing for it produces a false failure — see the comment
   at the top of the test for why an earlier version of this test got that
   wrong).
2. **Bounded, indefinitely.** The ring neither empties nor overflows over
   two million steps.

It also documents, and asserts against, a real and slightly non-obvious
property of a proportional-only integer-tilt controller: the settled tilt
does not converge to the exact injected ppm, because `tilt` only takes
integer Q16 values (~15.259 ppm per unit) and a true offset that falls
between two of those rungs is matched by holding a larger standing ring
offset instead. This is expected and bounded, not a bug — worked through
with real numbers in the header and reproduced in the test.

```sh
gcc -std=c99 -Wall -Wextra -O2 test_q16.c -o test_q16 -lm && ./test_q16
```

It exercises the sinc reader by default and the cubic one under
`-DQ16_NO_SINC`, so it builds and passes in both configurations.

## License

MIT — see `LICENSE` in this directory. This file has no ST or
libDaisy content in it (see "Where this came from" above), so unlike most
of the parent project it carries no scoping caveat: the whole thing is
this project's own work and the whole thing is MIT.
