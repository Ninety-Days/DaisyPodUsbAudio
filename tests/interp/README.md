# `tests/interp` — interpolator error against an ideal fractional delay

Reproduces the first table in [`docs/INTERPOLATION.md`](../../docs/INTERPOLATION.md).
Runs on a desktop; no hardware, no USB.

## Run

```sh
bash check.sh
```

`interp_check.c` carries a verbatim copy of `fd_read_frame()` from
`usbd_audio_if.c`, which cannot be included here because it pulls in
libDaisy and STM32 headers. `tests/check_reader_copies.py` fails the build
if that copy drifts from the firmware, so the copy is checked rather than
merely asked for.

It builds `interp_check.c` twice — once with `-DFD_USE_SINC=1` for the
polyphase windowed-sinc reader, once with `-DFD_USE_SINC=0` for Catmull-Rom —
and prints three checks for each.

## What it checks

**Worst-case output error.** For each test tone, the worst value of
`|H(ω,d) − e^(−jωd)|` over the full fractional-delay range, relative to the
tone. This is the number that limits quality, and it is not the same thing as
the float32 rounding error inside the kernel — see the closing section of
`docs/INTERPOLATION.md`.

**Channel independence.** The left channel is driven and the right must stay
at zero. A reader that indexed the ring without accounting for interleaving
would leak here.

**Ring wrap.** Reads that cross index 0 must match an unwrapped rotation of
the same data exactly, to the LSB. The wrap arithmetic is the easiest place in
the reader to get subtly wrong.

## The one thing to keep in step

`interp_check.c` carries its own copy of `fd_read_frame()`. It has to: the
firmware version pulls in HAL headers that will not compile on a host. If you
change the reader in `usbd_audio_if.c`, change it here too, or this test will
keep passing while measuring code that no longer ships.

It reads the real coefficient table from `../../fd_sinc_table.h`, so the
coefficients themselves are never duplicated.

## Expected result

The sinc column should agree with `docs/INTERPOLATION.md` to within 0.2 dB.
CI runs this on every push.

## What it does when a number moves

Each figure is checked against a bound taken from the first table in
[`docs/INTERPOLATION.md`](../../docs/INTERPOLATION.md) with a few dB of slack
for compiler and libm differences. Channel leakage and ring-wrap mismatch are
checked against zero, since neither has a tolerable non-zero value.

The script prints `RESULT: PASS` or `RESULT: FAIL` and exits non-zero on
failure, so CI fails the build rather than going green on a printed `FAIL`.
The bounds live in `interp_check.c` beside the code that measures them.
