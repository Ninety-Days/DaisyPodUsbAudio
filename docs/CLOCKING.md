# Clocking

Background for the PLL3 patch in the README: where the four constants come
from, where the +298 ppm comes from, when the silicon revision matters, and why
48 kHz is the only rate this firmware offers.

The README gives the actions. This file gives the reasoning.

---

## 1. Silicon revision — errata ES0392 §2.2.3

**On a stock Daisy Seed this section is informational.** Electro-Smith
specifies the module at 480 MHz, which the affected revisions cannot reach, so
production boards are revision V or X and the erratum below does not apply.
It is here for boards of unknown provenance, and because the symptom it
produces is easy to misread as a firmware fault.

This firmware locks HSI48, the USB peripheral's clock source, to the host's SOF
using the Clock Recovery System (CRS). **On STM32H7 revisions Y and W, CRS
synchronization by the USB SOF signal specifically does not work**, per ST
errata **ES0392 Rev 15 (September 2025)** §2.2.3, *"Clock recovery system
synchronization with USB SOF does not work"*.

The wording is narrow, and that matters: the CRS peripheral itself is
functional on affected silicon — only the SOF sync source fails. ST classifies
the limitation as **A**, meaning a workaround exists: *"When available, use the
LSE oscillator as synchronization source."* The *when available* qualifier is
what bites here — the Daisy Seed module has no LSE crystal fitted, so ST's
documented workaround does not apply on this hardware. Check the official
module schematic (`docs.daisy.audio/hardware/Seed`) for your board revision.

```sh
python3 read_chip_rev.py
```

| `REV_ID` | revision | ES0392 §2.2.3 |
|---|---|---|
| `0x2003` | **V** | not affected |
| `0x2001` | **X** | not affected |
| `0x1003` | **Y or W** | **AFFECTED — CRS SOF sync does not work** |

The `REV_ID` ↔ revision mapping is Table 2 of the same errata; `DEV_ID` comes
from RM0433, the `DBGMCU_IDC` register.

The board this firmware was developed and measured on reads `DEV_ID 0x450`,
`REV_ID 0x2003` — revision **V**, unaffected.

### Why the symptom is ambiguous

On affected silicon the symptom is `crs_sync_ok` sitting at zero forever with
no error anywhere — which is *exactly* the symptom of the two CRS bugs this
project spent weeks chasing: a `SYNCSRC` field read at the wrong bit position,
and the wrong sync-source constant. Both are fixed in
this code, so on a board that reads V or X a stuck `crs_sync_ok` is neither the
erratum nor those bugs, and the cause is elsewhere.

### What it costs you if you are on Y or W

The audio sample rate is derived from HSE through PLL3 and is unaffected. What
is affected is the USB peripheral's own 48 MHz clock, which falls back to
HSI48's factory trim with no runtime correction. That is usually adequate at
room temperature and degrades over the temperature range. The device is not
thereby broken, but its USB clock is untrimmed and this repository's CRS-lock
claims do not hold for it.

`read_chip_rev.py` also cross-checks the live `SYNCSRC` field against what this
firmware should have set (`USB2` = 3). That check is revision-independent and
worth reading in the same output.

---

## 2. The PLL3 patch

**The stock libDaisy clock tree does not produce 48 kHz.** It produces
**48014.32 Hz — +298 ppm**. This is a fixed configuration error, not drift: it
comes from a ratio of integer dividers off a single crystal and does not move
with temperature.

**The patch is optional.** An unpatched build enumerates, plays and records,
and on a host with an accurate clock sounds identical. What it costs is
corrector headroom, which matters on hardware you have not tested and in
aggregate devices; the arithmetic is under *What the patch buys, and what it
does not* below, and the README summarises the decision.

The fix is four constants in a file that lives **outside this repository**:
`../../libDaisy/src/sys/system.cpp` (path relative to this project, per the
`Makefile`'s expected layout), inside `System::ConfigureClocks()`, in the block
commented `// PLL 3`. In current libDaisy master that block is at **lines
490–497**:

| line | stock libDaisy | patch to |
|---|---|---|
| 490 | `PeriphClkInitStruct.PLL3.PLL3M = 6;` | `PeriphClkInitStruct.PLL3.PLL3M = `**`5`**`;` |
| 491 | `PeriphClkInitStruct.PLL3.PLL3N = 295;` | `PeriphClkInitStruct.PLL3.PLL3N = `**`192`**`;` |
| 492 | `PeriphClkInitStruct.PLL3.PLL3P = 16;` | `PeriphClkInitStruct.PLL3.PLL3P = `**`10`**`;` |
| 493 | `PeriphClkInitStruct.PLL3.PLL3Q = 4;` | **unchanged** |
| 494 | `PeriphClkInitStruct.PLL3.PLL3R = 32;` | `PeriphClkInitStruct.PLL3.PLL3R = `**`25`**`;` |
| 495 | `PLL3RGE = RCC_PLL3VCIRANGE_1;` | **unchanged** |
| 496 | `PLL3VCOSEL = RCC_PLL3VCOWIDE;` | **unchanged** |
| 497 | `PeriphClkInitStruct.PLL3.PLL3FRACN = 0;` | **unchanged — leave at 0** |

Four lines. `PLL3Q` and `PLL3FRACN` stay exactly as they are.

> **Trap: there are two copies of this block in that file.** Lines **728–735**
> hold a commented-out duplicate inside the dead `//void SystemClock_Config()`
> block of the old C-style API. It carries the same `PLL3M = 6 / PLL3N = 295`
> values. A global search-and-replace will hit it, and patching *only* it
> changes nothing at all. **Patch the live block around line 490; leave the
> commented one alone.**

### Rebuild the library, not just the project

`system.cpp` compiles into `libdaisy.a`. Rebuilding this project relinks
against the **existing** archive and silently keeps the old clock tree, so the
patch appears to have no effect and the natural conclusion — that the values
are wrong — is false.

```sh
cd ../../libDaisy
make clean && make          # rebuilds libdaisy.a with the patched system.cpp
cd -                        # back to this project
make clean && make
```

### Verify

```sh
python3 read_sai_clock.py
```

| `PLL3DIVR` reads | meaning |
|---|---|
| **`0x180312BF`** | patch is live — SAI at **48000.0000 Hz**, 0.0 ppm |
| `0x1F031F26` | stock libDaisy — **+298 ppm**, patch not applied or library not rebuilt |

The register value is derived from the constants above, so it is a direct test
of what the silicon is actually running, not of what the source says.

### Where the four constants come from

Both configurations assume **HSE = 16 MHz**, which is what the Daisy Seed
carries — a generic 16 MHz part, *not* an audio-family 12.288 or 11.2896 MHz
crystal. On a board with a different crystal these numbers do not carry over;
the chain below has to be recomputed from that crystal rather than copied.

| | stock | patched |
|---|---|---|
| PLL3 ref (HSE / M) | 16 / 6 = 2.6667 MHz | 16 / 5 = **3.2 MHz** |
| VCO (ref × N) | 786.667 MHz | **614.4 MHz** |
| SAI kernel (VCO / P) | 49.167 MHz | **61.44 MHz** |
| MCKDIV chosen by HAL | 4 | **5** |
| MCLK | 12.291667 MHz | **12.288 MHz** = 256 × 48000 |
| **sample rate** | **48014.32 Hz (+298 ppm)** | **48000.0000 Hz (0.0 ppm)** |
| PLL3R kernel (ADC, I2C4) | 24.583 MHz | 24.576 MHz |

Every divider in the patched column is an integer and `FRACN` is zero, so the
result is exact rather than rounded. `PLL3R` moves from 32 to 25 purely to keep
the ADC and I2C4 kernel clock near its original 24.58 MHz after the VCO change;
it has nothing to do with audio. Both configurations keep the PLL3 input inside
`RCC_PLL3VCIRANGE_1` (2–4 MHz) and the VCO inside `RCC_PLL3VCOWIDE`
(192–836 MHz).

### What the patch buys, and what it does not

**It does not change the audio.** The corrector is an asynchronous sample-rate
converter, and at equilibrium its tilt exactly cancels the offset between the
host's SOF timing and the board's SAI clock — that identity is the subject of
section 4 below. A signal resampled by 48014.32/48000 and clocked out at
48014.32 Hz reproduces the original in real time: same duration, same pitch.
Uncorrected, +298 ppm would be 0.52 cents against a discrimination threshold
near 5–10 cents; corrected, it is not there at all. Nor is the interpolator's
amplitude modulation an argument here: a larger standing offset moves the AM
rate *away* from the ~4 Hz peak of human sensitivity rather than toward it, and
at the shipped table's 0.0002 dB depth neither rate is audible — see
[`INTERPOLATION.md`](INTERPOLATION.md).

**What it buys is ring headroom, measured.** Two 160-second duplex runs, same
board, same host, same firmware source; the only difference is whether
`libdaisy.a` was built from a patched `system.cpp`:

| | stock | patched |
|---|---|---|
| measured offset (from tilt) | 242 ppm | 59 ppm |
| authority used, rec / pb | 50 % / 47 % | 12 % / 9 % |
| standing ring offset | 130 / 126 frames | 32 / 31 frames |
| `pb_av` steady-state band | 102…169 of 512 | 273…320 of 512 |
| worst margin to empty | 102 frames (2.1 ms) | 273 frames (5.7 ms) |
| sawtooth period | 4.1 s | 16.9 s |
| sawtooth amplitude | ~67 frames | ~47 frames |
| underruns / overruns / `tx_size_deviations` | 0 / 0 / 0 | 0 / 0 / 0 |

Both runs passed every check. The unpatched build is not broken; what it lacks
is cushion. `REC_RATE_DIV` and `PB_RATE_DIV` are 8, so the standing ring error a
proportional controller must hold is `tilt × 8` frames — 130 frames at 242 ppm
against 32 at 59 ppm, which the two runs confirm directly. That offset is
subtracted from the 256 frames between target and an edge, and the sawtooth
rides on what is left.

The sawtooth behaves as the model predicts, which is worth stating because it
is the part most easily misread. Its **amplitude** barely moves between the two
runs; its **period** scales inversely with the offset, 16.9 s at 59 ppm against
a predicted 4.1 s and a measured 4.1 s at 242 ppm. An unpatched board does not
swing further, it swings four times as often, and it does so from a starting
point 100 frames closer to empty.

> **242 ppm is the favourable case, not the typical one.** The divider error is
> +298 ppm and this board's crystal leans the other way, so the two partly
> cancel. A board whose crystal leans with the error lands nearer 360 ppm, a
> standing offset near 189 frames, and a playback ring sitting inside 70 frames
> of empty before the sawtooth is counted. The patch removes the divider term
> on any board; the crystal term is physical and is what the corrector exists
> for.

`read_transport_diagnostics.py` reaches the same conclusion from the same data:
above a threshold fraction of authority its standing-offset check appends *"fix
the clock rather than widening the clamp"*. At 50 % it appears; at 12 % it does
not.

---

## 3. Sample rate: 48 kHz only

This device runs at **48000 Hz and nothing else**, by design. The Type I format
descriptor declares a single discrete frequency (`bSamFreqType = 0x01`,
`tSamFreq = 48000`), and the class-specific endpoint descriptors set
`bmAttributes = 0x00`, so no sampling-frequency control is offered and a host
cannot request a different rate.

**Why it is fixed.** The whole point of the PLL3 work above is an integer clock
chain: HSE 16 MHz / M=5 = 3.2 MHz, × N=192 = 614.4 MHz VCO, / R=25 =
24.576 MHz = 48000 × 512, with `FRACN = 0`. That yields exactly 48000.0000 Hz
with no fractional synthesis anywhere in the path, and it is verifiable from
the live registers with `read_sai_clock.py` rather than taken on trust.

44.1 kHz has no integer solution on this board. Against the fixed 3.2 MHz PFD
input, the required 22.5792 MHz needs N = 176.4 — not an integer, and PLL3N
only takes integers. Measured against HSE directly, 22.5792 MHz / 16 MHz
reduces to 882/625: since 882 and 625 share no factors, an exact ratio would
need N itself to be a multiple of 882, and PLL3N tops out at 512, so no
divider combination reaches it. Supporting it
natively would mean enabling fractional-N synthesis, reconfiguring PLL3 and SAI
live on a running stream, adding endpoint sampling-frequency control, and
re-deriving every ring constant (`PB_TARGET`, `REC_TARGET`, `RATE_DIV`, the Q16
reader's authority limits) for a second rate. That is a materially different
firmware with a materially different validation burden, and none of the
measurements in this repository would carry over to it.

Choosing one rate that is provably exact over two rates that are approximately
right is deliberate.

**What this means in practice.** 44.1 kHz material — most music files, CD rips,
and streaming services such as Spotify — is resampled by the host before it
reaches the device. On macOS and Windows this is transparent and needs no
configuration. If you want the host resampler out of the path entirely, feed
the device 48 kHz sources, or set your player or project sample rate to 48 kHz.

Note that this is a difference in kind from a multi-rate interface: an
interface that supports 44.1 natively will switch to it and bypass the host
resampler, and this one cannot. For a fixed-rate embedded transport on a 48 kHz
platform that is a normal and reasonable trade; it is documented here so it is
not a surprise.

---

## 4. Reading the measured offset

`read_transport_diagnostics.py` reports a clock-domain offset under CHECK 4,
in ppm, derived from the corrector's tilt rather than from event counts. At
equilibrium the tilt must exactly cancel the offset between the host's SOF
timing and the board's SAI clock, so the tilt *is* the offset.

That number on its own does not say where the offset comes from. Pair it with
`read_sai_clock.py`, which reads the live PLL and SAI registers and computes
the rate the hardware is actually configured for. The two together resolve it:

**Both report a similar figure.** The rate is misconfigured — a divider error
in the PLL/SAI constants, fixable in the source. This is what an unpatched
libDaisy looks like: section 2 above measures it at 298 ppm, which is
`48014.32 Hz` instead of `48000 Hz`.

**`read_sai_clock.py` reports ~0 ppm while CHECK 4 shows a real offset.** The
rate is configured exactly and the difference is physical: the actual HSE
crystal deviates from the `HSE_VALUE` the firmware was compiled against. That
deviation scales every derived clock and **appears in no register anywhere** —
the registers report the dividers, which are correct. Only the corrector can
see it, because only the corrector compares against the host's real SOF
timing.

The second case is the normal one for a small offset and is not a fault. A
16 MHz crystal specified at ±30 ppm can legitimately land anywhere in a
±30 ppm band, and boards differ. The corrector absorbs it, at the cost of the
standing ring offset reported under CHECK 5 — roughly 12 % of authority and
32 frames of headroom for an offset in the tens of ppm, against the ±488 ppm
and 256 frames available. Both figures are for one board at one temperature;
crystals drift with both.

Worth chasing only when the fraction of authority gets large. At that point
CHECK 5 says so directly, and the fix is the clock, not a wider clamp.
