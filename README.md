# DaisyPodUsbAudio

[![tests](../../actions/workflows/tests.yml/badge.svg)](../../actions/workflows/tests.yml)

USB Audio Class 1 (UAC1) bidirectional device firmware for the Daisy Pod
(STM32H750), plus MIDI. Class-compliant: no driver on macOS, Windows or Linux.

2 in / 2 out, 24-bit, 48 kHz.

This is the DSP-free base — a straight round-trip USB audio card with no effect
or synthesis stage. It is the transport layer I later put my own Clario DSP on,
along with FlickPort, NeuralPort and FmPort as checks and worked examples.

## Why this transport exists

The goal was never a sound card as such. It was to run DSP **on the board** and
drive it from a DAW — the Pod as real studio hardware in a session, where the
plugin is the control surface and audio leaves over USB and comes back
processed.

That gives four working setups, which are what the whole thing was built for:

- **Insert processing from the DAW.** Audio goes out to the board, is processed
  there, and returns to the project.
- **Live input with simultaneous recording.** A signal feeds the board's audio
  input, is heard processed at its output in real time, and is recorded to a DAW
  track at the same time — dry or processed. Automation can be applied
  afterwards, to the dry signal, still using the board's DSP.
- **An instrument rather than an effect.** If the board runs a synth or other
  instrument, the same holds over MIDI: you play and hear the result
  immediately, while MIDI data lands on a track for later editing and
  automation.
- **Aggregate-device operation.** The Daisy acts as a second interface and takes
  the DSP, the main interface handles everything else, and synchronisation
  holds. Or the Pod is used as the only interface, with the same capabilities.

All four work.

That goal is what made a correct class-compliant USB audio transport mandatory
rather than convenient: it carries both the audio and the control parameters.

## Status

Released under MIT for this project's own files — see
[Licensing](#licensing), which is not a single-licence story.

Every figure quoted here for this firmware was measured on hardware or by the
desktop tests, and is reproducible with the scripts in this repository.

Host-side observations — enumeration behaviour, stream negotiation, what a
given host does or refuses to do — were made on **macOS 15.2 Sequoia** and,
where Windows is named, **Windows 10**. Hosts change between releases: read
those as dated observations, not as properties of the protocol. Figures for the
transport itself (rates, ring depths, ppm) do not depend on the host.

This is a finished transport, published as a dated snapshot rather than an
actively developed project. Reports of the documentation disagreeing with the
hardware are welcome and are the thing most worth sending — see
[Bug reports](#bug-reports) for what to include. Pull requests will be read,
but no response time is promised. If you are vendoring a file rather than
using the whole tree, check it in `THIRD_PARTY_NOTICES.md` first: most `usbd_*`
files are ST's under SLA0044 and the root MIT licence does not reach them.

---

## Try it without building

A prebuilt firmware image is attached to the release, with the libDaisy PLL3
patch already applied. If you only want to hear the device work, this skips the
entire build section below.

Download the archive from [Releases](../../releases), unzip it, and flash
`DaisyPodUsbAudio.bin` with the [Electro-Smith web programmer](https://flash.daisy.audio/):
put the Pod into DFU mode (hold BOOT, tap RESET, release BOOT), connect, choose
the file, flash.

The release notes carry the SHA256, the libDaisy commit the image was built
against, and the toolchain version, so the binary can be reproduced from source
rather than trusted. They also repeat the licensing summary below — the image
contains ST code under SLA0044, which restricts execution to ST silicon.

Build from source if you want to change anything, or to verify the measured
figures yourself.

### If you listen on the Pod's headphone jack first

You will probably hear a steady tone behind the music. Before filing it as a
transport bug: it is not in the samples. The recorded return is clean, and the
diagnostics report zero underruns, overruns and packet-size deviations.

It is supply modulation. The CPU computes an audio block and then idles until
the next interrupt, and that alternation modulates `+3V3_A`. On Pod Rev5 the
headphone amplifier's VCC sits on that rail through a single 100 nF capacitor,
where the vendor's own reference circuit for the same part uses a filtered
rail. The tone's fundamental is the audio callback rate — change
`AUDIO_BLOCK_FRAMES` and it moves with it.

Use LINE OUT. With a monitor amp at maximum and the stream open the tone is
right at the threshold of being distinguishable there, so at any normal
listening level it is simply not present. For headphones, drive them from LINE
OUT through an external amplifier — the same headphones that ring on the Pod's
own jack are almost silent that way.

[`docs/PITFALLS.md`](docs/PITFALLS.md) section 7 has the measurements, what was
tried, and why the fix is not in this firmware.

---

## Building

Requires libDaisy, cloned separately, and its toolchain (`arm-none-eabi-gcc`,
`make`). See the `Makefile` for the expected directory layout.

### libDaisy versions this was built against

Two, chosen because they sit on opposite sides of an incompatible change in
ST's USB Device Library rather than because they are recent:

| | commit | date | `pUserData` |
|---|---|---|---|
| newer | `cc146d50` (`v8.1.0-15`) | 2026-08-11 | `void *pUserData[USBD_MAX_SUPPORTED_CLASS]` |
| older | `85172e2b` (`v5.4.0-22`) | 2024-01-08 | `void *pUserData` |

Both commits were built and run in both clock configurations — stock PLL3 and
patched — so four builds in all. `src/sys/system.cpp` is the only tracked file
that differs between them; checking out either commit and applying that one
edit reproduces the patched case, and leaving it alone reproduces the stock
case. The two patched builds used the earlier `PLL3P = 10` form of the patch;
the `PLL3P = 25` form below was built and run on the newer commit. The
diagnostic figures quoted under the PLL3 section below were taken on the newer
of the two.

A future libDaisy can carry an ST release that changes something neither of
these two versions covers, and break the build. If that happens, see
[`docs/PITFALLS.md`](docs/PITFALLS.md) for what to look at first, and please
open an issue.

### Optional: patch libDaisy's PLL3

**This patch is recommended, not required.** Build without it and the device
enumerates, plays and records normally. Both configurations were measured on
this board over 160-second duplex runs and both passed every check; the
difference is buffer margin, and the numbers are below rather than asserted.

**What is wrong.** The stock libDaisy clock tree does not produce 48 kHz. It
produces 48014.32 Hz, +298 ppm. That is a fixed configuration error, not drift.

**Why you cannot hear it.** Uncorrected, +298 ppm is a pitch error of 0.52
cents, against a human discrimination threshold of roughly 5–10 cents. And it
does not even reach the output: the drift corrector absorbs it by resampling.
Nothing about the audio changes.

**What it actually costs.** Two 160-second duplex runs on the same board and
host, one build against stock libDaisy and one against a patched clone, read
with `read_transport_diagnostics.py`:

| | stock | patched |
|---|---|---|
| measured clock offset | 242 ppm | 59 ppm |
| corrector authority used (rec / pb) | 50 % / 47 % | 12 % / 9 % |
| standing ring offset | 130 / 126 frames | 32 / 31 frames |
| playback ring, steady state | 102…169 of 512 | 273…320 of 512 |
| **worst margin to empty** | **102 frames (2.1 ms)** | **273 frames (5.7 ms)** |
| sawtooth period | 4.1 s | 16.9 s |
| underruns / overruns / packet-size deviations | 0 / 0 / 0 | 0 / 0 / 0 |

The patched run was taken with an earlier form of the patch, `PLL3P = 10`,
which gives the codec the same 12.288 MHz MCLK as the `PLL3P = 25` patch below.
A later 160-second run with `PLL3P = 25` gave 59 ppm, a standing offset of
31 / 31 frames, 272 frames worst margin, a 16.9 s sawtooth, and no underruns,
overruns or packet-size deviations.

**Both runs passed every check.** Nothing underran, nothing overran, no IN
packet came out the wrong size, the watchdog never fired. An unpatched build is
not broken, and on a host that delivers on time you will not hear it.

What changes is the bottom row above. A proportional controller trades a
standing offset for stability, and that offset is ring headroom spent. Stock,
the playback ring bottoms out 102 frames from empty — about two 1 ms USB
service intervals — and revisits that point every 4 seconds. Patched, it bottoms
out at 273 frames and takes 17 seconds to get there. The patch roughly triples
the worst-case cushion against a host that delivers late or in bursts.

The firmware says so itself: under stock dividers `read_transport_diagnostics.py`
adds *"At this fraction of authority, fix the clock rather than widening the
clamp"* to its standing-offset check. Patched, that line does not appear.

**So: skip the patch** to get the device running quickly, or to read the code
without building against a modified libDaisy. **Apply it** before you rely on
the device for real work, put a build on hardware you cannot measure, or
reproduce the figures in this repository — all of which assume the patched
clock. Whether 2.1 ms of cushion is enough is a property of your host, and the
only way to know is to run the diagnostic yourself.

<details>
<summary><strong>How to apply it (under a minute, plus a library rebuild)</strong></summary>

Edit `../../libDaisy/src/sys/system.cpp`, inside `System::ConfigureClocks()`,
in the block commented `// PLL 3` — lines 490–497 at `85172e2b`, one line
earlier at `cc146d50`:

| line | stock | patch to |
|---|---|---|
| 490 | `PLL3M = 6;` | `PLL3M = `**`5`**`;` |
| 491 | `PLL3N = 295;` | `PLL3N = `**`192`**`;` |
| 492 | `PLL3P = 16;` | `PLL3P = `**`25`**`;` |
| 494 | `PLL3R = 32;` | `PLL3R = `**`25`**`;` |

Four lines. `PLL3Q` and `PLL3FRACN` stay exactly as they are — `FRACN` must
remain 0.

On libDaisy v9.0.0 or later the patch also makes `SAI_192KHZ` run at 96 kHz in
every project built against that copy; see
[`docs/CLOCKING.md`](docs/CLOCKING.md).

> **There are two copies of this block in that file.** Lines 728–735 hold a
> commented-out duplicate carrying the same stock values. Patching only that
> one changes nothing. Patch the live block around line 490.

Then rebuild **the library**, not just this project — `system.cpp` compiles
into `libdaisy.a`, and relinking against the existing archive silently keeps
the old clock tree:

```sh
cd ../../libDaisy
make clean && make
cd -
make clean && make
```

Verify it took:

```sh
python3 read_sai_clock.py
```

| `PLL3DIVR` reads | meaning |
|---|---|
| **`0x180330BF`** | patch is live — SAI at **48000.0000 Hz**, 0.0 ppm |
| `0x1F031F26` | stock libDaisy — **+298 ppm**, patch not applied or library not rebuilt |

The value is derived from the constants above, so it tests what the silicon is
running rather than what the source says.

</details>

The full arithmetic is in [`docs/CLOCKING.md`](docs/CLOCKING.md).

---

## What you get

### How a sample gets across

```
 HOST                       DEVICE (STM32H750)                    ANALOGUE

        ISO OUT 0x01   ┌─────────────┐  fractional read  ┌─────┐
  DAW ──────────────►  │  pb_ring    │ ────────────────► │ SAI │──► DAC ──► out
        288 B/ms       │  512 frames │   step = 1 + tilt └─────┘
                       └─────────────┘        ▲
                              │ fill level    │ polyphase sinc,
                              └───────────────┘ 32 taps × 128 phases

        ISO IN 0x81    ┌─────────────┐  fractional read  ┌─────┐
  DAW ◄──────────────  │  rec_ring   │ ◄──────────────── │ SAI │◄── ADC ◄── in
        288 B/ms       │  512 frames │                   └─────┘
                       └─────────────┘

        BULK OUT 0x02  ┌─────────────┐
  DAW ──────────────►  │ CC table    │   stored only; this firmware
        MIDI CC        └─────────────┘   never acts on the values
```

Both rings are read at a fractional position that advances by nominally one
frame per output frame, tilted a few tens of ppm by a proportional loop on the
fill level. The USB packet is therefore always exactly 48 frames, and the
host/device clock difference is absorbed between samples rather than by
inserting or dropping one.

### 48 kHz, and only 48 kHz

The format descriptor declares a single discrete frequency, and no
sampling-frequency control is offered, so a host cannot request another rate.
This is deliberate: the patched clock chain is integer end to end with
`FRACN = 0`, giving exactly 48000.0000 Hz, verifiable from live registers
rather than taken on trust. 44.1 kHz has no integer solution on this board.

In practice 44.1 kHz material is resampled by the host, transparently and with
no configuration. To keep the host resampler out of the path, feed the device
48 kHz sources. Note this is a difference in kind from a multi-rate interface,
which would switch natively and bypass the resampler.

Why one exact rate was chosen over two approximate ones, and what supporting
44.1 would actually cost: [`docs/CLOCKING.md`](docs/CLOCKING.md).

### UAC1, not UAC2

The STM32H750 has a `USB_OTG_HS` block, but High Speed needs an external ULPI
PHY that the Daisy Seed does not carry — only `USB_OTG_FS` (12 Mbit/s) is wired.
UAC2 in practice almost always wants an explicit asynchronous feedback endpoint;
without one the usual result is jitter and drift. Hence UAC1 with adaptive
synchronisation. The ceiling that came with that choice — most host drivers at
48 kHz, and reluctant above 16 bits — was accepted going in, and 24-bit works
anyway.

### Volume is done in software

libDaisy sets no codec-level gain on this board, so USB Volume and Mute (the
Feature Unit requests) are applied in the digital path, in firmware, and
affect both playback and the recorded stream. The Pod's own headphone knob is
separate from all of this — an analogue control in the headphone amplifier,
after the codec, affecting the headphone output only.

### Drift correction without dropouts

Both rings are read at a fractional position advancing by nominally exactly one
frame, tilted a few tens of ppm by a proportional loop on the fill level. The
packet is always exactly nominal length, and the host/device clock difference
is absorbed by interpolation rather than by inserting or dropping a frame. **No
discontinuity is ever emitted.**

The interpolator is a polyphase windowed-sinc fractional-delay filter — 32 taps,
128 phase rows, 16 KB of `const float32`. Worst-case error against an ideal
fractional delay, and the audible consequence:

| | linear | Catmull-Rom | polyphase sinc |
|---|---|---|---|
| error @ 10 kHz | −13.7 dB | −24.5 dB | **−97.2 dB** |
| AM depth @ 15 kHz | 5.11 dB | 2.53 dB | **0.0003 dB** |

Rebuild with `-DFD_USE_SINC=0` to fall back to Catmull-Rom and A/B the two on
hardware.

Full tables across the band, the AM-depth measurement and why a sweeping read
phase makes it audible, the Nyquist limit at the top of the band, and the cost
in cycles and cache: [`docs/INTERPOLATION.md`](docs/INTERPOLATION.md).

---

## What's here

Scripts meant to be run directly — the two verification steps, the table
generator, and the main on-hardware validation — sit at the root. `scripts/`
holds the supporting ones.

- `usbd_audio.c` / `.h` — the UAC1 class implementation, bidirectional 24-bit
  plus MIDIStreaming.
- `usbd_audio_if.c` / `.h` — the application layer: ring buffers, Q16
  fractional clock correction, MIDI CC dispatch.
- `usbd_desc.c` / `.h` — the device, configuration and string descriptors.
- `usbd_ctlreq.c` — standard USB chapter 9 request handling.
- `usbd_conf.c` / `.h` — board support and PCD glue between the class code and
  the HAL.
- `usbd_compat.h` — accessors that let the ST-derived files above compile
  against both the pre- and post-composite versions of ST's USB Device
  Library, and refuse a field layout they do not recognise.
- `usbd_irq.c`, `main.cpp` — IRQ vector wiring and firmware entry point.
- `fd_sinc_table.h` — the generated interpolator table. Do not edit by hand;
  regenerate with `gen_table.py`.
- `gen_table.py` — generates `fd_sinc_table.h` and prints the error figures
  that go into its header comment.
- `read_chip_rev.py` — CRS `SYNCSRC` cross-check and silicon revision read.
  The `SYNCSRC` check is the useful half day to day; the revision read only
  matters on a board of unknown provenance, where ST errata ES0392 §2.2.3 can
  apply — see [`docs/CLOCKING.md`](docs/CLOCKING.md).
- `read_sai_clock.py` — derives the actual SAI sample rate from the live PLL
  registers. **This is the verification for the patch above.**
- `read_transport_diagnostics.py` — long-window on-hardware validation of the
  transport layer over USB vendor requests.
- `scripts/soak_run.py` — wraps `read_transport_diagnostics.py` for long runs
  and resolves the corrector's tilt as a function of time, which a single
  averaged figure hides.
- `tests/interp/` — numeric check of the fractional-delay reader.
  `tests/am_probe/` — drives the shipped reader and measures the amplitude
  modulation it produces. Both run on a desktop with no hardware, and between
  them they reproduce every figure quoted in
  [`INTERPOLATION.md`](docs/INTERPOLATION.md).
- `docs/` — [`PITFALLS.md`](docs/PITFALLS.md) (read before changing anything),
  [`CLOCKING.md`](docs/CLOCKING.md),
  [`INTERPOLATION.md`](docs/INTERPOLATION.md).
- `q16-drift-corrector/` — the fractional-Q16 corrector extracted as a
  standalone header with its own MIT licence and no scope restriction.
  Dependency-free; `Q16_NO_SINC` drops the optional sinc reader. Ships all
  three readers plus a self-test. Redistributable on its own.
- `THIRD_PARTY_NOTICES.md`, `LICENSE`, `LICENSE-SLA0044.txt`,
  `LICENSE-libDaisy-MIT.txt` — see below.

---

## Licensing

This repository is **not** single-licensed. It combines:

- **Original work** under **MIT** (`LICENSE`) — the transport layer, the
  application code, the descriptor bytes, the diagnostics and the documentation.
- **STMicroelectronics** USB Device Library code under **SLA0044**
  (`LICENSE-SLA0044.txt`) — notably: no Open Source Terms may attach to it, and
  it must run only on ST silicon.
- **Electro-Smith libDaisy** code under **MIT**
  (`LICENSE-libDaisy-MIT.txt`), which itself redistributes ST code internally,
  so files that came through libDaisy carry both obligations.

**`THIRD_PARTY_NOTICES.md` is the authoritative file-by-file breakdown.** Every
source header saying "see README.md's Licensing section" means: start here,
then go there for the specific file.

If you are vendoring or redistributing any file from this repository, check
`THIRD_PARTY_NOTICES.md` for that file before assuming MIT applies — for most
of the `usbd_*` files, it does not. SLA0044's ST-silicon restriction travels
with those files regardless of what you do with the MIT-covered ones.

The libDaisy patch modifies a file under libDaisy's own MIT licence,
in a tree you clone separately. This repository ships the instructions plus a
prebuilt image compiled against the patched library — not the patched source
file itself.

---

## Putting DSP on this

This repository is the transport layer only. There is no DSP here, no
processing hook, and no example of one — the `dsp_params[]` table is a MIDI
Control Change store, not an audio path.

Adding a real engine on top is a separate problem, and most of its difficulty
has nothing to do with USB: fitting the engine into the real-time budget (one
48 kHz block on this part is about 480 000 cycles, and overrunning it by any
amount is an audible dropout rather than a slowdown); numerical behaviour on
program material as against a test harness; gain staging across boundaries that
carry different signals by design; and keeping all of it inside the transport's
timing without disturbing the measured behaviour documented here.

I have built several DSP firmwares on this transport — NeuralPort (a neural amp
modeller), FmPort (an FM synthesiser), FlickPort (guitar reverb/delay/tremolo)
and Clario (parallel brightness EQ with a vari-mu compressor) among them — and
the problems above are mostly what took the time, not the USB layer.

Those firmwares are not part of this repository, which is deliberately the
transport alone.

## Host compatibility: macOS Tahoe 26

The firmware is unaffected — it is a class-compliant UAC1 device and Apple's
USB audio driver still handles it. The change is host-side: per Apple's TN3190,
macOS Tahoe 26 no longer publishes the IOAudioFamily kernel services
`AppleUSBAudioEngine`, `AppleUSBAudioDevice` and `AppleUSBAudioStream`, so
their IORegistry properties are gone. Software that finds the device through
Core Audio by name or UID is fine (the UID still begins
`AppleUSBAudioEngine:`); software that walks IORegistry looking for those
classes needs updating. Nothing in this repository does the latter.

## What the LEDs mean

- **LED1 — the record path.** Dark when idle, green while the stream is clean,
  red if a record underrun has happened since the stream opened.
- **LED2 — the playback path.** Dark when idle or while the prime gate is
  closed, green while the stream is clean, red if a playback over- or underrun
  has happened since the gate opened.

Red latches until that stream is restarted, so a glitch that happened while you
were not watching is still visible afterwards. Blue is unused.

**Normal behaviour differs by host, and none of the differences are faults:**

- **macOS** — both dark when idle, **both green during ordinary playback**.
  CoreAudio opens both directions of a full-duplex interface even if you are
  deliberately only using the output.
- **Windows** — LED2 green during playback, **LED1 stays dark**. `usbaudio.sys`
  only opens alt 1 on the interface actually in use, so playback alone does not
  start the record stream. Do not go looking for a broken record path.
  Separately, **LED2 often latches red when playback stops**, most reliably from
  a streaming source: the playback ring flags irregular OUT-packet delivery, and
  a browser feeding the Windows shared-mode mixer is close to the worst case for
  delivery regularity. Identical firmware runs on all three hosts and the latch
  logic is host-agnostic, so that difference comes from the host, not the
  transport. A device that stayed dark through it would have a broken indicator.
- **Linux** — behaves like macOS. No red-on-stop was observed.

## Bug reports

If you are using this transport and something in it does not behave the way
this repository documents, open an issue. That is not a support contract and I
make no response-time commitment, but discrepancies between the documentation
and the hardware are exactly what I want to know about.

**Before reporting a clock, drift or CRS issue, give your `read_chip_rev.py`
output and your `PLL3DIVR` value.** An unpatched PLL3 is the most common
explanation for symptoms in those areas, and it is not a defect — it is the
reduced ring margin described under [Building](#optional-patch-libdaisys-pll3).
`read_transport_diagnostics.py` identifies it in one run: a measured offset in
the hundreds of ppm rather than the tens, standing ring offset above 100 frames,
a playback band whose low end sits near 100 of 512, and the check appending
*"fix the clock rather than widening the clamp"*. If your offset is in the tens
of ppm and the standing offset is in the thirties, the clock is not your
problem and I would like to see the report.

## Integration

This firmware occupies about 86 % of the 128 KB internal flash page when built
against libDaisy `cc146d50`; against the older `85172e2b` it was about 79 %.
The difference is entirely in libDaisy and ST's middleware, not in this code.

Putting a particular DSP engine on this and taking it to a firmware that
behaves the same on macOS, Windows and Linux is separate work; if that is what
you need and it is giving you trouble, write to me and we may well find an
arrangement that suits us both.

Contact: 90daysandnights@gmail.com
