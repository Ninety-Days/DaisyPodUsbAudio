# Pitfalls: read this before changing anything

Six places in this firmware look wrong and are not, plus one symptom that
looks like a firmware bug and is not one, plus one more that lives outside this
repository entirely. Each of the six has already been "fixed" at least once,
and each fix broke the device. They are collected here because the code
comments that guard them are easy to lose in a refactor.

---

## 1. The literal `4U` in `usbd_ctlreq.c` must stay

The wIndex bound check reads `if (LOBYTE(req->wIndex) <= 4U)`. Upstream ST has
`USBD_MAX_NUM_INTERFACES` there. Restoring the macro breaks enumeration
outright.

The reason is a header shadowing problem. The `Makefile` passes
`-I../../libDaisy/src/usbd`, and libDaisy ships its own `usbd_conf.h` there with
`USBD_MAX_NUM_INTERFACES = 1U`, tuned for its CDC serial logging. This project's
own `usbd_conf.h` sets `4U`, but `usbd_ctlreq.c` resolves the include to
libDaisy's copy, not this one. With `1U` in force, every control request aimed
at an interface above interface 1 is stalled, and the device disappears — not
partially working, absent.

A warning comment sits above the `#define` in `usbd_conf.h` explaining this. If
you ever find that comment missing, it was lost in an edit and the trap is open
again.

The same shadowing applies to `USBD_SUPPORT_USER_STRING`, which has a
consequence of its own — see section 2 — and to `USBD_DEBUG_LEVEL`,
which has none.

**The interface count lives in four places**, which is the underlying reason
this trap exists at all:

| Where | Form |
|---|---|
| `usbd_audio.c` | `0x04` — `bNumInterfaces` in the descriptor bytes |
| `usbd_audio.h` | `#undef` then `#define USBD_MAX_NUM_INTERFACES 4U` |
| `usbd_conf.h` | `#define USBD_MAX_NUM_INTERFACES 4U` |
| `usbd_ctlreq.c` | the `4U` literal above |

Plus libDaisy's `1U`, which wins in translation units that do not include
`usbd_audio.h`. Changing the interface count means changing all four.

## 2. `USBD_AUDIO_GetUsrStrDesc` must stay

On libDaisy `85172e2b` it answers `GET_DESCRIPTOR(STRING)` for any index
this device does not define — 6 and above, since `usbd_desc.c` covers 0 to
5. There, removing it as dead code, or dropping the last element of
`USBD_AUDIO` to match the fourteen fields the struct appears to have, brings
back a measured failure: the control transfer goes unanswered and playback
stops until the device is replugged.

The branch that reaches it lives in `usbd_ctlreq.c` under
`#if (USBD_SUPPORT_USER_STRING == 1U)`. This project's `usbd_conf.h` sets
that to `0U`, but that file is not the one compiled against `usbd_ctlreq.c` —
libDaisy's copy is, and on `85172e2b` it sets `1U`. Same shadowing as
section 1. libDaisy commit `7651b28` renamed the macro to
`USBD_SUPPORT_USER_STRING_DESC`, the name ST's middleware has used since
v2.5.3. On `cc146d50` the old name is defined nowhere in that translation
unit, so the branch is compiled out and its `#else` stalls the request; the
stub is still compiled, but nothing calls it. It stays because the older
tree needs it.

The request comes in practice from anything that walks string indices:
`lsusb -v`, USBTreeView, and Windows asking for index `0xEE` on first
enumeration.

The `#if` around the stub tests both `USBD_SUPPORT_USER_STRING` and
`USBD_SUPPORT_USER_STRING_DESC` because ST renamed the macro, so the stub
is compiled on both trees: under the old name on `85172e2b`, under the new
one on `cc146d50`.

---

## 3. `bInCollection` must stay 3, and must list interface 3

```c
0x0B, ..., AUDIO_CONTROL_HEADER,
0x00, 0x01,          /* bcdADC = 1.00 */
0x49, 0x00,          /* wTotalLength = 73 */
0x03,                /* bInCollection = 3 */
0x01, 0x02, 0x03,    /* MIC AS, SPK AS, MIDIStreaming */
```

The AudioControl header must enumerate the MIDIStreaming interface alongside the
two AudioStreaming ones. Omitting it makes `usbccgp.sys` self-inconsistent and
Windows refuses the device, while macOS and Linux accept it.

**It is not the only Windows fix, and three of them present as the same Code
10.** `bcdUSB` was lowered from 2.00 to 1.10, because declaring 2.00 makes
Windows request a Device Qualifier that a full-speed-only device cannot answer.
The class-specific `GET_DESCRIPTOR` handler was removed, because it ignored
`wIndex` and returned the AC header block for every interface, which
`usbaudio.sys` rejects. And the PID was moved off its earlier value, because
Windows caches the PID-to-driver mapping and will reuse stale settings for a
changed descriptor set. Each is commented at its own site.

Three values move together, and all three must change if `bInCollection` ever
does: `bLength = 8 + bInCollection`, `wTotalLength = bLength + 62` (six
terminals and units), and `USB_AUDIO_CONFIG_DESC_SIZ`.

**After any edit here, recount the array.** The initialiser must be exactly
`USB_AUDIO_CONFIG_DESC_SIZ` bytes, and the `bLength` chain must consume it to
the last byte with no trailing zeros. A short array leaves a descriptor with
`bLength = 0` at the tail, which macOS tolerates and Windows does not.

## 4. `last_tx_size` is deliberately not static

`usbd_audio.c` declares it `volatile uint16_t` at file scope because
`HAL_PCD_ResumeCallback` in `usbd_conf.c` reads it to re-arm the IN pipe. It is
declared in `usbd_audio.h`. Do not make it static.

## 5. The `USBD_LL_FlushEP` pair on the IN endpoint is not redundant

`SET_INTERFACE(wIndex=1)` flushes `AUDIO_IN_EP` on both the alt=1 and the alt=0
branch. Both calls look removable and neither is.

The open path already resets `in_wr_ptr`, clears `in_iso_incomplete_pending`,
zeroes `in_buf` and re-arms with `USBD_LL_Transmit`. Adding a flush on top of
that reads as belt-and-braces. It is not: without it the pipe frequently fails
to start at all. `in_pipe_kicks` reached 7 in a single session — one per unlucky
stream open — and each failure left the IN pipe dead until the 500 ms watchdog
in `main.cpp` noticed and recovered it.

**The symptom points at the wrong subsystem entirely.** What you hear is a
dropout with a click at each edge on *playback*, at the start of a track. Every
playback counter stays clean through it: `pb_underrun_events` 0,
`pb_overrun_events` 0, `tx_size_deviations` 0. The reason is that this device is
duplex and CoreAudio services both directions of one device in a single IO
cycle. When the IN stream stops delivering, the host's cycle breaks and it feeds
digital silence to the OUT stream. An electrical loopback capture shows the gaps
as exact digital zero with 35–44 dB steps at both edges — steps no firmware path
produced, because the zeros arrived as data over USB and were played back
faithfully. Chasing this in the playback ring finds nothing, because there is
nothing there to find.

The root cause is on the close side. `alt=0` used to only clear `in_streaming`,
leaving the endpoint armed, so a packet the host had stopped polling for stayed
queued in the TX FIFO. `USBD_LL_Transmit` on top of a stale queued packet leaves
the core believing the endpoint is busy, `DataIn` never fires, and the chain is
dead. The flush on alt=1 covers the cases where the host never sends alt=0 at
all: configuration teardown, suspend, cable pull.

The evidence was a direct comparison of two code paths doing the same job.
`USBD_AUDIO_KickInPipe()` recovers the pipe reliably and differs from the open
path by exactly this one call. The watchdog was succeeding *because* it flushed.

**Do not add the symmetric flush on the SPK side.** It looks like an obvious
omission and it is a trap. `USBD_LL_FlushEP` on an IN endpoint calls
`USB_FlushTxFifo` for that endpoint's own dedicated FIFO, which is contained. On
an OUT endpoint the same call maps to `USB_FlushRxFifo`, and the OTG RX FIFO is
shared by every OUT endpoint including EP0. `SET_INTERFACE` is handled inside
the control transfer's own SETUP callback, so flushing there discards the
control transfer currently in flight. There is also no failure to fix: the OUT
path has never stalled, and `pb_overrun_events` has stayed at zero across every
run.

`in_pipe_kicks` is the metric. Read it with `read_transport_diagnostics.py`.
After a power cycle it should stay at zero across repeated stream opens; a kick
during steady streaming is the watchdog in its intended role and a different
question from this one.

---

## 6. `#ifdef USBD_MAX_SUPPORTED_CLASS` cannot tell the ST versions apart

ST's composite-device rework changed two `USBD_HandleTypeDef` fields from
pointers into arrays:

    pClass      USBD_ClassTypeDef *pClass;      ->  *pClass[USBD_MAX_SUPPORTED_CLASS]
    pUserData   void              *pUserData;   ->  *pUserData[USBD_MAX_SUPPORTED_CLASS]

The obvious way to straddle that is a preprocessor test on
`USBD_MAX_SUPPORTED_CLASS`, since the macro arrived with the rework. It does
not work, and it fails in the direction that looks like it works. In the
post-composite `usbd_def.h` the macro is defined **twice**, once as `4U` and
once as `1U`, each inside its own `#ifndef` under a different branch. Its
presence is therefore not evidence of anything, and `#ifdef` on it is a guess
wearing the costume of a check. `classId` is no better: it is a struct member,
and the preprocessor cannot see struct members at all.

`usbd_compat.h` uses `_Generic` instead, which dispatches on the field's
declared type rather than on a macro that may or may not correlate with it.

Of the four ways the old code touched these fields, three fail loudly on the
new layout — assigning to an array, and applying `->` to one, are hard errors.
The fourth is silent:

    (USBD_AUDIO_ItfTypeDef *)pdev->pUserData

On the array layout this still compiles, with no diagnostic under `-Wall
-Wextra -Wpedantic`, because the array decays to `void **` and casting `void
**` to a struct pointer is legal C. It yields a pointer to the array's own
storage rather than to what was stored in it. Nothing in the toolchain catches
this one, which is why every access goes through the accessors rather than
being fixed in place. For the same reason `usbd_compat.h` carries static
assertions that reject a third, unrecognised layout outright: a future ST
change to these fields must fail the build rather than compile into a wrong
pointer.

## 7. The headphone tone is supply modulation, not the audio path

Symptom on a Daisy Pod: a steady tone behind the music on the headphone jack.
It is faintly there as soon as the board is powered, and becomes obvious the
moment the host opens the device — opening a player and *selecting* a track is
enough, playback need not start. It fades a second or two after the
application closes, which is CoreAudio releasing the device, not the device
lingering.

**It is not in the samples.** Over a 60 s window with both streams running,
`read_transport_diagnostics.py` reports zero underruns, zero overruns and zero
`tx_size_deviations` across ~60,000 IN packets. The return recorded in a DAW is
clean. The data leaving this firmware is correct.

### The frequency is the callback rate

| `AUDIO_BLOCK_FRAMES` | callback rate | measured comb fundamental |
| --- | --- | --- |
| 48 | 1000 Hz | 1000 Hz |
| 96 | 500 Hz | 500 Hz |
| 256 | 187.5 Hz | 187.5 Hz |

Measured from the headphone output through a line input. Nothing in USB moves
with the block size, so this is not bus activity.

### The level follows the *modulation*, not the amount of work

A diagnostic build was used that runs the playback path's convolution as a
synthetic load, switchable at runtime. Board on a battery pack, **no USB cable
connected at all** — no host, no bus traffic, nothing earthed anywhere in the
setup. Headphones straight into the Pod.

| synthetic load | result |
| --- | --- |
| none | barely audible |
| one convolution per block | clearly audible |
| two convolutions per block | not louder — lower in pitch |
| the same work run continuously outside the ISR | **gone** |

The third row is the signature. Doubling the work did not raise the level, it
changed the timbre: the current waveform went from roughly a quarter duty cycle
to roughly half, which suppresses the even harmonics while leaving the
fundamental where it was. Same period, different shape.

The fourth row draws *more* average current than any other, because the CPU is
busy 100% of the time instead of a fraction of each block. Even load, no tone.

So the CPU computes a block and then idles until the next DMA interrupt, and it
is that alternation — not the total work — that modulates the current drawn
from `+3V3_A`.

(The synthetic load is lighter than real playback: it omits the volume glide,
the fade handling and the ring-pointer work in `AudioIF_PopSamples`, and the
second full interpolator pass that `copy_frames_from_rec_ring` runs in the USB
interrupt whenever the IN stream is up. It reproduces the mechanism, not the
magnitude.)

### The interference enters in the headphone amplifier

`AUDIO_OUT_L/R` reach the LINE OUT jack directly and also feed the headphone
chain: volume pot, then a TPA6110 at a gain of 1.5 (33K/22K). On Pod Rev5 the
amplifier's VCC sits on `+3V3_A` through a single 100 nF capacitor.
Electro-Smith's own reference circuit for this part powers it from `+5V_FILT` —
a rail built from three 100 µF capacitors and a 3.3 Ω resistor (or a 1 mH
inductor). At 187–1000 Hz, 100 nF does essentially nothing. The vendor
documentation is explicit that this filtering exists specifically to stop noise
generated by the Daisy's CPU from getting back into the supply.

Two measurements place the entry point after the DAC rather than before it.
Driving the *same* headphones from LINE OUT through an external amplifier
leaves the tone barely perceptible. And on LINE OUT itself, with a monitor amp
at maximum, host volume at 0 dB and the stream open, the tone sits right at the
threshold of being distinguishable at all — far more than the 1.5× the gain
difference alone would explain.

### Do not try to fix this in the firmware

Tried, and does not help: removing all computation from the USB interrupt
(`Audio_PeriodicTC` replaced by a zero-filled packet of the same length)
changes nothing, because the expensive work is in the callback. Switching the
interpolator to Catmull-Rom (`FD_USE_SINC=0`) does not improve it. Raising the
block size moves the comb down in frequency without making it inaudible, and
above ~248 frames it collides with the playback ring — see below.

A busy-loop ballast in the main loop does remove the tone, by keeping the CPU
occupied so the audio interrupt preempts work already in progress instead of
interrupting idle. It is deliberately **not** in this firmware. It pays for a
missing supply filter with the entire CPU budget, on every board — including
ones whose output stage is built correctly — and it consumes exactly the
headroom anyone layering DSP on this transport needs. The fix belongs in the
analogue supply.

**If you are on a Pod:** use LINE OUT, and drive headphones from it through an
external amplifier if you want to listen on headphones. **If you are building
your own board around a Seed:** filter the supply to the headphone amplifier as
the vendor's reference circuit shows.

### Related: block size and the playback ring

The underrun gate in `AudioIF_PopSamples` triggers at
`avail < num_frames + FD_LOOKAHEAD`, while the corrector holds the ring at
`PB_TARGET` (256 of `PB_RING_SIZE` 512). At 248 frames per callback those two
meet and the ring loses its margin: simulation shows occasional underruns and a
fill level parked well above target for tens of seconds afterwards, which
latches the red fault LED. Block sizes beyond ~240 frames need `PB_RING_SIZE`
and `PB_TARGET` raised to match.

---

## And one that is not in this repository at all

The libDaisy PLL3 patch. Without it the device runs at 48014.32 Hz — see
[`CLOCKING.md`](CLOCKING.md) and the build section of the README. Rebuilding
this project without rebuilding libDaisy itself silently keeps the old clock
tree, which makes the patch look ineffective when it is simply not linked in.
