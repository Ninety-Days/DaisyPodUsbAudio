# Third-party notices — DaisyPodUsbAudio

This repository combines original work with code originating from three
outside sources, each under its own terms. The root `LICENSE` is **MIT, and
it covers only this project's own original files** — it does not apply
repository-wide, and for most of the `usbd_*` files it does not apply at
all. This file is the authoritative per-file breakdown; the third-party
terms below are fixed and are unaffected by this project's own licence
choice.

Each entry states which upstream a file came from and under what terms.
Percentages are deliberately omitted: they vary with measurement method
and normalisation, and what governs use of a file is the identity and
license of its upstream, not a similarity score.

---

## 1. STMicroelectronics — STM32Cube USB Device Library

**License:** SLA0044 ("Ultimate Liberty"), full text in
`LICENSE-SLA0044.txt` or at <http://www.st.com/SLA0044>.
**Copyright:** © STMicroelectronics.

Two clauses are why this code is fenced off from this project's own
file set and cannot be covered by the root `LICENSE`:

- **Section 4** — the software may be used and executed solely on or with
  a processing unit manufactured by or for STMicroelectronics. Satisfied:
  the target is an STM32H750 on Daisy Seed hardware.
- **Section 5** — the software may not be placed under Open Source Terms,
  a category §5 defines broadly enough to name the BSD license explicitly.
  MIT is functionally equivalent to BSD, so this clause is the reason the
  root `LICENSE` grant is scoped to this project's own files and is never
  extended to ST's. The scoping is not a formality: applying MIT
  repository-wide would breach §5. libDaisy, which redistributes ST files
  under its own MIT, takes the same approach.

**Upstream:** <https://github.com/STMicroelectronics/stm32_mw_usb_device>

| File | Relationship to upstream |
|---|---|
| `usbd_ctlreq.c` | Copy of `Core/Src/usbd_ctlreq.c` at tag `v2.5.2` with **two deviations**, both listed here; nothing else differs. Its `@file` header reads `usbd_req.c`, which is upstream's own stale doxygen tag, kept as-is because this is otherwise a verbatim copy. **(1)** The `wIndex` bound check hardcodes `4U` where ST writes `USBD_MAX_NUM_INTERFACES`. This is not a stylistic choice — libDaisy's `usbd_conf.h` shadows this project's copy on the include path and defines that macro as `1U`, which STALLs every interface request above 1 and prevents the device from enumerating. See the warning at `USBD_MAX_NUM_INTERFACES` in `usbd_conf.h`. Restoring the macro has been tried and breaks the device. **(2)** The nine `pdev->pClass->` accesses go through `USBD_COMPAT_CLASS()` from this project's `usbd_compat.h`, which is included at the top. ST's composite rework changed `pClass` from a pointer into an array, so the upstream expression compiles against one middleware generation and not the other; the macro dispatches on the field's actual type. This is the only project header in the file and it exists solely to keep the file compiling across that boundary — do not add others. |
| `usbd_audio.h` | Descriptor-layout macros, `USBD_AUDIO_ItfTypeDef` and the class handle structure derive from `Class/AUDIO/Inc/usbd_audio.h` (v2.5.2). Modified for bidirectional 24-bit operation; note the `PeriodicTC` return type is widened from `int8_t` to `uint16_t`, without which variable-length packets are impossible. |
| `usbd_audio.c` | The `USBD_ClassTypeDef` callback contract (Init/DeInit/Setup/DataIn/DataOut/SOF/IsoIN/IsoOUT) derives from `Class/AUDIO/Src/usbd_audio.c` (v2.5.2). The descriptor bytes, both stream directions, MIDIStreaming, the ping-pong transmit path and the IN-pipe watchdog were written for this project. |
| `usbd_conf.c`, `usbd_conf.h` | Board-support / PCD glue. Reached this project through libDaisy (see §2), not directly from ST. `usbd_conf.c` additionally includes this project's `usbd_compat.h` for `USBD_LL_XFER_SIZE_T`: the size parameter of `USBD_LL_Transmit` and `USBD_LL_PrepareReceive` widened from `uint16_t` to `uint32_t` in ST's composite rework, and the definitions here must match whichever prototype the middleware declares. Two `_Static_assert`s next to those definitions verify the match. |

Also derived from ST, from a different product:

**Upstream:** STM32CubeF4,
`Projects/STM324xG_EVAL/Applications/USB_Device/AUDIO_Standalone`
(<https://github.com/STMicroelectronics/STM32CubeF4>)

| File | Relationship |
|---|---|
| `usbd_audio_if.c`, `usbd_audio_if.h` | **MIT, not ST's** — listed here only because the callback names and fops-table shape (`Audio_Init`, `Audio_DeInit`, `Audio_PlaybackCmd`, `Audio_VolumeCtl`, `Audio_MuteCtl`, `Audio_PeriodicTC`, `Audio_GetState`) match this example, confirmed by direct comparison. Those names are the linkage contract with `usbd_audio.c` and cannot differ. Every function body — the rings, the fractional Q16 correctors, sample packing, MIDI parsing — was written for this project. |
| `usbd_desc.c` | Descriptor-callback layout and the `IntToUnicode` / `Get_SerialNum` helpers follow the same example. |

### ST HAL and CMSIS

`usbd_irq.c` and all `HAL_*` / `__HAL_*` calls target ST's STM32CubeH7
HAL, licensed **BSD-3-Clause** (per STM32CubeH7's `LICENSE.md`) — a
separate and more permissive grant than SLA0044. The HAL sources are not
vendored here; they arrive via libDaisy.

---

## 2. Electro-Smith — libDaisy

**License:** MIT. **Copyright:** © 2019 Electrosmith.
**Upstream:** <https://github.com/electro-smith/libDaisy>

libDaisy redistributes ST USB Device Library files under its own MIT
repository license while those files retain ST's copyright headers
internally. Code that reached this project through libDaisy therefore
carries **both** obligations: MIT attribution to Electro-Smith, and ST's
SLA0044 terms on the ST-authored content inside. Where the two conflict,
SLA0044 §5 is the binding constraint.

| File | Relationship |
|---|---|
| `usbd_desc.h` | Apart from renamed descriptor-table symbols and the removal of the high-speed extern, a copy of `src/usbd/usbd_desc.h`. |
| `usbd_conf.c` | Derived from `src/usbd/usbd_conf.c`; the immediate upstream is libDaisy, not ST, so libDaisy's MIT terms apply alongside SLA0044 on the ST content inside. Deviations from libDaisy's copy are this project's own and exist to support a device libDaisy's copy does not describe: a third TX FIFO for the MIDI Bulk IN endpoint, the FIFO address layout that follows from it, the RxFIFO size, and the `HAL_PCD_ResumeCallback` IN-pipe re-arm. |
| `usbd_conf.h` | Same terms and same immediate upstream. Substantially rewritten for this project's endpoint and interface count. Carries the `USBD_MAX_NUM_INTERFACES` shadowing warning — see §1 above and `docs/PITFALLS.md`. |

libDaisy's MIT text must be reproduced in full in any distribution. It is
included as `LICENSE-libDaisy-MIT.txt`.

---

## 3. Development history

The project began from a UAC1 input-only firmware archive posted on the
Electro-Smith Daisy forum by `nadavb` on 2026-02-25. The archive stated no
licence. It consisted mostly of ST and libDaisy code (§1, §2), and its own
contribution — a fill-level packet-size controller — has been replaced by the
fractional Q16 reader and is not present in this tree. **No obligation arises
from it.**

## 4. `q16-drift-corrector/` — standalone, separately licensed

The `q16-drift-corrector/` subdirectory is a self-contained extraction of
this project's fractional-Q16 drift corrector (§3, above), stripped of
every dependency on USB, HAL, libDaisy, or a fixed channel count. It has
its own `LICENSE` and `README.md` and is documented there in full.

**Its licence is unscoped MIT** — unlike the root `LICENSE`, it carries no
SCOPE section, because there is no ST or libDaisy content in it to exclude.
Confirmed by construction: the file was extracted to have zero includes
beyond `<stdint.h>`. Redistributing that subdirectory alone, without the
rest of this repository, is fine and is exactly what it's for.

## 5. References consulted, no code used

- **USB Device Class Definition for Audio Devices, Release 1.0 (USB-IF)** —
  the UAC1 descriptor byte layouts in `usbd_audio.c` are dictated by this
  specification. Identical descriptor bytes across implementations reflect
  the standard, not copying.

---

## Files original to this project

`main.cpp`, `usbd_irq.c`, `usbd_compat.h`, the `read_*.py` host-side diagnostic
scripts, the tests under `tests/`, and the application layer of
`usbd_audio_if.c` — the fractional Q16 drift correctors on both rings, sample
packing, 24-bit conversion, gain ramping, diagnostics — are original work,
licensed **MIT** by the root `LICENSE`.

`usbd_compat.h` is worth a word because it is included by three ST-derived files.
It carries no ST code and reproduces none: it defines accessor macros that
dispatch on the declared type of two `USBD_HandleTypeDef` fields, plus static
assertions that reject a field layout it does not recognise. Naming a
structure's fields in order to read them is use of an interface, not
reproduction of the code that declares it, so the file is MIT and the root
`LICENSE` applies to it in full.

---

*Verification note: the file-to-upstream relationships above were checked
by direct comparison against `stm32_mw_usb_device` tag `v2.5.2`, libDaisy
`master`, and the STM32CubeF4 `AUDIO_Standalone` example, rather than
inferred from file headers.*
