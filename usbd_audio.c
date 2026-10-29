/**
 * usbd_audio.c — UAC1 bidirectional, 24-bit, + MIDIStreaming
 *
 * @attention
 * ST's portions: Copyright (c) 2015 STMicroelectronics. All rights reserved.
 * MIT License (see LICENSE at repository root) applies to this project's
 * contributions in this file, not to the file as a whole -- see that
 * file's SCOPE section for which portions are covered.
 * Its overall shape — the USBD_ClassTypeDef callback structure the
 * STM32Cube USB Device Library core requires to register any class —
 * originates from ST's Audio class example (SLA0044, Copyright (c)
 * STMicroelectronics), reached through a mono input-only community
 * reference. Essentially everything inside that shape was written or
 * substantially rewritten here: the descriptor bytes, the duplex, volume
 * and MIDI logic, the cache-coherency fix. THIRD_PARTY_NOTICES.md §3
 * carries the reasoning; LICENSE-SLA0044.txt has ST's terms.
 *
 * Interfaces:
 *   0  AudioControl
 *   1  AudioStreaming MIC  — ISO  IN  0x81
 *   2  AudioStreaming SPK  — ISO  OUT 0x01
 *   3  MIDIStreaming       — Bulk OUT 0x02, Bulk IN 0x82
 *
 * Total descriptor 269 bytes, matching USBD_MAX_NUM_INTERFACES and
 * USB_AUDIO_CONFIG_DESC_SIZ in usbd_audio.h. The byte-by-byte accounting
 * is in that header; recount it after any descriptor edit.
 *
 * Two counts here are easy to get wrong, and both have been:
 *   - Samples are 24-bit tightly packed (bSubframeSize=3), not 16-bit;
 *     every packet size derives from that.
 *   - The MS Interface Header's wTotalLength covers the header and jack
 *     descriptors only (37 bytes), not the endpoint CS descriptors, the
 *     same convention the AC Header uses. An earlier draft counted 47.
 *
 * MIDI runs one way in practice: host → device on Bulk OUT 0x02, decoded
 * by AudioIF_PushMIDI() into dsp_params and readable via
 * AudioIF_GetCCParam(). Nothing in this tree calls that from the audio
 * callback — the receive path is complete and tested but not wired to any
 * processing. The Bulk IN jack is declared and opened per spec; this
 * firmware never transmits on it.
 */
#include "usbd_audio.h"
#include "usbd_audio_if.h"
#include "usbd_ctlreq.h"
#include "usbd_compat.h"   /* pUserData/pClass shape compatibility, see header */
#include "stm32h7xx_hal.h"  /* for RCC_CRS_FLAG_* / __HAL_RCC_CRS_GET_FLAG, used in
                              * VENDOR_REQ_GetRingDiagnosticsEx -- not pulled in
                              * transitively by the other includes above */

/* Diagnostics: was SET_INTERFACE(wIndex=2, alt=1) ever received? */
volatile uint32_t spk_alt1_activations = 0U;

/* IN-pipe health. datain_completions advances ~1000 times a second while
 * the host has the mic interface open; if it freezes while in_streaming
 * is still set, the transmit chain has died and nothing inside the USB
 * stack can restart it -- see USBD_AUDIO_KickInPipe at the end of this
 * file. in_pipe_kicks counts how often the watchdog had to act. */
volatile uint32_t datain_completions = 0U;
volatile uint32_t in_pipe_kicks      = 0U;

/* Diagnostics: mirrors haudio->out_streaming so main.cpp (which has no
 * access to the private USBD_AUDIO_HandleTypeDef) can compare it directly
 * against pb_active / pb_activation_count on the LEDs, instead of only
 * inferring out_streaming indirectly from a debug probe. */
volatile uint8_t out_streaming_mirror = 0U;

#define HAUDIO  ((USBD_AUDIO_HandleTypeDef *)pdev->pClassData)

/* pUserData changed from `void *` to `void *[]` when ST added composite
 * support; ITFOPS goes through usbd_compat.h so this file builds against
 * either shape. Casting pdev->pUserData directly still *compiles* on the
 * array form but silently yields the wrong pointer -- see the header. */
#define ITFOPS  ((USBD_AUDIO_ItfTypeDef *)(*USBD_COMPAT_USERDATA_SLOT(pdev)))

static uint8_t  USBD_AUDIO_Init               (USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t  USBD_AUDIO_DeInit             (USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t  USBD_AUDIO_Setup              (USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static uint8_t *USBD_AUDIO_GetCfgDesc         (uint16_t *length);
static uint8_t *USBD_AUDIO_GetDeviceQualifierDesc(uint16_t *length);
static uint8_t  USBD_AUDIO_DataIn             (USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t  USBD_AUDIO_DataOut            (USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t  USBD_AUDIO_EP0_RxReady        (USBD_HandleTypeDef *pdev);
static uint8_t  USBD_AUDIO_EP0_TxReady        (USBD_HandleTypeDef *pdev);
static uint8_t  USBD_AUDIO_SOF                (USBD_HandleTypeDef *pdev);
static uint8_t  USBD_AUDIO_IsoINIncomplete    (USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t  USBD_AUDIO_IsoOutIncomplete   (USBD_HandleTypeDef *pdev, uint8_t epnum);
static void     AUDIO_REQ_GetCurrent          (USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static void     AUDIO_REQ_SetCurrent          (USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static void     AUDIO_REQ_GetRange            (USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static void     VENDOR_REQ_GetRingDiagnostics  (USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static void     VENDOR_REQ_GetRingDiagnosticsEx(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static void     VENDOR_REQ_GetChipRev          (USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static void     VENDOR_REQ_GetClockTree        (USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static void     VENDOR_REQ_GetTransportHealth   (USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);

/* Owned by usbd_audio_if.c; read by VENDOR_REQ_GetTransportHealth. */
extern volatile uint32_t tx_size_deviations;
extern volatile uint32_t pb_read_advances;

/* GET_DESCRIPTOR(STRING) for any index this device does not define -- 6 and
   above; 0..5 are answered by usbd_desc.c. libDaisy's usbd_conf.h is the copy
   that reaches usbd_ctlreq.c and it sets USBD_SUPPORT_USER_STRING to 1U, so
   the branch there calls this slot rather than stalling. Leaving the slot
   empty leaves the control transfer unanswered and kills playback until the
   device is replugged -- measured, not inferred. Stall instead, which is what
   the spec asks for and what the #else in usbd_ctlreq.c would have done.
   Both macro spellings are tested because ST renamed it; if a future libDaisy
   guards the struct field with only one of them, this fails to compile rather
   than silently reverting. */
#if (defined(USBD_SUPPORT_USER_STRING)      && (USBD_SUPPORT_USER_STRING      == 1U)) || \
    (defined(USBD_SUPPORT_USER_STRING_DESC) && (USBD_SUPPORT_USER_STRING_DESC == 1U))
static uint8_t *USBD_AUDIO_GetUsrStrDesc(USBD_HandleTypeDef *pdev,
                                         uint8_t index, uint16_t *length)
{
    (void)index;
    USBD_CtlError(pdev, &pdev->request);
    *length = 0U;
    return NULL;
}
#endif

USBD_ClassTypeDef USBD_AUDIO = {
    USBD_AUDIO_Init, USBD_AUDIO_DeInit, USBD_AUDIO_Setup,
    USBD_AUDIO_EP0_TxReady, USBD_AUDIO_EP0_RxReady,
    USBD_AUDIO_DataIn, USBD_AUDIO_DataOut, USBD_AUDIO_SOF,
    USBD_AUDIO_IsoINIncomplete, USBD_AUDIO_IsoOutIncomplete,
    USBD_AUDIO_GetCfgDesc, USBD_AUDIO_GetCfgDesc, USBD_AUDIO_GetCfgDesc,
    USBD_AUDIO_GetDeviceQualifierDesc,
#if (defined(USBD_SUPPORT_USER_STRING)      && (USBD_SUPPORT_USER_STRING      == 1U)) || \
    (defined(USBD_SUPPORT_USER_STRING_DESC) && (USBD_SUPPORT_USER_STRING_DESC == 1U))
    USBD_AUDIO_GetUsrStrDesc,
#endif
};

/* =========================================================================
 * Configuration descriptor — 269 bytes, 4 interfaces, 24-bit UAC1 + MIDI
 * ========================================================================= */

__ALIGN_BEGIN static uint8_t USBD_AUDIO_CfgDesc[USB_AUDIO_CONFIG_DESC_SIZ] __ALIGN_END =
{
    /* Configuration (9) */
    0x09, USB_DESC_TYPE_CONFIGURATION,
    LOBYTE(USB_AUDIO_CONFIG_DESC_SIZ), HIBYTE(USB_AUDIO_CONFIG_DESC_SIZ),
    0x04,           /* bNumInterfaces = 4 (AC, MIC AS, SPK AS, MIDIStreaming) */
    0x01, 0x00, 0xC0, 50U,   /* bMaxPower = 50 x 2mA = 100mA; a Daisy Seed
                                draws ~70mA under load */

    /* AudioControl Interface 0, alt 0 (9) */
    0x09, USB_DESC_TYPE_INTERFACE,
    0x00, 0x00, 0x00,
    USB_DEVICE_CLASS_AUDIO, AUDIO_SUBCLASS_AUDIOCONTROL,
    AUDIO_PROTOCOL_UNDEFINED, 0x00,

    /* AC Header (11) — bInCollection=3: MIC AS + SPK AS + MIDIStreaming.
     *
     * Interface 3 (MIDIStreaming) MUST be listed here. usbccgp.sys groups
     * every Audio-class interface of this device into ONE function -- there
     * is no separate MI_03 child node on Windows, only MI_00 -- so
     * usbaudio.sys owns interfaces 0,1,2,3 and then finds interface 3 absent
     * from the collection that its own AudioControl interface declares. That
     * is a self-inconsistent configuration and it fails StartDevice with
     * STATUS_DEVICE_CONFIGURATION_ERROR (0xC0000182), surfacing as Code 10 /
     * CM_PROB_FAILED_START with only the two MIDI bulk pipes open and no ISO
     * pipes at all.
     *
     * macOS parses AS/MS interfaces independently of this list and never
     * noticed. USB MIDI 1.0 requires the MS interface to be part of the
     * Audio Interface Collection, so this is a conformance fix, not a
     * Windows workaround. Do not "simplify" bInCollection back to 2.
     *
     * bLength, wTotalLength and USB_AUDIO_CONFIG_DESC_SIZ all move together
     * with bInCollection: bLength = 8 + bInCollection, wTotalLength =
     * bLength + 62 (the six terminals/units below), CONFIG_DESC_SIZ = 269. */
    0x0B, AUDIO_INTERFACE_DESCRIPTOR_TYPE, AUDIO_CONTROL_HEADER,
    0x00, 0x01,     /* bcdADC = 1.00 */
    0x49, 0x00,     /* wTotalLength = 73 */
    0x03,           /* bInCollection = 3 */
    0x01, 0x02, 0x03,   /* baInterfaceNr = MIC AS, SPK AS, MIDIStreaming */

    /* MIC Input Terminal ID=1 (12) */
    0x0C, AUDIO_INTERFACE_DESCRIPTOR_TYPE, AUDIO_CONTROL_INPUT_TERMINAL,
    0x01, 0x01, 0x02, 0x00, 0x02, 0x03, 0x00, 0x00, 0x00,

    /* MIC Feature Unit ID=2 (10) — bmaControls[0]=0x03: bit0=Mute,
     * bit1=Volume. Was 0x01 (Mute only) -- same root cause as the SPK
     * Feature Unit fix: the device itself declared "no volume control
     * exists here", so macOS's input-level slider was greyed out and
     * pinned left, independent of anything in the record data path. */
    0x0A, AUDIO_INTERFACE_DESCRIPTOR_TYPE, AUDIO_CONTROL_FEATURE_UNIT,
    0x02, 0x01, 0x01,
    0x03, 0x00, 0x00, 0x00,

    /* MIC Output Terminal ID=3 (9) */
    0x09, AUDIO_INTERFACE_DESCRIPTOR_TYPE, AUDIO_CONTROL_OUTPUT_TERMINAL,
    0x03, 0x01, 0x01, 0x00, 0x02, 0x00,

    /* SPK Input Terminal ID=4 (12) */
    0x0C, AUDIO_INTERFACE_DESCRIPTOR_TYPE, AUDIO_CONTROL_INPUT_TERMINAL,
    0x04, 0x01, 0x01, 0x00, 0x02, 0x03, 0x00, 0x00, 0x00,

    /* SPK Feature Unit ID=5 (10) — bmaControls[0]=0x03 (master: Mute+
     * Volume), bmaControls[1]=0x03 (Left: Mute+Volume), bmaControls[2]=
     * 0x03 (Right: Mute+Volume). Left/Right were 0x00/0x00 (no controls
     * at all) -- that is why the per-channel L/R sliders in Audio MIDI
     * Setup were greyed out and pinned left: the device itself declared
     * "left/right channels have no controls", so the host never even
     * sent GET_MIN/MAX/RES/CUR for them. This does not change bLength/
     * bControlSize, only which controls this Feature Unit claims to
     * support per channel. */
    0x0A, AUDIO_INTERFACE_DESCRIPTOR_TYPE, AUDIO_CONTROL_FEATURE_UNIT,
    0x05, 0x04, 0x01,
    0x03, 0x03, 0x03, 0x00,

    /* SPK Output Terminal ID=6 (9) */
    0x09, AUDIO_INTERFACE_DESCRIPTOR_TYPE, AUDIO_CONTROL_OUTPUT_TERMINAL,
    0x06, 0x01, 0x03, 0x00, 0x05, 0x00,

    /* ── Interface 1: MIC AudioStreaming ── */
    0x09, USB_DESC_TYPE_INTERFACE, 0x01, 0x00, 0x00,
    USB_DEVICE_CLASS_AUDIO, AUDIO_SUBCLASS_AUDIOSTREAMING,
    AUDIO_PROTOCOL_UNDEFINED, 0x00,

    0x09, USB_DESC_TYPE_INTERFACE, 0x01, 0x01, 0x01,
    USB_DEVICE_CLASS_AUDIO, AUDIO_SUBCLASS_AUDIOSTREAMING,
    AUDIO_PROTOCOL_UNDEFINED, 0x00,

    0x07, AUDIO_INTERFACE_DESCRIPTOR_TYPE, AUDIO_STREAMING_GENERAL,
    0x03, 0x01, 0x01, 0x00,

    /* bSubframeSize=3, bBitResolution=24 (0x18) -- 24-bit tightly packed,
     * not 4-byte "24-in-32". Was 0x02/0x10 (16-bit). */
    0x0B, AUDIO_INTERFACE_DESCRIPTOR_TYPE, AUDIO_STREAMING_FORMAT_TYPE,
    AUDIO_FORMAT_TYPE_I, 0x02, 0x03, 0x18, 0x01,
    0x80, 0xBB, 0x00,

    /* MIC endpoint: bmAttributes = 0x09 (Isochronous + Adaptive).

       Adaptive is the correct type for a source with no feedback channel:
       the device hands back however many frames rec_ring actually holds
       each 1 ms tick, which is what Audio_PeriodicTC already does. An Asynchronous source (0x05) would
       owe the host explicit clock information we do not implement, and
       macOS's CoreAudio will not open a stream it cannot trust -- it never
       sends SET_INTERFACE(wIndex=1, wValue=1), so AUDIO_IN_TC never runs
       even though the device still appears selectable and the SPK side
       still opens.

       Tried and rejected: 0x05 here did not fix Windows Code 10, and it
       split the device into two nodes on macOS. The Windows fix is
       bInCollection in the AC header, above. */
    0x09, USB_DESC_TYPE_ENDPOINT, AUDIO_IN_EP,
    0x09, LOBYTE(AUDIO_PACKET_MAX), HIBYTE(AUDIO_PACKET_MAX),
    0x01, 0x00, 0x00,

    0x07, AUDIO_ENDPOINT_DESCRIPTOR_TYPE, AUDIO_ENDPOINT_GENERAL,
    0x00, 0x00, 0x00, 0x00,

    /* ── Interface 2: SPK AudioStreaming ── */
    0x09, USB_DESC_TYPE_INTERFACE, 0x02, 0x00, 0x00,
    USB_DEVICE_CLASS_AUDIO, AUDIO_SUBCLASS_AUDIOSTREAMING,
    AUDIO_PROTOCOL_UNDEFINED, 0x00,

    0x09, USB_DESC_TYPE_INTERFACE, 0x02, 0x01, 0x01,
    USB_DEVICE_CLASS_AUDIO, AUDIO_SUBCLASS_AUDIOSTREAMING,
    AUDIO_PROTOCOL_UNDEFINED, 0x00,

    0x07, AUDIO_INTERFACE_DESCRIPTOR_TYPE, AUDIO_STREAMING_GENERAL,
    0x04, 0x01, 0x01, 0x00,

    /* bSubframeSize=3, bBitResolution=24 (0x18) -- 24-bit tightly packed,
     * not 4-byte "24-in-32". Was 0x02/0x10 (16-bit). */
    0x0B, AUDIO_INTERFACE_DESCRIPTOR_TYPE, AUDIO_STREAMING_FORMAT_TYPE,
    AUDIO_FORMAT_TYPE_I, 0x02, 0x03, 0x18, 0x01,
    0x80, 0xBB, 0x00,

    /* SPK endpoint: bmAttributes = 0x09 (Isochronous + Adaptive).
       Adaptive is the correct synchronization type for a sink with no
       feedback channel, and this device has none (bSynchAddress = 0
       below). The USB Audio spec requires an Asynchronous sink to
       provide an explicit feedback endpoint reporting its actual sample
       rate; declaring 0x05 without one produces a device CoreAudio does
       not accept as a valid async sink, and it then never sends
       SET_INTERFACE at all -- no error, no stream (macOS 15.2 Sequoia;
       Windows is more lenient and tolerates the mismatch). Adaptive says what
       Audio_PlaybackData actually does: consume packets at whatever rate
       the host sends them.

       Do not "fix" this by adding an explicit feedback endpoint. Three
       attempts, three different failures: refusal to enumerate at all;
       audio working for two DAW sessions then absent from the third; and
       fb_transmits frozen at 5 while the host went on trusting a stale
       -2021 ppm figure, audible as clicks even in system playback. The
       clock difference is absorbed by the fractional reader instead. */
    0x09, USB_DESC_TYPE_ENDPOINT, AUDIO_OUT_EP,
    0x09, LOBYTE(AUDIO_PACKET_MAX), HIBYTE(AUDIO_PACKET_MAX),
    0x01, 0x00, 0x00,

    0x07, AUDIO_ENDPOINT_DESCRIPTOR_TYPE, AUDIO_ENDPOINT_GENERAL,
    0x00, 0x00, 0x00, 0x00,

    /* ── Interface 3: MIDIStreaming ──
     * Byte layout: 9(std IF) + 7(MS header) + 6+6(IN jacks) + 9+9(OUT
     * jacks) + 9+5(Bulk OUT std+CS) + 9+5(Bulk IN std+CS) = 74 bytes.
     * 195 (AC + MIC/SPK blocks above) + 74 = 269 = USB_AUDIO_CONFIG_DESC_SIZ. */

    /* Standard MS Interface Descriptor (9) */
    0x09, USB_DESC_TYPE_INTERFACE, 0x03, 0x00, 0x02,
    USB_DEVICE_CLASS_AUDIO, AUDIO_SUBCLASS_MIDISTREAMING,
    AUDIO_PROTOCOL_UNDEFINED, 0x00,

    /* Class-Specific MS Interface Header (7) — wTotalLength=37: MS
     * Header (7) + both IN jacks (6+6) + both OUT jacks (9+9) = 37, NOT
     * counting the interface descriptor itself or the endpoint
     * descriptors below (per spec, wTotalLength here covers only the
     * MS-class-specific block — same convention as the AC Header's
     * wTotalLength above, which only covers AC class-specific
     * descriptors, not endpoints). */
    0x07, AUDIO_INTERFACE_DESCRIPTOR_TYPE, MS_HEADER,
    0x00, 0x01,     /* bcdMSC = 1.00 */
    0x25, 0x00,     /* wTotalLength = 37 */

    /* MIDI IN Jack, Embedded, ID=1 (6) — this device's own Bulk IN
     * endpoint as a MIDI source (reserved for future meter/feedback
     * use; not driven by any logic today). */
    0x06, AUDIO_INTERFACE_DESCRIPTOR_TYPE, MIDI_IN_JACK,
    MIDI_JACK_EMBEDDED, 0x01, 0x00,

    /* MIDI IN Jack, External, ID=2 (6) — the HOST as a MIDI source.
     * This is what MIDI OUT Jack ID=3 below sources from. */
    0x06, AUDIO_INTERFACE_DESCRIPTOR_TYPE, MIDI_IN_JACK,
    MIDI_JACK_EXTERNAL, 0x02, 0x00,

    /* MIDI OUT Jack, Embedded, ID=3 (9) — the actual Bulk OUT endpoint
     * (host -> device, where CC messages from the plugin arrive).
     * Sources from External IN Jack ID=2 (the host): data arriving
     * from the host lands on an embedded OUT jack wired to the
     * device's Bulk OUT EP. */
    0x09, AUDIO_INTERFACE_DESCRIPTOR_TYPE, MIDI_OUT_JACK,
    MIDI_JACK_EMBEDDED, 0x03, 0x01, 0x02, 0x01, 0x00,

    /* MIDI OUT Jack, External, ID=4 (9) — the HOST as a MIDI sink,
     * sourced from this device's own Embedded IN Jack ID=1. Not
     * currently used (nothing pushes data through Jack 1), but
     * required for a spec-complete, symmetric jack topology that
     * class-compliant MIDI hosts expect to parse without special-
     * casing a device that only implements one direction. */
    0x09, AUDIO_INTERFACE_DESCRIPTOR_TYPE, MIDI_OUT_JACK,
    MIDI_JACK_EXTERNAL, 0x04, 0x01, 0x01, 0x01, 0x00,

    /* Standard Bulk OUT Endpoint (9) — MIDI_OUT_EP = 0x02 */
    0x09, USB_DESC_TYPE_ENDPOINT, MIDI_OUT_EP,
    0x02, LOBYTE(MIDI_EP_SIZE), HIBYTE(MIDI_EP_SIZE), 0x00,
    0x00, 0x00,

    /* Class-Specific MS Bulk OUT Endpoint (5) — associates this
     * endpoint with Embedded OUT Jack ID=3, the jack that data arriving
     * from the host (via External IN Jack ID=2) is considered to
     * terminate at. baAssocJackID for a Bulk OUT endpoint must reference
     * an embedded OUT jack, never an embedded IN jack; ID=1 is the
     * embedded IN jack and belongs to the Bulk IN descriptor below. */
    0x05, CS_ENDPOINT_DESCRIPTOR_TYPE, MS_GENERAL,
    0x01, 0x03,

    /* Standard Bulk IN Endpoint (9) — MIDI_IN_EP = 0x82 */
    0x09, USB_DESC_TYPE_ENDPOINT, MIDI_IN_EP,
    0x02, LOBYTE(MIDI_EP_SIZE), HIBYTE(MIDI_EP_SIZE), 0x00,
    0x00, 0x00,

    /* Class-Specific MS Bulk IN Endpoint (5) — associates with
     * Embedded IN Jack ID=1 (this device's own MIDI source; reserved,
     * unused today). Was pointing at ID=3 -- same swap as the Bulk OUT
     * descriptor above, corrected here to match. */
    0x05, CS_ENDPOINT_DESCRIPTOR_TYPE, MS_GENERAL,
    0x01, 0x01,
};

__ALIGN_BEGIN static uint8_t USBD_AUDIO_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END =
{
    USB_LEN_DEV_QUALIFIER_DESC, USB_DESC_TYPE_DEVICE_QUALIFIER,
    0x00, 0x02, 0x00, 0x00, 0x00, 0x40, 0x01, 0x00,
};

/* NOT static: HAL_PCD_ResumeCallback in usbd_conf.c re-arms the IN pipe
 * after a suspend/resume cycle and needs to read the last transmitted
 * packet size to do it. That resume fix predates this comment; the
 * extern there was declared `volatile uint16_t`, this was plain
 * `uint16_t`, and the file was still `static` -- three problems that
 * only showed up at link time, not at compile time, because a bare
 * `extern` declaration type-checks locally with nothing to check it
 * against. Matching the qualifier here rather than loosening the
 * extern: the value is written from ISR context (USBD_AUDIO_DataIn) and
 * read from ISR context (HAL_PCD_ResumeCallback runs from the same USB
 * ISR), so `volatile` is correct and must match on both sides. */
volatile uint16_t       last_tx_size              = AUDIO_OUT_PACKET;
static volatile uint8_t in_iso_incomplete_pending = 0U;

/* -----------------------------------------------------------------------
 * Static, 32-byte-aligned double-buffer storage for the IN (mic) and OUT
 * (speaker) endpoints. See usbd_audio.h's comment on the handle struct
 * for why these live here as separate arrays rather than inline in
 * USBD_AUDIO_HandleTypeDef: USBD_malloc gives no alignment guarantee
 * beyond the C library default, so a 32-byte-aligned array nested inside
 * a non-32-byte-aligned struct would still not have a guaranteed-aligned
 * runtime address. These top-level statics, with __ALIGN_BEGIN/
 * __ALIGN_END (the same mechanism already used for USBD_AUDIO_CfgDesc
 * above), are guaranteed 32-byte aligned by the linker regardless of
 * where the handle struct itself ends up. */
__ALIGN_BEGIN static uint8_t USBD_AUDIO_InBufStorage[AUDIO_EP_BUF_SIZE]  __ALIGN_END;
__ALIGN_BEGIN static uint8_t USBD_AUDIO_OutBufStorage[AUDIO_EP_BUF_SIZE] __ALIGN_END;

/* ----------------------------------------------------------------------- */
static uint8_t USBD_AUDIO_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
    (void)cfgidx;

    USBD_AUDIO_HandleTypeDef *haudio =
        (USBD_AUDIO_HandleTypeDef *)USBD_malloc(sizeof(USBD_AUDIO_HandleTypeDef));
    if(haudio == NULL) { pdev->pClassData = NULL; return USBD_FAIL; }
    pdev->pClassData = haudio;

    haudio->in_buf  = USBD_AUDIO_InBufStorage;
    haudio->out_buf = USBD_AUDIO_OutBufStorage;

    USBD_LL_OpenEP(pdev, AUDIO_IN_EP,  USBD_EP_TYPE_ISOC, AUDIO_PACKET_MAX);
    pdev->ep_in[AUDIO_IN_EP  & 0x7FU].is_used = 1U;

    USBD_LL_OpenEP(pdev, AUDIO_OUT_EP, USBD_EP_TYPE_ISOC, AUDIO_PACKET_MAX);
    pdev->ep_out[AUDIO_OUT_EP & 0x7FU].is_used = 1U;

    /* MIDI Bulk endpoints -- unlike the Audio streaming interfaces above,
     * MIDIStreaming (interface 3) has no alt-setting mechanism: per the
     * USB MIDI 1.0 spec, a class-compliant MIDI interface is declared at
     * bAlternateSetting=0 with bNumEndpoints=2 directly, so there is no
     * SET_INTERFACE(alt=1) the host needs to send to "start" it -- these
     * endpoints are simply active as soon as the device is configured,
     * same as any ordinary Bulk-only interface would be. */
    USBD_LL_OpenEP(pdev, MIDI_OUT_EP, USBD_EP_TYPE_BULK, MIDI_EP_SIZE);
    pdev->ep_out[MIDI_OUT_EP & 0x7FU].is_used = 1U;

    USBD_LL_OpenEP(pdev, MIDI_IN_EP, USBD_EP_TYPE_BULK, MIDI_EP_SIZE);
    pdev->ep_in[MIDI_IN_EP & 0x7FU].is_used = 1U;

    haudio->alt_setting_in  = 0U;
    haudio->alt_setting_out = 0U;
    haudio->in_wr_ptr       = 0U;
    haudio->out_wr_ptr      = 0U;
    haudio->in_streaming    = 0U;
    haudio->out_streaming   = 0U;
    out_streaming_mirror    = 0U;
    memset(haudio->in_buf,  0, AUDIO_EP_BUF_SIZE);
    memset(haudio->out_buf, 0, AUDIO_EP_BUF_SIZE);
    memset(haudio->midi_buf, 0, MIDI_EP_SIZE);

    in_iso_incomplete_pending = 0U;
    last_tx_size              = AUDIO_OUT_PACKET;

    if(ITFOPS->Init(USBD_AUDIO_FREQ, AUDIO_DEFAULT_VOLUME, 0U) != 0)
        return USBD_FAIL;

    USBD_LL_Transmit(pdev, AUDIO_IN_EP, haudio->in_buf, AUDIO_OUT_PACKET);

    USBD_LL_PrepareReceive(pdev, AUDIO_OUT_EP,
                           &haudio->out_buf[haudio->out_wr_ptr], AUDIO_PACKET_MAX);

    /* Arm the MIDI Bulk OUT endpoint to receive its first packet. Without
     * this call the endpoint is open but the HAL has nothing queued to
     * write incoming data into, so the host's first MIDI packet would
     * simply have nowhere to land. Re-armed again at the end of every
     * MIDI DataOut event (see USBD_AUDIO_DataOut's MIDI branch) so it's
     * always ready for the next packet, mirroring the same "receive,
     * process, immediately re-arm" pattern already used for AUDIO_OUT_EP
     * above. */
    USBD_LL_PrepareReceive(pdev, MIDI_OUT_EP, haudio->midi_buf, MIDI_EP_SIZE);

    return USBD_OK;
}

static uint8_t USBD_AUDIO_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
    (void)cfgidx;

    USBD_LL_CloseEP(pdev, AUDIO_IN_EP);
    pdev->ep_in[AUDIO_IN_EP  & 0x7FU].is_used = 0U;
    USBD_LL_CloseEP(pdev, AUDIO_OUT_EP);
    pdev->ep_out[AUDIO_OUT_EP & 0x7FU].is_used = 0U;

    USBD_LL_CloseEP(pdev, MIDI_OUT_EP);
    pdev->ep_out[MIDI_OUT_EP & 0x7FU].is_used = 0U;
    USBD_LL_CloseEP(pdev, MIDI_IN_EP);
    pdev->ep_in[MIDI_IN_EP & 0x7FU].is_used = 0U;

    if(pdev->pClassData != NULL) {
        ITFOPS->DeInit(0U);
        USBD_free(pdev->pClassData);
        pdev->pClassData = NULL;
    }
    return USBD_OK;
}

/* ----------------------------------------------------------------------- */
static uint8_t USBD_AUDIO_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
    USBD_AUDIO_HandleTypeDef *haudio = HAUDIO;
    uint16_t           status_info = 0U;
    USBD_StatusTypeDef ret         = USBD_OK;

    if(haudio == NULL) return USBD_FAIL;

    switch(req->bmRequest & USB_REQ_TYPE_MASK)
    {
        case USB_REQ_TYPE_CLASS:
            switch(req->bRequest) {
                case AUDIO_REQ_GET_CUR: AUDIO_REQ_GetCurrent(pdev, req); break;
                case AUDIO_REQ_SET_CUR: AUDIO_REQ_SetCurrent(pdev, req); break;
                case AUDIO_REQ_GET_MIN:
                case AUDIO_REQ_GET_MAX:
                case AUDIO_REQ_GET_RES:
                    AUDIO_REQ_GetRange(pdev, req);
                    break;
                default: USBD_CtlError(pdev, req); ret = USBD_FAIL; break;
            }
            break;

        case USB_REQ_TYPE_VENDOR:
            switch(req->bRequest) {
                case VENDOR_REQ_GET_RING_DIAGNOSTICS:
                    VENDOR_REQ_GetRingDiagnostics(pdev, req);
                    break;
                case VENDOR_REQ_GET_RING_DIAGNOSTICS_EX:
                    VENDOR_REQ_GetRingDiagnosticsEx(pdev, req);
                    break;
                case VENDOR_REQ_GET_CHIP_REV:
                    VENDOR_REQ_GetChipRev(pdev, req);
                    break;
                case VENDOR_REQ_GET_TRANSPORT_HEALTH:
                    VENDOR_REQ_GetTransportHealth(pdev, req);
                    break;
                case VENDOR_REQ_GET_CLOCK_TREE:
                    VENDOR_REQ_GetClockTree(pdev, req);
                    break;
                default: USBD_CtlError(pdev, req); ret = USBD_FAIL; break;
            }
            break;

        case USB_REQ_TYPE_STANDARD:
            switch(req->bRequest)
            {
                case USB_REQ_GET_STATUS:
                    if(pdev->dev_state == USBD_STATE_CONFIGURED)
                        USBD_CtlSendData(pdev, (uint8_t *)&status_info, 2U);
                    else { USBD_CtlError(pdev, req); ret = USBD_FAIL; }
                    break;

                case USB_REQ_GET_DESCRIPTOR:
                    /* Windows fix: the previous handler intercepted this
                       request and ALWAYS returned the AC-header block (offset
                       +18, fixed 72 bytes), ignoring req->wIndex entirely and
                       even matching on the wrong descriptor type (0x21 is the
                       HID descriptor type; class-specific audio descriptors are
                       0x24/0x25). All class-specific AC/AS/MS descriptors are
                       already embedded in the Configuration Descriptor, which
                       the host reads in full -- so this per-interface handler
                       is not needed. Worse, usbaudio.sys DOES issue class-
                       specific GET_DESCRIPTOR with a real wIndex (1,2,...) and
                       got the same AC block back for every interface, which it
                       rejects as "configuration parameters incorrect" -> Code
                       10. (macOS/Linux never send this and read from the full
                       config descriptor, so the bug was invisible there.)
                       Correct behaviour: don't special-case it -- STALL so the
                       host falls back to the config descriptor it already has. */
                    USBD_CtlError(pdev, req);
                    ret = USBD_FAIL;
                    break;

                case USB_REQ_GET_INTERFACE:
                    if(pdev->dev_state == USBD_STATE_CONFIGURED) {
                        uint8_t alt = 0U;
                        if(req->wIndex == 1U) alt = (uint8_t)haudio->alt_setting_in;
                        else if(req->wIndex == 2U) alt = (uint8_t)haudio->alt_setting_out;
                        haudio->control.data[0] = alt;
                        USBD_CtlSendData(pdev, haudio->control.data, 1U);
                    } else { USBD_CtlError(pdev, req); ret = USBD_FAIL; }
                    break;

                case USB_REQ_SET_INTERFACE:
                    if(pdev->dev_state == USBD_STATE_CONFIGURED)
                    {
                        if(req->wIndex == 1U)
                        {
                            uint8_t prev_alt_in = haudio->alt_setting_in;
                            haudio->alt_setting_in = req->wValue;

                            if(req->wValue == 1U) {
                                if(prev_alt_in != 1U) {
                                    /* Record-only, never AudioIF_Reset():
                                       the mic interface must not touch
                                       the speaker path. The host
                                       negotiates the two independently,
                                       so a mic-side alt-setting change
                                       can arrive mid-playback, and a
                                       shared reset would discard the
                                       speaker buffer and force a full
                                       re-prime for no reason. */
                                    AudioIF_ResetRecord();
                                }
                                haudio->in_streaming      = 1U;
                                haudio->in_wr_ptr         = 0U;
                                last_tx_size              = AUDIO_OUT_PACKET;
                                in_iso_incomplete_pending = 0U;
                                memset(haudio->in_buf, 0, AUDIO_EP_BUF_SIZE);
                                /* Flush before re-arming, and again
                                   on close below. A packet the host
                                   stopped polling for stays queued in
                                   the TX FIFO otherwise, and Transmit on
                                   top of it leaves the core thinking the
                                   endpoint is busy: DataIn never fires
                                   and the pipe is dead until the
                                   watchdog in main.cpp clears it. IN
                                   only -- see docs/PITFALLS.md item 5
                                   for why the SPK side must not get the
                                   symmetric call. */
                                USBD_LL_FlushEP(pdev, AUDIO_IN_EP);
                                USBD_LL_Transmit(pdev, AUDIO_IN_EP,
                                                 haudio->in_buf, AUDIO_OUT_PACKET);
                            } else {
                                haudio->in_streaming = 0U;
                                /* Leave nothing queued behind; the
                                   endpoint stays armed after the host
                                   stops polling it. Pairs with the flush
                                   in the alt=1 branch above. */
                                USBD_LL_FlushEP(pdev, AUDIO_IN_EP);
                                in_iso_incomplete_pending = 0U;
                                /* The alt=1 path above has always
                                   been guarded on a real transition, but
                                   this one reset unconditionally, so a
                                   repeated SET_INTERFACE(alt=0) -- which
                                   hosts do send -- re-ran it every time.
                                   Same guard here, and record-only. */
                                if(prev_alt_in != 0U) {
                                    AudioIF_ResetRecord();
                                }
                            }
                        }
                        else if(req->wIndex == 2U)
                        {
                            uint8_t prev_alt = haudio->alt_setting_out;
                            haudio->alt_setting_out = req->wValue;

                            if(req->wValue == 1U) {
                                spk_alt1_activations++;
                                /* Only reset the ring on a real 0->1
                                   transition. macOS can resend
                                   SET_INTERFACE(wIndex=2, alt=1) more
                                   than once while a stream is already
                                   running (renegotiation/keepalive);
                                   resetting on every such repeat wiped
                                   pb_write/pb_read back to 0 each time,
                                   so the same few frames that had
                                   accumulated since the last reset kept
                                   replaying -- heard as a tiny, pitch-
                                   stable loop that never advanced. */
                                if(prev_alt != 1U) {
                                    AudioIF_ResetPlayback();
                                }
                                haudio->out_streaming = 1U;
                                out_streaming_mirror  = 1U;
                                haudio->out_wr_ptr    = 0U;
                                memset(haudio->out_buf, 0, AUDIO_EP_BUF_SIZE);
                                AudioIF_SetPlaybackActive(1U);
                                USBD_LL_PrepareReceive(pdev, AUDIO_OUT_EP,
                                                       haudio->out_buf, AUDIO_PACKET_MAX);
                            } else {
                                haudio->out_streaming = 0U;
                                out_streaming_mirror  = 0U;
                                if(prev_alt != 0U) {
                                    /* SetPlaybackActive(0) already zeroes
                                       pb_write/pb_read/pb_primed; the
                                       reset adds the ring memset so no
                                       stale audio survives into the next
                                       session. */
                                    AudioIF_SetPlaybackActive(0U);
                                    AudioIF_ResetPlayback();
                                }
                            }
                        }
                        else if(req->wIndex == 0U)
                        {
                            if(req->wValue != 0U) {
                                USBD_CtlError(pdev, req);
                                ret = USBD_FAIL;
                            }
                        }
                    } else { USBD_CtlError(pdev, req); ret = USBD_FAIL; }
                    break;

                default: USBD_CtlError(pdev, req); ret = USBD_FAIL; break;
            }
            break;

        default: USBD_CtlError(pdev, req); ret = USBD_FAIL; break;
    }
    return ret;
}

/* ----------------------------------------------------------------------- */
static uint8_t USBD_AUDIO_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
    USBD_AUDIO_HandleTypeDef *haudio = HAUDIO;
    if(haudio == NULL) return USBD_OK;

    if(epnum == (AUDIO_IN_EP & 0x7FU))
    {
        /* Counted FIRST, before any early exit, so a frozen count means
         * "DataIn is genuinely not being called" rather than "DataIn is
         * being called but bailing out early" -- two very different
         * faults that a later placement cannot tell apart. */
        datain_completions++;

        if(!haudio->in_streaming) return USBD_OK;

        /* NO EARLY RETURN HERE.
         *
         * Returning USBD_OK at this point permanently kills the IN
         * transmit chain -- note the guard two lines above does exactly
         * that and is correct there, which is why the same shape looks
         * safe here and is not. An isochronous IN
         * endpoint only ever gets another DataIn if something is in
         * flight, and the only thing that puts a transfer in flight is
         * the USBD_LL_Transmit at the bottom of this function.
         *
         * The fatal sequence:
         *   1. USBD_AUDIO_IsoINIncomplete fires (host missed an ISO
         *      frame). It sets in_iso_incomplete_pending, flushes the
         *      endpoint, and transmits.
         *   2. That transmit completes -> DataIn runs -> saw the flag,
         *      cleared it, and returned WITHOUT arming anything.
         *   3. Nothing in flight => no further DataIn, no further
         *      IsoINIncomplete. The pipe is dead for good.
         *
         * Everything observed follows from step 3: Audio_PeriodicTC is
         * only ever called from here, so rec_read freezes, rec_active
         * never arms, and AudioIF_PushSamples fills rec_ring to
         * REC_RING_SIZE-1 and then hits its "ring full" guard forever.
         * EP0 is untouched, so SET_INTERFACE keeps succeeding and
         * in_streaming still reads 1 -- the device looks healthy to the
         * host while recorded audio is silently gone.
         *
         * Fix: clear the flag but fall through to the normal path, so
         * the next transfer is always armed. A duplicate transmit is
         * survivable (the host drops it as an early/duplicate frame);
         * a dead pipe is not. */
        in_iso_incomplete_pending = 0U;

        haudio->in_wr_ptr = (haudio->in_wr_ptr == 0U) ? AUDIO_PACKET_SLOT : 0U;

        last_tx_size = ITFOPS->PeriodicTC(
            &haudio->in_buf[haudio->in_wr_ptr], AUDIO_PACKET_MAX, AUDIO_IN_TC);

        SCB_CleanDCache_by_Addr(
            (uint32_t *)&haudio->in_buf[haudio->in_wr_ptr], last_tx_size);

        USBD_LL_Transmit(pdev, AUDIO_IN_EP,
                         &haudio->in_buf[haudio->in_wr_ptr], last_tx_size);
    }
    return USBD_OK;
}

static uint8_t USBD_AUDIO_IsoINIncomplete(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
    (void)epnum;
    USBD_AUDIO_HandleTypeDef *haudio = HAUDIO;
    if(haudio == NULL || !haudio->in_streaming) return USBD_OK;

    in_iso_incomplete_pending = 1U;
    USBD_LL_FlushEP(pdev, AUDIO_IN_EP);
    USBD_LL_Transmit(pdev, AUDIO_IN_EP,
                     &haudio->in_buf[haudio->in_wr_ptr], last_tx_size);
    return USBD_OK;
}

static uint8_t USBD_AUDIO_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
    USBD_AUDIO_HandleTypeDef *haudio = HAUDIO;
    if(haudio == NULL) return USBD_FAIL;

    if(epnum == (AUDIO_OUT_EP & 0x7FU))
    {
        if(!haudio->out_streaming) return USBD_OK;

        uint16_t rx_size = (uint16_t)USBD_LL_GetRxDataSize(pdev, AUDIO_OUT_EP & 0x7FU);

        /* Do not add SCB_InvalidateDCache_by_Addr on out_buf here. It was
         * here once and it caused the worst bug in this project's history.
         *
         * The premise was that OTG_FS writes out_buf by DMA behind the
         * CPU's back, leaving the D-cache stale. It does not: OTG_FS has
         * no internal DMA, every ST reference sets dma_enable = DISABLE,
         * and HAL_PCD_IRQHandler copies FIFO words in with ordinary CPU
         * stores. The cache therefore already holds the correct bytes.
         *
         * Invalidate does not refresh from RAM, it DISCARDS the line. On a
         * write-back cache the just-stored bytes are dirty and live only
         * in cache, so invalidating threw away the packet that had just
         * arrived and the next read returned whatever was in SRAM before.
         * Misaligned, it clobbered a few bytes per cycle and produced a
         * repeating 2 ms tone; once the alignment was fixed it discarded
         * the whole packet every cycle and produced silence.
         *
         * The alignment work (AUDIO_PACKET_SLOT, the 32-byte-aligned
         * statics) stays regardless: correct hygiene, and it would matter
         * immediately if DMA were ever enabled here. */

        if(rx_size > 0U && ITFOPS->PlaybackData != NULL)
            ITFOPS->PlaybackData(&haudio->out_buf[haudio->out_wr_ptr], rx_size);

        haudio->out_wr_ptr = (haudio->out_wr_ptr == 0U) ? AUDIO_PACKET_SLOT : 0U;
        USBD_LL_PrepareReceive(pdev, AUDIO_OUT_EP,
                               &haudio->out_buf[haudio->out_wr_ptr], AUDIO_PACKET_MAX);
    }
    else if(epnum == (MIDI_OUT_EP & 0x7FU))
    {
        /* MIDI Bulk OUT -- this branch and the AUDIO_OUT_EP branch above
         * are two separate calls to this same function with different
         * epnum values, dispatched sequentially by the HAL from whichever
         * single NVIC vector (OTG_FS_EP1_OUT_IRQn) fired; they are never
         * concurrently interleaved, so there's no priority contention
         * between MIDI and Audio here to configure. Keep this branch
         * short regardless (parse-and-store into dsp_params[], no DSP
         * math here) so it can't itself become a source of delay for
         * whichever endpoint's event the HAL services next. */
        uint16_t rx_size = (uint16_t)USBD_LL_GetRxDataSize(pdev, MIDI_OUT_EP & 0x7FU);

        if(rx_size > 0U)
            AudioIF_PushMIDI(haudio->midi_buf, rx_size);

        USBD_LL_PrepareReceive(pdev, MIDI_OUT_EP, haudio->midi_buf, MIDI_EP_SIZE);
    }
    return USBD_OK;
}

static uint8_t USBD_AUDIO_IsoOutIncomplete(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
    (void)epnum;
    USBD_AUDIO_HandleTypeDef *haudio = HAUDIO;
    if(haudio == NULL || !haudio->out_streaming) return USBD_OK;

    USBD_LL_PrepareReceive(pdev, AUDIO_OUT_EP,
                           &haudio->out_buf[haudio->out_wr_ptr], AUDIO_PACKET_MAX);
    return USBD_OK;
}

static uint8_t USBD_AUDIO_SOF(USBD_HandleTypeDef *pdev)
    { (void)pdev; return USBD_OK; }

static uint8_t USBD_AUDIO_EP0_TxReady(USBD_HandleTypeDef *pdev)
    { (void)pdev; return USBD_OK; }

static uint8_t *USBD_AUDIO_GetCfgDesc(uint16_t *length)
    { *length = sizeof(USBD_AUDIO_CfgDesc); return USBD_AUDIO_CfgDesc; }

/* Unreachable on this device and kept anyway: the Device Qualifier is only
   ever requested from a high-speed-capable device, and usbd_ctlreq.c only
   calls this when dev_speed == USBD_SPEED_HIGH. The field is mandatory in
   USBD_ClassTypeDef, though, and a NULL there is worse than a stub. */
static uint8_t *USBD_AUDIO_GetDeviceQualifierDesc(uint16_t *length)
    { *length = sizeof(USBD_AUDIO_DeviceQualifierDesc); return USBD_AUDIO_DeviceQualifierDesc; }

/* -----------------------------------------------------------------------
 * Feature Unit volume range — SPK Feature Unit ID=5, master channel only.
 *
 * 0 dB to -96 dB in 1 dB steps, encoded per UAC1 spec as signed 1/256 dB
 * fixed-point (bDescriptorSubtype FEATURE_UNIT GET_CUR/MIN/MAX/RES all
 * use this same 2-byte format for VOLUME_CONTROL):
 *   0 dB   = 0x0000
 *   -1 dB  = 0xFF00  (-1 * 256)
 *   -96 dB = 0xA000  (-96 * 256)
 * Values below -96 dB (0xA000..0x8001) are reserved by spec for
 * manufacturer use; 0x8000 is defined as "silence" (effectively mute).
 * We don't use the 0x8000 silence code separately -- MUTE_CONTROL below
 * already provides an explicit, honest mute independent of the volume
 * value, which is clearer for the host UI than overloading -infinity
 * into the volume range itself. */
#define AUDIO_VOL_MAX      ((int16_t)0x0000)      /*   0 dB */
#define AUDIO_VOL_MIN       ((int16_t)0xA000)      /* -96 dB */
#define AUDIO_VOL_RES       ((int16_t)0x0100)      /*   1 dB step */
#define AUDIO_VOL_DEFAULT   AUDIO_VOL_MAX          /* start at 0 dB (unity),
                                                     * not silence, so a
                                                     * freshly-enumerated
                                                     * device is audible
                                                     * without the user
                                                     * first having to
                                                     * raise the slider */

/* Feature Unit IDs, must match the descriptor above. Used to route a
 * GET/SET request to the right AudioIF_*Volume/Mute* pair -- MIC (ID=2)
 * and SPK (ID=5) each need their own independent gain/mute state, not
 * one shared value, or moving the input slider would also move the
 * output slider and vice versa. */
#define FEATURE_UNIT_MIC   0x02U
#define FEATURE_UNIT_SPK   0x05U

static uint8_t USBD_AUDIO_EP0_RxReady(USBD_HandleTypeDef *pdev)
{
    USBD_AUDIO_HandleTypeDef *haudio = HAUDIO;
    if(haudio == NULL) return USBD_FAIL;

    if(haudio->control.cmd == AUDIO_REQ_SET_CUR)
    {
        uint8_t unit    = haudio->control.unit;
        uint8_t channel = haudio->control.channel; /* 0=master, 1=left, 2=right (wValue low byte) */

        if(haudio->control.selector == AUDIO_MUTE_CONTROL && haudio->control.len >= 1U)
        {
            if(unit == FEATURE_UNIT_MIC) {
                /* MIC Feature Unit descriptor still advertises master-only
                 * controls -- route everything there regardless of channel. */
                AudioIF_SetMuteStateMic(haudio->control.data[0]);
            } else if(unit == FEATURE_UNIT_SPK) {
                if(channel == 1U)
                    AudioIF_SetMuteStateL(haudio->control.data[0]);
                else if(channel == 2U)
                    AudioIF_SetMuteStateR(haudio->control.data[0]);
                else
                    AudioIF_SetMuteState(haudio->control.data[0]); /* channel 0 = master */
            }
        }
        else if(haudio->control.selector == AUDIO_VOLUME_CONTROL && haudio->control.len >= 2U)
        {
            int16_t vol_raw = (int16_t)((uint16_t)haudio->control.data[0] |
                                        ((uint16_t)haudio->control.data[1] << 8));
            /* Clamp to the range we actually advertised via GET_MIN/MAX --
             * a well-behaved host won't send outside it, but don't trust
             * that blindly since this crosses a USB boundary. */
            if(vol_raw > AUDIO_VOL_MAX) vol_raw = AUDIO_VOL_MAX;
            if(vol_raw < AUDIO_VOL_MIN) vol_raw = AUDIO_VOL_MIN;
            /* NOTE: intentionally not calling ITFOPS->VolumeCtl(uint8_t)
             * here -- that fops callback's signature only carries a
             * single byte, which cannot represent this 16-bit signed
             * dB value without losing the sign and most of the range.
             * AudioIF_SetVolumeRaw()/AudioIF_SetVolumeRawMic() below are
             * the real path; VolumeCtl stays as an unused stub in
             * usbd_audio_if.c (see comment there) rather than silently
             * mis-forwarding data. */
            if(unit == FEATURE_UNIT_MIC) {
                AudioIF_SetVolumeRawMic(vol_raw);
            } else if(unit == FEATURE_UNIT_SPK) {
                if(channel == 1U)
                    AudioIF_SetVolumeRawL(vol_raw);
                else if(channel == 2U)
                    AudioIF_SetVolumeRawR(vol_raw);
                else
                    AudioIF_SetVolumeRaw(vol_raw); /* channel 0 = master */
            }
        }
        haudio->control.cmd = 0U;
        haudio->control.len = 0U;
    }
    return USBD_OK;
}

/* -----------------------------------------------------------------------
 * VENDOR_REQ_GET_RING_DIAGNOSTICS (0x41) -- 8x uint32, little-endian.
 *
 * Field order matches the host script's FIELD_NAMES exactly;
 * changing the order or count here without updating that script's
 * FIELD_NAMES/struct.unpack format will silently mislabel every value.
 *
 * This is the same 8-field shape the script already expects. It used
 * to get a STALL here (no USB_REQ_TYPE_VENDOR case existed at all) --
 * this is that missing piece, not a reinterpretation of the field
 * meanings, which were already documented in usbd_audio_if.h. */
static void VENDOR_REQ_GetRingDiagnostics(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
    USBD_AUDIO_HandleTypeDef *haudio = HAUDIO;
    if(haudio == NULL) return;
    if(req->wLength == 0U) { USBD_CtlSendStatus(pdev); return; }

    uint32_t vals[8];
    vals[0] = pb_last_avail;              /* pb_fill_frames  -- current, not a historical max */
    vals[1] = rec_last_avail;             /* rec_fill_frames -- current, not a historical max */
    vals[2] = drift_corrections_up;       /* combined rec+pb, kept for backward compatibility */
    vals[3] = drift_corrections_down;     /* combined rec+pb, kept for backward compatibility */
    vals[4] = pb_overrun_events;
    vals[5] = pb_underrun_events;
    vals[6] = rec_underrun_events;
    vals[7] = pb_max_avail_seen;

    uint8_t len = (uint8_t)MIN(req->wLength, sizeof(vals));
    memcpy(haudio->control.data, vals, len);
    USBD_CtlSendData(pdev, haudio->control.data, len);
}

/* -----------------------------------------------------------------------
 * VENDOR_REQ_GET_RING_DIAGNOSTICS_EX (0x42) -- 16x uint32, little-endian.
 *
 * Superset of the 0x41 response, for isolating rec-side vs pb-side
 * drift-controller behavior instead of only seeing their combined sum.
 * Add fields at the END if this ever needs to grow again -- read_ring_
 * diagnostics.py's --split mode indexes these positionally. */
static void VENDOR_REQ_GetRingDiagnosticsEx(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
    USBD_AUDIO_HandleTypeDef *haudio = HAUDIO;
    if(haudio == NULL) return;
    if(req->wLength == 0U) { USBD_CtlSendStatus(pdev); return; }

    uint32_t vals[16];
    _Static_assert(sizeof(vals) <= USB_MAX_EP0_SIZE,
        "ring diagnostics EX payload no longer fits in the EP0 control buffer -- "
        "either trim fields or send in two requests instead of one");
    vals[0]  = rec_last_avail;
    vals[1]  = pb_last_avail;
    vals[2]  = rec_min_avail_seen;
    vals[3]  = rec_max_avail_seen;
    vals[4]  = pb_min_avail_seen;
    vals[5]  = pb_max_avail_seen;
    vals[6]  = rec_drift_corrections_up;
    vals[7]  = rec_drift_corrections_down;
    vals[8]  = pb_drift_corrections_up;
    vals[9]  = pb_drift_corrections_down;
    /* Live rate tilt in Q16, +-32 == +-488 ppm. Signed, bit-cast into
     * the slot; read_transport_diagnostics.py decodes it under CHECK 4. */
    vals[10] = (uint32_t)AudioIF_GetRecTiltQ16();
    vals[11] = (uint32_t)AudioIF_GetPbTiltQ16();
    vals[12] = rec_underrun_events;
    vals[13] = pb_overrun_events + pb_underrun_events;

    /* CRS sync status -- cumulative event counts, not raw ISR flags.
     * The raw CRS->ISR flags are read-and-clear and are already owned by
     * Audio_PeriodicTC (see usbd_audio_if.c), which polls them ~1000x/sec
     * -- far more often than this control request will be polled. Reading
     * the same read-and-clear flags here too would race with that and
     * almost always see them already cleared. Report the cumulative
     * counters instead: a healthy lock is crs_sync_ok_count climbing
     * steadily every read, with crs_sync_miss_count/crs_trim_ovf_count
     * staying flat at whatever they were (ideally 0).
     * Packing warn/miss/trim_ovf into the high bits of vals[14] instead
     * of using separate slots, to avoid growing this further:
     *   bits 0-15:  crs_sync_ok_count   (low 16 bits, wraps but that's
     *               fine -- it's a rate indicator, not an exact total)
     *   bits 16-23: crs_sync_miss_count (low 8 bits)
     *   bits 24-31: crs_trim_ovf_count  (low 8 bits)
     * crs_sync_warn_count is intentionally omitted -- SYNCWARN fires
     * routinely during normal operation (it just means the frequency
     * error exceeded half the configured tolerance on that one sync
     * event, not a lock failure) and would just add noise here. */
    vals[14] = (crs_sync_ok_count & 0xFFFFU)
             | ((crs_sync_miss_count & 0xFFU) << 16)
             | ((crs_trim_ovf_count  & 0xFFU) << 24);

    /* vals[15] -- added specifically to answer "is SYNCOK genuinely
     * never firing, or is this polling loop somehow not running at
     * all" after crs_sync_ok_count alone stayed at 0 across several
     * full-length test runs with no SYNCMISS/TRIMOVF either. See the
     * comment above crs_trim_snapshot/crs_poll_checked_count's
     * declarations in usbd_audio_if.c for the full reasoning.
     *   bits 0-5:   crs_trim_snapshot       (6-bit CRS->CR TRIM field,
     *               0-63, default/no-adjustment value is 32)
     *   bits 8-31:  crs_poll_checked_count  (low 24 bits -- this
     *               increments once per Audio_PeriodicTC call, so it
     *               climbs at ~1 kHz while the mic stream is open; if
     *               it is climbing while
     *               crs_sync_ok_count stays flat, the polling loop
     *               itself is confirmed running and the explanation
     *               narrows to TRIM/hardware, not "loop never
     *               executes") */
    vals[15] = (crs_trim_snapshot & 0x3FU)
             | ((crs_poll_checked_count & 0xFFFFFFU) << 8);

    uint8_t len = (uint8_t)MIN(req->wLength, sizeof(vals));
    memcpy(haudio->control.data, vals, len);
    USBD_CtlSendData(pdev, haudio->control.data, len);
}

/* -----------------------------------------------------------------------
 * VENDOR_REQ_GET_CHIP_REV (0x43) -- 2x uint32, little-endian: raw
 * DBGMCU_IDC register plus CRS->CFGR's live SYNCSRC field.
 *
 * One-shot diagnostic to settle which silicon revision this specific
 * board's STM32H750 is, without needing a SWD session -- see ES0392
 * errata §2.2.3 ("Clock recovery system synchronization with USB SOF
 * does not work"), which is scoped to specific revisions (affects Y, W,
 * X; not listed as affecting V). crs_sync_ok_count staying at 0 across
 * 60000+ confirmed polls (see VENDOR_REQ_GET_RING_DIAGNOSTICS_EX above)
 * is consistent with this erratum, but only this register tells us
 * whether this board's silicon is actually in the affected set.
 *
 * DBGMCU_IDC is not behind any HAL accessor in this codebase -- it's a
 * plain memory-mapped register, always readable regardless of debug
 * connection state, so no additional clock-enable or init is needed
 * before this read.
 *
 * Bits [11:0]  = DEV_ID  (0x450 expected for STM32H750)
 * Bits [31:16] = REV_ID  (0x1003 = Y/W, 0x2001 = X, 0x2003 = V)
 *
 * Also returns CRS->CFGR's live SYNCSRC[1:0] field in the response's
 * upper half. SYNCSRC is bits [29:28] of that register, confirmed
 * against ST's official STM32H7 SVD via docs.rs/stm32ral (checked
 * 2026-08). Not [15:14]: that range falls inside RELOAD[15:0], so
 * reading there returns part of the reload counter instead -- a
 * plausible-looking number that never faults, which is exactly how a
 * stale "SYNCSRC=2" can survive several rounds of changing crs.Source
 * without ever moving. See below. This settles, from the actual hardware state
 * rather than by tracing HAL macros, whether InitCRS()'s
 * RCC_CRS_SYNC_SOURCE_USB1 landed on the same physical USB controller
 * instance this firmware's USBD_LL_Init() actually uses (USB_OTG_FS in
 * usbd_conf.c) -- STM32H7 parts have two USB cores (USB1_OTG_HS and
 * USB2_OTG_FS in ST's current naming; the HAL's "USB_OTG_FS" macro that
 * usbd_conf.c uses resolves to the USB2 core's base address on this
 * family), and CRS's SYNCSRC field has a separate encoding for "USB1
 * SOF" vs "USB2 SOF" -- picking the wrong one means CRS is configured
 * and polling correctly but is listening for SOF pulses from a USB core
 * that was never initialized, which would explain crs_sync_ok_count
 * staying at 0 even on a chip revision (V) not listed as affected by
 * ES0392 §2.2.3. */
#define DBGMCU_IDC_ADDR   (0x5C001000UL)
static void VENDOR_REQ_GetChipRev(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
    USBD_AUDIO_HandleTypeDef *haudio = HAUDIO;
    if(haudio == NULL) return;
    if(req->wLength == 0U) { USBD_CtlSendStatus(pdev); return; }

    uint32_t idc = *(volatile uint32_t *)DBGMCU_IDC_ADDR;

    /* CRS_CFGR_SYNCSRC is bits [29:28]: 00=GPIO/PIN, 01=LSE, 10=USB1,
     * 11=USB2 (confirmed against ST's official SVD for STM32H7 via
     * docs.rs/stm32ral -- CRS_CFGR_SYNCSRC mask 0b11, offset 28). This
     * matches where RCC_CRS_SYNC_SOURCE_USB1/USB2 in
     * stm32h7xx_hal_rcc_ex.h actually place their bits (those macros
     * were correct all along; only this diagnostic read was wrong
     * before). Packed into vals[1] bits [1:0] here so a 2-word (8-byte)
     * response covers both the chip revision and this cross-check in
     * one request. */
    uint32_t syncsrc = (CRS->CFGR >> 28U) & 0x3U;

    uint32_t vals[2];
    vals[0] = idc;
    vals[1] = syncsrc;

    uint8_t len = (uint8_t)MIN(req->wLength, sizeof(vals));
    memcpy(haudio->control.data, vals, len);
    USBD_CtlSendData(pdev, haudio->control.data, len);
}

/* -----------------------------------------------------------------------
 * VENDOR_REQ_GET_TRANSPORT_HEALTH (0x46) -- 4x uint32, little-endian.
 *
 * Exists because three things are invisible from the host through
 * 0x41/0x42: whether the IN pipe ever stalled,
 * whether the watchdog had to save it, and whether the packet size is
 * genuinely constant. Two of these have exactly one acceptable value.
 *
 * Note the buffer: it must outlive this function, and a local array
 * will not. That is wrong for a reason no test will show:
 * USBD_CtlSendData does not copy. It stores the pointer, and the write
 * into the Tx FIFO happens later, from the FIFO-empty interrupt --
 * USB_EP0StartXfer in non-DMA mode only arms DIEPEMPMSK. By then this
 * function has returned and its frame, which lives on the interrupt
 * stack, has been reused. The buffer must outlive the transfer, so it
 * uses the same persistent control.data as the other four vendor
 * requests. The assert guards the 64-byte ceiling that buffer has;
 * anything larger would run past it into the rest of the handle. */
static void VENDOR_REQ_GetTransportHealth(USBD_HandleTypeDef   *pdev,
                                          USBD_SetupReqTypedef *req)
{
    USBD_AUDIO_HandleTypeDef *haudio = HAUDIO;
    if(haudio == NULL) { USBD_CtlError(pdev, req); return; }
    if(req->wLength == 0U) { USBD_CtlSendStatus(pdev); return; }

    uint32_t vals[4];
    _Static_assert(sizeof(vals) <= USB_MAX_EP0_SIZE,
        "transport health payload exceeds the EP0 control buffer");

    vals[0] = datain_completions;
    vals[1] = in_pipe_kicks;
    vals[2] = tx_size_deviations;
    vals[3] = pb_read_advances;

    uint8_t len = (uint8_t)MIN(req->wLength, sizeof(vals));
    memcpy(haudio->control.data, vals, len);
    USBD_CtlSendData(pdev, haudio->control.data, len);
}

/* -----------------------------------------------------------------------
 * VENDOR_REQ_GET_CLOCK_TREE (0x45) -- 16x uint32, little-endian.
 *
 * Read-only dump of the registers that determine the SAI sample rate,
 * so the host can compute what this board's audio clock ACTUALLY is
 * rather than assuming the nominal 48000 Hz.
 *
 * WHY THIS EXISTS: the playback drift corrector reports a stable
 * -240 ppm offset between the SAI clock and the USB host. That figure
 * reproduced across four independent runs (-238.5, -238.7, -243.0,
 * -239.9) and, decisively, moved only 0.38 ppm across a 35-minute
 * thermal soak. Crystals cannot do that -- two independent crystals
 * drift tens of ppm over that temperature range. A ratio of integer
 * dividers off a single HSE, on the other hand, is temperature-
 * invariant by construction, which is exactly what was observed. So
 * the offset is a fixed nominal-rate error in the clock configuration,
 * and these registers are where it lives.
 *
 * Note the magnitude also rules out fractional-divider quantisation:
 * one LSB of FRACN at a typical DIVN is on the order of 1 ppm, so a
 * rounding error would be single-digit ppm, not 240.
 *
 * Word 15 is HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SAI1), i.e. the
 * SAI kernel clock as the HAL itself computes it from the live register
 * state. It is included deliberately as a cross-check: the host script
 * recomputes the same number from words 0..8 independently, and the two
 * must agree. If they diverge, the host-side model of the clock tree is
 * wrong and any sample-rate conclusion drawn from it is void.
 *
 * All reads are plain memory-mapped register loads with no side effects
 * and no clock-enable requirements, and run only when the host issues
 * this control request -- nothing here touches the audio data path. */
static void VENDOR_REQ_GetClockTree(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
    USBD_AUDIO_HandleTypeDef *haudio = HAUDIO;

    uint32_t vals[16];

    vals[0]  = RCC->PLLCKSELR;   /* PLLSRC + DIVM1/2/3 prescalers        */
    vals[1]  = RCC->PLLCFGR;     /* per-PLL FRACEN / range / output enab */
    vals[2]  = RCC->PLL1DIVR;    /* DIVN1/P1/Q1/R1                       */
    vals[3]  = RCC->PLL1FRACR;
    vals[4]  = RCC->PLL2DIVR;
    vals[5]  = RCC->PLL2FRACR;
    vals[6]  = RCC->PLL3DIVR;    /* the one libDaisy normally feeds SAI  */
    vals[7]  = RCC->PLL3FRACR;
    vals[8]  = RCC->D2CCIP1R;    /* SAI1SEL[2:0] -- which PLL drives SAI */
    vals[9]  = RCC->CFGR;
    vals[10] = RCC->CR;          /* HSE/HSI/PLL ready + bypass state     */
    vals[11] = (uint32_t)HSE_VALUE; /* compile-time crystal constant --
                                     * if this does not match the board's
                                     * actual crystal, every derived
                                     * frequency is wrong by that ratio  */
    vals[12] = SAI1_Block_A->CR1;   /* MCKDIV + NOMCK live in here       */
    vals[13] = SAI1_Block_B->CR1;
    vals[14] = SAI1_Block_A->FRCR;  /* frame length -> FS divider        */
    vals[15] = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SAI1);

    uint8_t len = (uint8_t)MIN(req->wLength, sizeof(vals));
    memcpy(haudio->control.data, vals, len);
    USBD_CtlSendData(pdev, haudio->control.data, len);
}

static void AUDIO_REQ_GetCurrent(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
    USBD_AUDIO_HandleTypeDef *haudio = HAUDIO;
    if(haudio == NULL) return;
    if(req->wLength == 0U) {
        USBD_CtlSendStatus(pdev);
        return;
    }

    uint8_t selector = HIBYTE(req->wValue);
    uint8_t unit     = HIBYTE(req->wIndex);
    uint8_t channel  = LOBYTE(req->wValue); /* 0=master, 1=left, 2=right */
    memset(haudio->control.data, 0, USB_MAX_EP0_SIZE);

    if(selector == AUDIO_MUTE_CONTROL) {
        if(unit == FEATURE_UNIT_MIC) {
            haudio->control.data[0] = AudioIF_GetMuteStateMic();
        } else if(unit == FEATURE_UNIT_SPK && channel == 1U) {
            haudio->control.data[0] = AudioIF_GetMuteStateL();
        } else if(unit == FEATURE_UNIT_SPK && channel == 2U) {
            haudio->control.data[0] = AudioIF_GetMuteStateR();
        } else {
            haudio->control.data[0] = AudioIF_GetMuteState();
        }
        USBD_CtlSendData(pdev, haudio->control.data, MIN(req->wLength, 1U));
    } else if(selector == AUDIO_VOLUME_CONTROL) {
        int16_t vol;
        if(unit == FEATURE_UNIT_MIC) {
            vol = AudioIF_GetVolumeRawMic();
        } else if(unit == FEATURE_UNIT_SPK && channel == 1U) {
            vol = AudioIF_GetVolumeRawL();
        } else if(unit == FEATURE_UNIT_SPK && channel == 2U) {
            vol = AudioIF_GetVolumeRawR();
        } else {
            vol = AudioIF_GetVolumeRaw();
        }
        haudio->control.data[0] = LOBYTE((uint16_t)vol);
        haudio->control.data[1] = HIBYTE((uint16_t)vol);
        USBD_CtlSendData(pdev, haudio->control.data, MIN(req->wLength, 2U));
    } else {
        /* Unknown selector -- respond with zeroed data of the requested
         * length rather than erroring, matching the previous behavior
         * for anything we don't explicitly recognize. */
        USBD_CtlSendData(pdev, haudio->control.data, MIN(req->wLength, USB_MAX_EP0_SIZE));
    }
}

/* GET_MIN / GET_MAX / GET_RES all share this handler -- req->bRequest
 * tells us which of the three was asked; only VOLUME_CONTROL has a
 * meaningful range (MUTE_CONTROL is a boolean and per spec never
 * receives GET_MIN/MAX/RES from a compliant host, but we respond
 * sanely rather than stalling if one ever does). Both MIC and SPK
 * Feature Units share the same advertised range (0dB..-96dB, 1dB step)
 * -- only the *current* value differs per-unit, not the range itself. */
static void AUDIO_REQ_GetRange(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
    USBD_AUDIO_HandleTypeDef *haudio = HAUDIO;
    if(haudio == NULL) return;
    if(req->wLength == 0U) {
        USBD_CtlSendStatus(pdev);
        return;
    }

    uint8_t selector = HIBYTE(req->wValue);
    int16_t val = 0;

    if(selector == AUDIO_VOLUME_CONTROL) {
        switch(req->bRequest) {
            case AUDIO_REQ_GET_MIN: val = AUDIO_VOL_MIN; break;
            case AUDIO_REQ_GET_MAX: val = AUDIO_VOL_MAX; break;
            case AUDIO_REQ_GET_RES: val = AUDIO_VOL_RES; break;
            default: val = 0; break;
        }
    }

    memset(haudio->control.data, 0, USB_MAX_EP0_SIZE);
    haudio->control.data[0] = LOBYTE((uint16_t)val);
    haudio->control.data[1] = HIBYTE((uint16_t)val);
    USBD_CtlSendData(pdev, haudio->control.data, MIN(req->wLength, 2U));
}

static void AUDIO_REQ_SetCurrent(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
    USBD_AUDIO_HandleTypeDef *haudio = HAUDIO;
    if(haudio == NULL) return;
    if(req->wLength != 0U) {
        haudio->control.cmd      = AUDIO_REQ_SET_CUR;
        haudio->control.len      = (uint8_t)MIN(req->wLength, USB_MAX_EP0_SIZE);
        haudio->control.unit     = HIBYTE(req->wIndex);
        haudio->control.selector = HIBYTE(req->wValue);
        haudio->control.channel  = LOBYTE(req->wValue);
        USBD_CtlPrepareRx(pdev, haudio->control.data, haudio->control.len);
    } else {
        USBD_CtlSendStatus(pdev);
    }
}

uint8_t USBD_AUDIO_RegisterInterface(USBD_HandleTypeDef *pdev,
                                     USBD_AUDIO_ItfTypeDef *fops)
{
    if(fops == NULL) return USBD_FAIL;
    *USBD_COMPAT_USERDATA_SLOT(pdev) = fops;
    return USBD_OK;
}

/* -----------------------------------------------------------------------
 * USBD_AUDIO_KickInPipe — watchdog recovery for a stalled IN endpoint.
 *
 * Called from the main loop (thread mode), NOT from an ISR. It exists
 * because a dead isochronous IN pipe is unrecoverable from inside the
 * USB stack: the only thing that arms a transfer is USBD_LL_Transmit,
 * and the only places that call it are USBD_AUDIO_DataIn (which needs a
 * completed transfer to run) and SET_INTERFACE(alt=1) (which needs the
 * host to act). If a transfer is ever lost without a completion --
 * whatever the cause -- both doors are shut and only a physical replug
 * recovers.
 *
 * The caller decides when the pipe looks stalled (datain_completions
 * frozen while in_streaming is set); this function just re-arms. Safe to
 * call spuriously: an extra ISO IN packet is dropped by the host as an
 * early/duplicate frame.
 *
 * Racing the USB ISR is not a practical concern: the caller only fires
 * this after the completion counter has been frozen for hundreds of
 * consecutive 1 ms frames, i.e. once the endpoint is provably idle. */
void USBD_AUDIO_KickInPipe(USBD_HandleTypeDef *pdev)
{
    if(pdev == NULL || pdev->dev_state != USBD_STATE_CONFIGURED) return;

    USBD_AUDIO_HandleTypeDef *haudio =
        (USBD_AUDIO_HandleTypeDef *)pdev->pClassData;
    if(haudio == NULL || !haudio->in_streaming) return;

    in_iso_incomplete_pending = 0U;
    USBD_LL_FlushEP(pdev, AUDIO_IN_EP);
    USBD_LL_Transmit(pdev, AUDIO_IN_EP,
                     &haudio->in_buf[haudio->in_wr_ptr], last_tx_size);
    in_pipe_kicks++;
}
