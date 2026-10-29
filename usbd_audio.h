/**
 * usbd_audio.h — declarations and constants for the UAC1 + MIDIStreaming
 * class implementation in usbd_audio.c.
 *
 * ST's portions: Copyright (c) 2015 STMicroelectronics. All rights reserved.
 * MIT License (see LICENSE at repository root) applies to this project's
 * contributions in this file, not to the file as a whole; the SCOPE section
 * of that file names which portions are ST's.
 *
 * This header retains more of ST's original than usbd_audio.c does, and the
 * difference is worth stating rather than glossing. Compared line-by-line
 * against Class/AUDIO/Inc/usbd_audio.h at tag v2.5.2, roughly 40% of that
 * file's substantive lines survive verbatim here: the AUDIO_* class and
 * descriptor constants, the endpoint and descriptor-size macros, and the
 * shape of the control-request and handle structures. Those are ST's and
 * SLA0044 governs them. What was written for this project is the duplex,
 * 24-bit, MIDI-carrying descriptor layout built from them and the constants
 * that describe it. usbd_audio.c is the reverse case — about 7% of ST's
 * file survives there, essentially the USBD_ClassTypeDef callback
 * declarations. See that file's header, THIRD_PARTY_NOTICES.md §1, and
 * README.md's "Licensing" section.
 */
#ifndef __USB_AUDIO_H
#define __USB_AUDIO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "usbd_ioreq.h"

/* -----------------------------------------------------------------------
 * Frequency & interface counts */
#define USBD_AUDIO_FREQ             48000U

/* 4 interfaces: AudioControl(0) + AS MIC(1) + AS SPK(2) + MIDIStreaming(3).
   Must match USBD_MAX_NUM_INTERFACES in usbd_conf.h. */
#ifdef USBD_MAX_NUM_INTERFACES
#undef USBD_MAX_NUM_INTERFACES
#endif
#define USBD_MAX_NUM_INTERFACES     4U

/* -----------------------------------------------------------------------
 * Endpoint addresses
 *
 *   OUT (host → device):
 *     AUDIO_OUT_EP  0x01  ISO OUT  — speaker / playback
 *     MIDI_OUT_EP   0x02  Bulk OUT — MIDI CC from the host
 *
 *   IN (device → host):
 *     AUDIO_IN_EP   0x81  ISO IN  — microphone / record
 *     MIDI_IN_EP    0x82  Bulk IN — declared and opened per spec, but
 *                          this firmware never transmits on it */
#define AUDIO_OUT_EP                0x01U
#define AUDIO_IN_EP                 0x81U
#define MIDI_OUT_EP                 0x02U
#define MIDI_IN_EP                  0x82U

#define AUDIO_FS_BINTERVAL          0x01U   /* legend: bInterval written literally below */

/* -----------------------------------------------------------------------
 * Descriptor size
 *
 * Bidirectional UAC1 + MIDIStreaming (byte count):
 *   9   Configuration
 *   9   AC Interface
 *  11   AC Header        (bInCollection=3, 8 + 3 = 11)
 *  12   MIC Input Terminal
 *  10   MIC Feature Unit (bLength = 7 + (2ch+1)*1 = 10)
 *   9   MIC Output Terminal
 *  12   SPK Input Terminal
 *  10   SPK Feature Unit
 *   9   SPK Output Terminal        ← 91 bytes above
 *   9   MIC AS alt-0
 *   9   MIC AS alt-1
 *   7   MIC AS General
 *  11   MIC Format Type I
 *   9   MIC ISO IN  EP
 *   7   MIC AS EP descriptor       ← 52 bytes
 *   9   SPK AS alt-0
 *   9   SPK AS alt-1
 *   7   SPK AS General
 *  11   SPK Format Type I
 *   9   SPK ISO OUT EP
 *   7   SPK AS EP descriptor       ← 52 bytes
 *   9   MIDI Interface
 *   7   MS Header
 *   6   MIDI IN Jack 1 (Embedded)
 *   6   MIDI IN Jack 2 (External)
 *   9   MIDI OUT Jack 3 (Embedded)
 *   9   MIDI OUT Jack 4 (External)
 *   9   Bulk OUT EP 0x02
 *   5   CS Bulk OUT EP
 *   9   Bulk IN  EP 0x82
 *   5   CS Bulk IN  EP             ← 74 bytes
 * ---
 * 269  total */
/* The AC Header carries three interfaces in baInterfaceNr, not two: adding
 * MIDIStreaming (interface 3) is what fixed Windows Code 10 / 0xC0000182.
 * See the comment at the AC Header in usbd_audio.c. */
#define USB_AUDIO_CONFIG_DESC_SIZ   269U

/* The configuration descriptor below is written out as literal bytes, so
   these size constants are not used to build it. They are kept as a legend
   for reading those bytes: each one names the length the spec assigns to
   that descriptor type. */
#define AUDIO_INTERFACE_DESC_SIZE               0x09U
#define USB_AUDIO_DESC_SIZ                      0x0AU  /* legend: AC Header = 10 bytes */
#define AUDIO_STANDARD_ENDPOINT_DESC_SIZE       0x09U
#define AUDIO_STREAMING_ENDPOINT_DESC_SIZE      0x07U

/* -----------------------------------------------------------------------
 * Class / subclass / protocol codes */
#define USB_DEVICE_CLASS_AUDIO              0x01U
#define AUDIO_SUBCLASS_AUDIOCONTROL         0x01U
#define AUDIO_SUBCLASS_AUDIOSTREAMING       0x02U
#define AUDIO_SUBCLASS_MIDISTREAMING        0x03U
#define AUDIO_PROTOCOL_UNDEFINED            0x00U
#define AUDIO_STREAMING_GENERAL             0x01U
#define AUDIO_STREAMING_FORMAT_TYPE         0x02U

#define AUDIO_INTERFACE_DESCRIPTOR_TYPE     0x24U
#define AUDIO_ENDPOINT_DESCRIPTOR_TYPE      0x25U

/* AC descriptor subtypes */
#define AUDIO_CONTROL_HEADER                0x01U
#define AUDIO_CONTROL_INPUT_TERMINAL        0x02U
#define AUDIO_CONTROL_OUTPUT_TERMINAL       0x03U
#define AUDIO_CONTROL_FEATURE_UNIT          0x06U

#define AUDIO_INPUT_TERMINAL_DESC_SIZE      0x0CU
#define AUDIO_OUTPUT_TERMINAL_DESC_SIZE     0x09U
#define AUDIO_STREAMING_INTERFACE_DESC_SIZE 0x07U

#define AUDIO_CONTROL_MUTE                  0x0001U

/* Feature Unit Control Selectors (UAC1 spec, Table A-11) — these are
 * what actually arrives in HIBYTE(req->wValue) on GET/SET_CUR|MIN|MAX|RES,
 * distinct from AUDIO_CONTROL_MUTE above (which is a bmaControls bitmask
 * bit position, not a control selector value -- the two use overlapping
 * numeric space by coincidence and were being conflated before this fix). */
#define AUDIO_MUTE_CONTROL                  0x01U
#define AUDIO_VOLUME_CONTROL                0x02U

#define AUDIO_FORMAT_TYPE_I                 0x01U

#define AUDIO_ENDPOINT_GENERAL              0x01U

/* -----------------------------------------------------------------------
 * Class requests / PeriodicTC tags */
#define AUDIO_REQ_GET_CUR                   0x81U
#define AUDIO_REQ_SET_CUR                   0x01U
#define AUDIO_REQ_GET_MIN                    0x82U
#define AUDIO_REQ_GET_MAX                    0x83U
#define AUDIO_REQ_GET_RES                    0x84U

/* -----------------------------------------------------------------------
 * Vendor requests (bmRequest type = USB_REQ_TYPE_VENDOR) -- diagnostics
 * only, no effect on audio behavior. Consumed by the host-side script
 * the host diagnostics; wValue/wIndex are unused (0), wLength must
 * match the sizes below exactly or USBD_CtlSendData will be asked to
 * send data of a different amount than the host requested.
 *
 * These are about drift-controller convergence, not roundtrip latency --
 * there is no latency measurement in this firmware, so no GET_LATENCY
 * request exists here (a prior host-side script assumed one; that
 * assumption doesn't match this codebase and isn't being carried over). */
#define VENDOR_REQ_GET_RING_DIAGNOSTICS       0x41U  /* -> 8x uint32 (32 bytes), see usbd_audio.c */
#define VENDOR_REQ_GET_RING_DIAGNOSTICS_EX    0x42U  /* -> 16x uint32 (64 bytes = USB_MAX_EP0_SIZE,
                                                        * at the ceiling -- can't grow further without
                                                        * a second request), split rec/pb fields +
                                                        * CRS sync counters + CRS TRIM/poll-heartbeat */
#define VENDOR_REQ_GET_TRANSPORT_HEALTH       0x46U  /* -> 4x uint32 (16 bytes), little-endian.
                                                        *   [0] datain_completions  -- ~1000/s while the
                                                        *       host reads the record stream; a frozen
                                                        *       value with in_streaming set means the IN
                                                        *       pipe died.
                                                        *   [1] in_pipe_kicks       -- how often the
                                                        *       main-loop watchdog had to re-arm it.
                                                        *       MUST be 0 in a healthy run.
                                                        *   [2] tx_size_deviations  -- IN packets that
                                                        *       were not exactly SAMPLES_NOM frames.
                                                        *       MUST be 0: non-zero means the Adaptive
                                                        *       descriptor is lying to the host.
                                                        *   [3] pb_read_advances    -- integer playback
                                                        *       read advances (~48000/s when streaming).
                                                        *   Firmware without this request STALLs it,
                                                        *       which the host can use to tell the two
                                                        *       apart. */
#define VENDOR_REQ_GET_CLOCK_TREE             0x45U  /* -> 16x uint32 (64 bytes), little-endian.
                                                        * Raw PLL/SAI clock-tree registers plus the
                                                        * HAL-computed SAI kernel clock, so the host
                                                        * can derive the ACTUAL sample rate instead
                                                        * of trusting the nominal 48000. Added to
                                                        * chase a reproducible ~240 ppm offset that
                                                        * survived a 35-minute thermal soak with
                                                        * only 0.38 ppm of drift -- i.e. a fixed
                                                        * divider-ratio error, not crystal drift.
                                                        * See VENDOR_REQ_GetClockTree below. */
#define VENDOR_REQ_GET_CHIP_REV               0x43U  /* -> 2x uint32 (8 bytes), little-endian:
                                                        * word0 = raw DBGMCU_IDC register
                                                        *   (0x5C001000). Bits [11:0] = DEV_ID
                                                        *   (0x450 expected for STM32H750), bits
                                                        *   [31:16] = REV_ID (0x1003=Y/W, 0x2001=X,
                                                        *   0x2003=V -- see ES0392 errata sheet
                                                        *   §2.2.3: CRS synchronization by USB SOF
                                                        *   does not work on Y/W/X silicon).
                                                        * word1 = live CRS->CFGR SYNCSRC[1:0] field
                                                        *   (0=GPIO/PIN, 1=LSE, 2=USB1, 3=USB2) --
                                                        *   cross-checks which physical USB core CRS
                                                        *   is actually configured to listen to,
                                                        *   against which core usbd_conf.c's
                                                        *   USB_OTG_FS instance actually is on this
                                                        *   family (see usbd_audio.c's
                                                        *   VENDOR_REQ_GetChipRev for the full
                                                        *   reasoning). One-shot diagnostic query,
                                                        *   not part of the normal audio/MIDI data
                                                        *   path. */
/* Transfer-complete tag handed to Audio_PeriodicTC. Only the IN side has
   one: the OUT side has no periodic callback to tag. */
#define AUDIO_IN_TC                         0x02U

/* -----------------------------------------------------------------------
 * MIDI */
#define MIDI_EP_SIZE                64U    /* USB FS bulk max packet        */
#define MIDI_EVENT_SIZE             4U     /* one USB MIDI event = 4 bytes  */
#define MIDI_CIN_CONTROL_CHANGE     0x0BU  /* Code Index Number for CC      */

/* MIDIStreaming class-specific descriptor subtypes and jack types
 * (USB Device Class Definition for MIDI Devices, rev 1.0, table A-1/A-2) */
#define MS_HEADER                   0x01U
#define MIDI_IN_JACK                0x02U
#define MIDI_OUT_JACK               0x03U
#define MS_GENERAL                  0x01U  /* CS endpoint subtype -- numerically
                                             * same as MS_HEADER (0x01) but a
                                             * different descriptor type context
                                             * (CS_ENDPOINT vs CS_INTERFACE),
                                             * per spec */

#define MIDI_JACK_EMBEDDED          0x01U
#define MIDI_JACK_EXTERNAL          0x02U

#define CS_ENDPOINT_DESCRIPTOR_TYPE 0x25U  /* same numeric value as
                                             * AUDIO_ENDPOINT_DESCRIPTOR_TYPE
                                             * above -- CS_ENDPOINT is 0x25 in
                                             * both UAC1 and MIDI 1.0 specs;
                                             * aliased here under its MIDI-spec
                                             * name for readability at MIDI
                                             * descriptor call sites. */

/* -----------------------------------------------------------------------
 * Packet size constants
 *
 * AUDIO_BYTES_PER_SAMPLE = 3: 24-bit tightly packed, bSubframeSize=3 in
 * both Format Type descriptors, not the 4-byte "24-in-32" packing some
 * devices use. usbd_audio_if.c has its own BYTES_PER_SAMPLE for the same
 * quantity; the two must move together.
 *
 * AUDIO_OUT_PACKET = 48 frames × 2ch × 3B = 288 bytes  (nominal)
 * AUDIO_PACKET_MAX = 49 frames × 2ch × 3B = 294 bytes  (declared maximum)
 *
 * Both endpoints declare AUDIO_PACKET_MAX as wMaxPacketSize, and the FIFO
 * budget in usbd_conf.c is sized against it. This device always transmits
 * exactly AUDIO_OUT_PACKET on the IN endpoint -- the fractional corrector
 * absorbs drift inside the packet rather than by resizing it -- but the
 * host is free to vary its OUT packet within the declared maximum, so the
 * headroom is real on that side and must not be trimmed to 288. */
#define AUDIO_BYTES_PER_SAMPLE  3U

#define AUDIO_OUT_PACKET  (uint16_t)(((USBD_AUDIO_FREQ * 2U * AUDIO_BYTES_PER_SAMPLE) / 1000U))         /* 288 */
#define AUDIO_PACKET_MAX  (uint16_t)((((USBD_AUDIO_FREQ / 1000U) + 1U) * 2U * AUDIO_BYTES_PER_SAMPLE))  /* 294 */

#define AUDIO_DEFAULT_VOLUME    70U

/* -----------------------------------------------------------------------
 * Per-endpoint double-buffers (2 × 320 = 640 bytes each)
 *
 * Separate IN and OUT buffers — they toggle independently and may carry
 * different packet sizes simultaneously.
 *
 * AUDIO_PACKET_SLOT rounds AUDIO_PACKET_MAX up to the next multiple of
 * 32, the Cortex-M7 D-cache line size, giving 320. Without the padding
 * slot 1 would start off a cache-line boundary and maintenance on one
 * slot could bleed into the other -- a failure mode this project hit once
 * already. At 320, slot 0 is bytes 0..319 and slot 1 is 320..639, both on
 * 32-byte boundaries. */
#define AUDIO_PACKET_SLOT   ((uint16_t)(((AUDIO_PACKET_MAX + 31U) / 32U) * 32U))  /* 320 */
#define AUDIO_EP_BUF_SIZE   ((uint16_t)(AUDIO_PACKET_SLOT * 2U))   /* 640 bytes */

/* -----------------------------------------------------------------------
 * Handle
 *
 * Dead fields removed (offset / rd_enable / rd_ptr were never used after
 * the move to frame-based ring in usbd_audio_if.c).
 *
 * midi_buf — single-packet Bulk OUT receive buffer, 64 bytes.
 *            Immediately parsed in DataOut and discarded; no double-
 *            buffering needed since MIDI events are low-rate.
 *
 * in_buf/out_buf are NOT stored inline in this struct. The struct
 * itself is allocated with USBD_malloc (usbd_audio.c), which gives no
 * alignment guarantee beyond the C library's default (typically 8
 * bytes on this toolchain) -- nowhere near the 32-byte boundary D-cache
 * maintenance operations require. Putting a 32-byte-aligned array
 * inside a non-32-byte-aligned struct doesn't help: the array's actual
 * runtime address still isn't guaranteed aligned, only its offset
 * within the struct is. Instead, in_buf/out_buf are separate static
 * arrays (defined in usbd_audio.c) with explicit __ALIGN_BEGIN/
 * __ALIGN_END (32-byte aligned, matching the rest of this codebase's
 * convention for USBD_AUDIO_CfgDesc etc.), and this struct just holds
 * pointers to them. */
typedef struct {
    uint8_t  cmd;
    uint8_t  data[USB_MAX_EP0_SIZE];
    uint8_t  len;
    uint8_t  unit;
    uint8_t  selector;  /* wValue high byte: MUTE_CONTROL=0x01, VOLUME_CONTROL=0x02 */
    uint8_t  channel;   /* wValue low byte: 0=master, 1=left, 2=right */
} USBD_AUDIO_ControlTypeDef;

typedef struct {
    /* ── IN path: mic → host ───────────────────────────────────────── */
    uint32_t  alt_setting_in;
    uint8_t  *in_buf;                     /* -> static, 32-byte-aligned, AUDIO_EP_BUF_SIZE bytes */
    uint16_t  in_wr_ptr;                  /* 0 or AUDIO_PACKET_SLOT — next write slot */
    uint8_t   in_streaming;               /* 1 when alt-setting-in == 1 */

    /* ── OUT path: host → speaker ──────────────────────────────────── */
    uint32_t  alt_setting_out;
    uint8_t  *out_buf;                    /* -> static, 32-byte-aligned, AUDIO_EP_BUF_SIZE bytes */
    uint16_t  out_wr_ptr;                 /* slot currently armed for DMA reception */
    uint8_t   out_streaming;              /* 1 when alt-setting-out == 1 */

    /* ── MIDI path: host → device ──────────────────────────────────── */
    uint8_t   midi_buf[MIDI_EP_SIZE];     /* Bulk OUT receive buffer, 64 bytes */

    /* ── EP0 control ───────────────────────────────────────────────── */
    USBD_AUDIO_ControlTypeDef control;
} USBD_AUDIO_HandleTypeDef;

/* -----------------------------------------------------------------------
 * Interface operations
 *
 * PeriodicTC — IN direction (mic → host):
 *   Called every 1ms from DataIn ISR.  Returns the number of bytes written
 *   to pbuf (282 / 288 / 294, was 188/192/196 at 16-bit). DataIn passes
 *   this directly to USBD_LL_Transmit so variable packet size reaches
 *   the wire.
 *
 * PlaybackData — OUT direction (host → speaker):
 *   Called from DataOut ISR with a pointer to the just-received packet and
 *   its byte count.  Implementation pushes data into the playback ring.
 *   Must return quickly (ISR context). */
typedef struct {
    int8_t   (*Init)        (uint32_t AudioFreq, uint32_t Volume, uint32_t options);
    int8_t   (*DeInit)      (uint32_t options);
    int8_t   (*AudioCmd)    (uint8_t *pbuf, uint32_t size, uint8_t cmd);
    int8_t   (*VolumeCtl)   (uint8_t vol);
    int8_t   (*MuteCtl)     (uint8_t cmd);
    uint16_t (*PeriodicTC)  (uint8_t *pbuf, uint32_t size, uint8_t cmd);
    int8_t   (*GetState)    (void);
    void     (*PlaybackData)(uint8_t *pbuf, uint16_t size);
} USBD_AUDIO_ItfTypeDef;

extern USBD_ClassTypeDef USBD_AUDIO;

/* IN-pipe stall watchdog -- call from the main loop, never from an ISR.
 * See the definition in usbd_audio.c for why the USB stack cannot
 * recover a dead isochronous IN pipe on its own. */
void USBD_AUDIO_KickInPipe(USBD_HandleTypeDef *pdev);
extern volatile uint32_t datain_completions;
extern volatile uint32_t in_pipe_kicks;

/* Size (in bytes) of the most recent successful IN transmit. Not static
 * in usbd_audio.c because HAL_PCD_ResumeCallback (usbd_conf.c) reads it
 * to re-arm the pipe after a suspend/resume cycle. Written and read from
 * USB ISR context on both sides, hence volatile. */
extern volatile uint16_t last_tx_size;

uint8_t USBD_AUDIO_RegisterInterface(USBD_HandleTypeDef *pdev,
                                     USBD_AUDIO_ItfTypeDef *fops);

#ifdef __cplusplus
}
#endif

#endif /* __USB_AUDIO_H */
