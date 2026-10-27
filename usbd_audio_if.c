/**
 * @file usbd_audio_if.c
 * @brief UAC1 bidirectional + MIDI — fractional (Q16) clock correction
 *
 * @attention
 * MIT License (see LICENSE at repository root) applies to this file. This
 * is application-level code — ring buffers, sample packing, the fractional
 * clock-drift controller, MIDI CC parsing — written for this project, not
 * derived from any ST Audio class example. THIRD_PARTY_NOTICES.md carries
 * the repository's file-by-file breakdown.
 *
 * RECORD PATH (ADC -> host)
 *   AudioIF_PushSamples()  [audio DMA ISR, ~1 ms]  writes interleaved
 *     int32 frames (sign-extended 24-bit) into rec_ring, advancing
 *     rec_write.
 *   Audio_PeriodicTC()  [USB DataIn ISR, 1 ms]  runs the rate controller,
 *     copies tx_samples frames into pbuf packing each sample into 3 wire
 *     bytes, advances rec_read, and returns the byte count to DataIn.
 *
 * PLAYBACK PATH (host -> DAC)
 *   Audio_PlaybackData()  [USB DataOut ISR, 1 ms]  unpacks the host's
 *     3-byte-packed samples into pb_ring, dropping frames and counting
 *     pb_overrun_events when full.
 *   AudioIF_PopSamples()  [audio DMA ISR]  pulls frames out, zero-filling
 *     and counting pb_underrun_events when empty.
 *
 * MIDI PATH (Bulk OUT -> CC table)
 *   AudioIF_PushMIDI() parses 4-byte USB MIDI event packets and stores CC
 *   values in dsp_params; AudioIF_GetCCParam() reads them back. Single
 *   volatile byte either way, atomic on Cortex-M7, no mutex.
 *
 * FRACTIONAL (Q16) DRIFT CONTROLLER, both paths
 *
 * Every ms on the record side, every audio block on playback:
 *
 *   error    = available_frames - TARGET
 *   tilt     = clamp(error / RATE_DIV, +-RATE_MAX_Q16)
 *   step_q16 = FRAC_ONE + tilt
 *
 * The ring is then read at a fractional position advancing by step_q16 per
 * output frame, and the sample there is reconstructed by the fractional-
 * delay filter below. The correction is spread across every sample rather
 * than concentrated into one, so no discontinuity is ever produced and the
 * IN packet stays exactly SAMPLES_NOM frames — which is what the Adaptive
 * endpoint descriptor claims.
 *
 * Control is plain proportional, no integrator, so each ring settles at a
 * small standing offset proportional to the clock mismatch instead of
 * hunting. Authority is +-RATE_MAX_Q16 = +-488 ppm, far more than any real
 * crystal pairing needs and under a hundredth of a semitone.
 *
 * TUNING
 *   REC_RING_SIZE / PB_RING_SIZE  512, power of two, 10.67 ms @ 48 kHz
 *   REC_TARGET / PB_TARGET        256 = half the ring, 5.33 ms
 *   REC_RATE_DIV / PB_RATE_DIV    8 — smaller is stiffer
 *   REC_RATE_MAX_Q16 / PB_...     32 = +-488 ppm of authority
 */

#include "usbd_audio_if.h"
#include "stm32h7xx_hal.h"
#include <string.h>
#include <stdint.h>

/* -----------------------------------------------------------------------
 * Audio geometry */
#define AUDIO_CHANNELS      2U
#define BYTES_PER_SAMPLE    3U   /* 24-bit tightly packed on the wire; the
                                  * ring buffers below hold the unpacked
                                  * value as a sign-extended int32_t, not
                                  * 3 raw bytes -- this constant is about
                                  * USB packet/byte-count math only. */
#define SAMPLES_NOM         48U
/* SAMPLES_MIN is the underrun floor only. There is deliberately no
 * SAMPLES_MAX any more: the IN packet is now ALWAYS exactly SAMPLES_NOM
 * frames (see the fractional corrector below), so 49 has no caller. */
#define SAMPLES_MIN         47U
#define BYTES_NOM           (SAMPLES_NOM * AUDIO_CHANNELS * BYTES_PER_SAMPLE)  /* 288 */

/* -----------------------------------------------------------------------
 * Record ring tuning */
#define REC_RING_SIZE   512U
#define REC_TARGET      256U

/* ---- Continuous (fractional) rate correction, record side ------------
 *
 * Replaces the sigma-delta controller that varied the IN packet between
 * 47, 48 and 49 frames.
 *
 * WHY: this endpoint is declared Adaptive, which promises the host a
 * strictly constant frame rate. Varying the packet size broke that
 * promise on every correction, and a host that derives the device rate
 * from packet arrival cannot lock onto a stream whose size keeps
 * changing. Separately, every discrete correction is a step
 * discontinuity in the waveform; the correction RATE is fixed by the
 * clock offset and cannot be tuned away, so deadband/hysteresis only
 * choose between rare dense bursts and frequent thin ones.
 *
 * Instead the ring is read at a fractional position that advances by
 * nominally exactly one frame per output frame, tilted by a few hundred
 * ppm when the fill level needs to move, with the output linearly
 * interpolated between the two entries either side. The packet is always
 * SAMPLES_NOM frames, the descriptor tells the truth, and no
 * discontinuity is produced at all.
 *
 * Control law is plain proportional: rate = 1 + error/REC_RATE_DIV. No
 * integrator on purpose, so the ring settles at a small standing offset
 * proportional to the clock mismatch instead of hunting. Authority is
 * capped at REC_RATE_MAX_Q16 -- 488 ppm, far more than any real crystal
 * pairing needs and under a hundredth of a semitone, i.e. inaudible as
 * pitch, whereas one skipped sample is audible as a click. */
#define REC_RATE_DIV     8          /* error frames -> Q16 rate tilt */
#define REC_RATE_MAX_Q16 32         /* +-488 ppm of authority */
#define REC_FRAC_BITS    16
#define REC_FRAC_ONE     (1U << REC_FRAC_BITS)

/* -----------------------------------------------------------------------
 * Playback ring tuning */
#define PB_RING_SIZE    512U

/* Playback-side mirror of the record corrector above; same rationale,
 * same control law, same authority. */
#define PB_RATE_DIV      8          /* error frames -> Q16 rate tilt */
#define PB_RATE_MAX_Q16  32         /* +-488 ppm of authority */
#define PB_FRAC_BITS     16
#define PB_FRAC_ONE      (1U << PB_FRAC_BITS)

/* Fractional-delay reader. Both rings are read through this.
 *
 * FD_USE_SINC picks the interpolator at compile time so the two can be
 * A/B'd on hardware: 1 (default) is a polyphase windowed-sinc, 32 taps x
 * 128 phase rows with linear blending between rows, 16 KB table in
 * fd_sinc_table.h; 0 is Catmull-Rom, 4 taps, no table.
 *
 * The choice matters more than a static response curve suggests, because
 * the read phase sweeps 0->1 continuously at the clock offset and so
 * arrives as slow amplitude modulation rather than a fixed roll-off.
 * Measured error, AM depth and the Nyquist limit that no tap count can
 * fix are in docs/INTERPOLATION.md; tests/interp/check.sh reproduces them.
 *
 * Three things to know before editing this:
 *
 *   Ring headroom. The sinc reader touches idx-15 through idx+16 (the
 *   cubic, idx-1 through idx+2) against a TARGET fill of 256 frames in a
 *   512-frame ring, so neither approaches the write position in normal
 *   operation.
 *
 *   The clamp is load-bearing, not decorative. On material already
 *   hard-clipped at full scale the cubic overshoots by up to 1.02 dB and
 *   the sinc by up to 1.25 dB.
 *
 *   Float, not fixed point, deliberately: the M7 has a single-precision
 *   FPU with single-cycle FMA, a 24-bit sample is exact in a float32
 *   mantissa, and both kernels have coefficients outside the input range,
 *   which in fixed point would need headroom analysis for no benefit. */
#include "fd_sinc_table.h"

#ifndef FD_USE_SINC
#define FD_USE_SINC 1
#endif

#if ((REC_RING_SIZE) & ((REC_RING_SIZE) - 1U)) != 0U
#error "REC_RING_SIZE must be a power of two: fd_read_frame masks instead of dividing"
#endif
#if ((PB_RING_SIZE) & ((PB_RING_SIZE) - 1U)) != 0U
#error "PB_RING_SIZE must be a power of two: fd_read_frame masks instead of dividing"
#endif
#if (REC_FRAC_BITS != 16) || (PB_FRAC_BITS != 16)
#error "fd_read_frame assumes Q16 fractional read positions"
#endif

#define FD_SUB_BITS  (16 - FD_PHASE_BITS)
#define FD_SUB_ONE   (1U << FD_SUB_BITS)

/* How far ABOVE the integer read position fd_read_frame reaches. The
 * kernel is centred on FD_CENTER, so a read at `idx` touches ring slots
 * idx-FD_CENTER .. idx+(FD_TAPS-1-FD_CENTER). The playback gate has to
 * guarantee that much data beyond the block, not just the block itself,
 * or the tail of a block is interpolated from slots the host has not
 * written. */
#define FD_LOOKAHEAD (FD_TAPS - 1U - FD_CENTER)

static inline int32_t fd_clamp24(float y)
{
    if(y >  8388607.0f) return  8388607;
    if(y < -8388608.0f) return -8388608;
    return (int32_t)y;
}

/* ring:     interleaved frames, `channels` samples each
 * mask:     RING_SIZE - 1 (power of two enforced above)
 * idx:      integer read position, already reduced mod RING_SIZE
 * frac:     Q16 fractional part of the read position, 0..65535
 * out:      `channels` samples written */
static inline void fd_read_frame(const int32_t *ring, uint32_t mask,
                                 uint32_t channels, uint32_t idx,
                                 uint32_t frac, int32_t *out)
{
#if FD_USE_SINC
    const uint32_t p   = frac >> FD_SUB_BITS;          /* 0 .. FD_PHASES-1 */
    const uint32_t sub = frac & (FD_SUB_ONE - 1U);
    const float    fr  = (float)sub * (1.0f / (float)FD_SUB_ONE);
    const float   *a   = &fd_sinc_table[p * FD_TAPS];
    const float   *b   = &fd_sinc_table[(p + 1U) * FD_TAPS];

    float co[FD_TAPS];
    for(uint32_t k = 0; k < FD_TAPS; k++)
        co[k] = a[k] + fr * (b[k] - a[k]);

    const uint32_t base = (idx + (mask + 1U) - FD_CENTER) & mask;
    for(uint32_t ch = 0; ch < channels; ch++) {
        float acc = 0.0f;
        for(uint32_t k = 0; k < FD_TAPS; k++)
            acc += co[k] * (float)ring[(((base + k) & mask) * channels) + ch];
        out[ch] = fd_clamp24(acc);
    }
#else
    const uint32_t prv = (idx + mask) & mask;          /* idx - 1 */
    const uint32_t nxt = (idx + 1U)   & mask;
    const uint32_t nx2 = (idx + 2U)   & mask;
    const float    d   = (float)frac * (1.0f / 65536.0f);

    const int32_t *P = &ring[prv * channels];
    const int32_t *A = &ring[idx * channels];
    const int32_t *B = &ring[nxt * channels];
    const int32_t *C = &ring[nx2 * channels];

    for(uint32_t ch = 0; ch < channels; ch++) {
        const float w = (float)P[ch], x = (float)A[ch];
        const float y = (float)B[ch], z = (float)C[ch];
        const float c1 = 0.5f * (y - w);
        const float c2 = w - 2.5f * x + 2.0f * y - 0.5f * z;
        const float c3 = 0.5f * (z - w) + 1.5f * (x - y);
        out[ch] = fd_clamp24(((c3 * d + c2) * d + c1) * d + x);
    }
#endif
}

/* -----------------------------------------------------------------------
 * Diagnostics */
volatile uint32_t drift_corrections_up   = 0U;
volatile uint32_t drift_corrections_down = 0U;
volatile uint32_t rec_underrun_events    = 0U;
volatile uint32_t pb_overrun_events      = 0U;
volatile uint32_t pb_underrun_events     = 0U;
volatile uint32_t pb_last_packet_frames  = 0U;   /* num_frames in the most recent Audio_PlaybackData call */
volatile uint32_t pb_max_avail_seen      = 0U;   /* largest (pb_write - pb_read) ever observed by PopSamples */
volatile uint32_t pb_activation_count    = 0U;   /* how many times AudioIF_SetPlaybackActive(1) has fired */
volatile uint32_t pb_read_advances       = 0U;   /* how many times pb_read actually moved forward */

/* Falsifiable test of the central claim: the IN packet is ALWAYS
 * SAMPLES_NOM frames, which is what makes the Adaptive endpoint
 * descriptor honest. If this is ever non-zero the claim is false and the
 * descriptor is lying to the host -- there is no benign reading of it.
 * Incremented only by the safety clamp, which is the one remaining path
 * that can shorten a packet. */
volatile uint32_t tx_size_deviations     = 0U;

/* -----------------------------------------------------------------------
 * Split rec/pb drift diagnostics
 *
 * drift_corrections_up/down above are written by BOTH Audio_PeriodicTC
 * (rec side) and AudioIF_PopSamples (pb side) -- they're the sum of two
 * independent controllers, which makes it impossible to tell from the
 * combined counters alone whether rec or pb (or both) is the one
 * actually oscillating. These split counters track the exact same
 * correction events, just attributed to the side that fired them.
 * Kept as ADDITIONAL counters (not a replacement) so existing readers
 * of drift_corrections_up/down (main.cpp) keep working unchanged. */
volatile uint32_t rec_drift_corrections_up   = 0U;
volatile uint32_t rec_drift_corrections_down = 0U;
volatile uint32_t pb_drift_corrections_up    = 0U;
volatile uint32_t pb_drift_corrections_down  = 0U;

/* Min/max "available" frames ever observed on each ring, alongside the
 * existing pb_max_avail_seen. Together with the split correction
 * counters above, these let a host-side script see the actual envelope
 * each controller is operating within, not just "a correction fired". */
volatile uint32_t rec_max_avail_seen = 0U;
volatile uint32_t rec_min_avail_seen = 0xFFFFFFFFU;
volatile uint32_t pb_min_avail_seen  = 0xFFFFFFFFU;

/* Most recent "available" snapshot on each side -- lets the host poll a
 * live fill-level without waiting for a min/max extreme to be hit, and
 * without needing rec_write/rec_read/pb_write/pb_read exposed directly
 * (those stay static/internal to this file). */
volatile uint32_t rec_last_avail = 0U;
volatile uint32_t pb_last_avail  = 0U;

/* -----------------------------------------------------------------------
 * CRS (Clock Recovery System) sync-event counters.
 *
 * HSI48 (the USB peripheral's clock source, see main.cpp's InitCRS) is
 * only as accurate as CRS's live lock to the host's USB SOF timing. The
 * raw CRS->ISR flags are read-and-clear and polled only rarely (from the
 * USB control-request handler in usbd_audio.c), so a single poll can't
 * tell "locked continuously" apart from "locked once, then never
 * again" -- these cumulative counters, incremented every time
 * Audio_PeriodicTC observes a flag (roughly once per ms, i.e. far more
 * often than the host will poll), close that gap. A healthy lock looks
 * like sync_ok_count climbing steadily and sync_miss_count/trim_ovf_count
 * staying at 0; a rising sync_miss_count means HSI48 lost lock and is
 * free-running at its factory calibration (~+-1%) until the next
 * successful sync. */
volatile uint32_t crs_sync_ok_count    = 0U;
volatile uint32_t crs_sync_warn_count  = 0U;
volatile uint32_t crs_sync_miss_count  = 0U;
volatile uint32_t crs_trim_ovf_count   = 0U;

/* Deeper CRS diagnostics. The four cumulative counters above cannot
 * distinguish "SYNCOK fires but this loop never catches it set" from "CRS
 * produces no sync events at all". These two can.
 *
 *   crs_trim_snapshot is CRS->CR's TRIM field, sampled every call. It is
 *   not read-and-clear, so sampling it races with nothing. If CRS is
 *   really adjusting HSI48, TRIM drifts away from its startup value of 32
 *   and settles elsewhere. If it never leaves 32, CRS is doing nothing,
 *   whatever the flag counters say.
 *
 *   crs_poll_checked_count increments on every pass, so it tracks
 *   Audio_PeriodicTC's own ~1 kHz rate. If it is in the tens of thousands
 *   while crs_sync_ok_count stays at exactly 0, the polling loop is
 *   running and the explanation is either TRIM above or the flag never
 *   being set by hardware. */
volatile uint32_t crs_trim_snapshot      = 0xFFFFFFFFU;  /* sentinel until first read */
volatile uint32_t crs_poll_checked_count = 0U;
/* Debug: snapshot of first sample from last received packet. Declared here
 * (rather than next to Audio_PlaybackData further down) so that
 * AudioIF_SetPlaybackActive, which runs earlier in the file, can zero them
 * on gate-close without a forward declaration. */

/* Zero-crossing counter on incoming L channel — accumulated across every
 * Audio_PlaybackData call since the last time main.cpp read+reset it.
 * Purpose: distinguish "the tone is really encoded in the PCM bytes macOS
 * is sending" from "the tone is an artifact introduced somewhere after
 * this point (ring buffer, drift correction, SAI)". If the PCM itself is
 * a pure fixed-frequency tone, zero-crossings-per-second will be roughly
 * constant and match 2x the audible frequency. If the raw incoming data
 * is normal program material (music), the crossing rate will vary a lot
 * from one 250ms window to the next, since music isn't a single sine. */


/* -----------------------------------------------------------------------
 * Record ring
 *
 * CPU-only access (no USB DMA). Placed in BSS (AXI SRAM, cacheable).
 * DMA coherency is handled at haudio->in_buf via SCB_CleanDCache in
 * usbd_audio.c DataIn before USBD_LL_Transmit. */
static int32_t rec_ring[REC_RING_SIZE * AUDIO_CHANNELS];  /* sign-extended 24-bit, range +-8388607 */

static volatile uint32_t rec_write  = 0U;
static volatile uint32_t rec_read   = 0U;
static volatile uint8_t  rec_active = 0U;  /* pre-fill guard */
static          uint32_t rec_read_q16 = 0U;            /* fractional read position, Q16 frames */
static          uint32_t rec_step_q16 = REC_FRAC_ONE;  /* set by the controller each packet */

/* -----------------------------------------------------------------------
 * Playback ring
 *
 * Written by Audio_PlaybackData (DataOut ISR), read by AudioIF_PopSamples
 * (audio DMA ISR). SPSC — no mutex needed. */
static int32_t pb_ring[PB_RING_SIZE * AUDIO_CHANNELS];  /* sign-extended 24-bit, range +-8388607 */

static volatile uint32_t pb_write  = 0U;
static volatile uint32_t pb_read   = 0U;
static volatile uint8_t  pb_active = 0U;  /* 1 only while host SPK alt=1 is active */
static volatile uint8_t  pb_primed = 0U;  /* 1 once ring has reached PB_TARGET at least once */
static          uint32_t pb_read_q16 = 0U;            /* fractional read position, Q16 frames */
static          uint32_t pb_step_q16 = PB_FRAC_ONE;   /* set by the controller each block */

/* -----------------------------------------------------------------------
 * Prime fade-in
 *
 * When pb_primed flips 0->1 the output jumps from digital silence
 * straight to whatever sample sits at pb_read -- a step discontinuity of
 * arbitrary size, audible as a click on every start and every re-prime
 * after an underrun. The volume machinery further down cannot absorb it:
 * vol_gain_q16_current initialises to unity and only moves when the host
 * changes volume, so at a fresh start no ramp is in progress at all.
 *
 * A short linear ramp over PB_FADE_FRAMES fixes it. 64 frames at 48 kHz
 * is 1.33 ms -- far below the ~20 ms where a start delay becomes
 * perceptible, and long enough to spread the step well under the ear's
 * click threshold. Applied per-sample, unlike the volume ramp: a
 * per-block ramp on a 48-frame block would take 64 blocks = 85 ms, which
 * WOULD be an audible fade. Costs one multiply per sample and only while
 * the ramp is running -- once pb_fade_q16 saturates the loop skips it. */
#define PB_FADE_FRAMES    64U
#define PB_FADE_STEP_Q16  ((1U << 16) / PB_FADE_FRAMES)   /* 1024 */
static          uint32_t pb_fade_q16 = 0U;   /* 0..65536, unity when saturated */

/* Last frame actually emitted. pb_emit_silence ramps out of it, so every
 * transition into silence has a counterpart to the fade above. Written
 * only from the audio ISR, same as pb_fade_q16. */
static          int32_t  pb_last_out_l = 0;
static          int32_t  pb_last_out_r = 0;

/* Last gain actually applied to a sample, persisted across PopSamples
 * calls. The per-sample gain glide (see AudioIF_PopSamples) walks from
 * this value to the current block's settled gain, so the two ends of a
 * block boundary always meet -- a block-local variable would restart
 * from zero every callback and reintroduce the very discontinuity the
 * glide exists to remove. pb_gain_seeded distinguishes "no previous
 * block to glide from" (fresh prime) from "previous block ended at
 * zero", which are not the same thing. */
static          int32_t  pb_gain_l_q16  = 0;
static          int32_t  pb_gain_r_q16  = 0;
static          uint8_t  pb_gain_seeded = 0U;

/* See usbd_audio_if.h for why these exist and why a plain read is safe:
 * rec_step_q16/pb_step_q16 are each written only by their own ISR, and a
 * single int32_t load/store is atomic on Cortex-M7. Not marked volatile
 * on the getter's local copy since we WANT one honest read of the
 * current file-scope value each call, not a cached stale one -- and
 * these functions are never hot-path (called only from the control
 * endpoint, at most a few times a second). */
/* Vendor slots 10 and 11 carry the current rate tilt in Q16 -- that is
 * (step_q16 - FRAC_ONE), range +-RATE_MAX_Q16 (+-32), where 32 Q16 is
 * 488 ppm. Positive means the ring is filling and the reader is speeding
 * up to drain it. read_transport_diagnostics.py reads them as tilt under
 * CHECK 4; anything else reading those slots must use the same units. */
int32_t AudioIF_GetRecTiltQ16(void) { return (int32_t)rec_step_q16 - (int32_t)REC_FRAC_ONE; }
int32_t AudioIF_GetPbTiltQ16(void)  { return (int32_t)pb_step_q16  - (int32_t)PB_FRAC_ONE; }

/* -----------------------------------------------------------------------
 * MIDI CC parameter table
 *
 * dsp_params[cc] holds the most recent value (0–127) for each CC number.
 * Written by AudioIF_PushMIDI (USB ISR), read by AudioIF_GetCCParam
 * (DMA ISR). Single-byte volatile writes/reads are atomic on Cortex-M7. */
static volatile uint8_t dsp_params[128];

/* -----------------------------------------------------------------------
 * AudioIF_Reset — reset all state for a clean session start
 *
 * Called from usbd_audio.c on every SET_INTERFACE for IF1 or IF2.
 * Resets both rings so stale audio never leaks into a new session,
 * and clears MIDI params so no phantom CC values persist. */
void AudioIF_ResetRecord(void)
{
    rec_write  = 0U;  rec_read  = 0U;  rec_active = 0U;
    rec_read_q16 = 0U;  rec_step_q16 = REC_FRAC_ONE;
    memset(rec_ring, 0, sizeof(rec_ring));
}

void AudioIF_ResetPlayback(void)
{
    pb_write   = 0U;  pb_read   = 0U;  pb_primed = 0U;
    /* The fractional position must be reset together with the integer
     * one, or the reader resumes mid-frame into a ring just zeroed. */
    pb_read_q16 = 0U;  pb_step_q16 = PB_FRAC_ONE;
    /* Re-arm the prime fade so the next start ramps in from silence
     * rather than stepping onto whatever sample the ring holds. */
    pb_fade_q16 = 0U;
    pb_gain_seeded = 0U;
    /* pb_last_out_* is deliberately NOT cleared here: this also runs on
     * the SET_INTERFACE that closes the stream, and pb_emit_silence needs
     * the anchor to ramp out of. It clears the anchor itself once done. */
    memset(pb_ring, 0, sizeof(pb_ring));
}

/* Class-init reset only -- see the SCOPE WARNING in usbd_audio_if.h.
 * Do not call this from a SET_INTERFACE branch. */
void AudioIF_Reset(void)
{
    AudioIF_ResetRecord();
    AudioIF_ResetPlayback();
    memset((void *)dsp_params, 0, sizeof(dsp_params));
}

/* Playback ring fill target.
 *
 * The playback path cannot pace its own input: the host decides how many
 * bytes to send per ms (Adaptive sink) and the SAI/DMA framework requires
 * AudioIF_PopSamples to return exactly num_frames every callback, with no
 * way to ask for fewer. Meanwhile the two clocks — the host's USB frame
 * clock and the Daisy SAI clock — differ by tens of ppm even when both
 * nominally claim 48 kHz, so pb_ring drifts monotonically toward empty or
 * full and every overrun or underrun is an audible recurring glitch.
 *
 * PopSamples therefore still returns exactly num_frames, but reads them
 * from a fractional position whose advance rate is tilted by the fill
 * level relative to a target in the middle of the ring. Nothing is ever
 * skipped or repeated; the correction lands between samples. */
#define PB_TARGET        (PB_RING_SIZE / 2U)   /* 256 frames, 5.33ms — mirrors REC_TARGET */
/* Playback fractional-corrector constants are defined next to
 * PB_RING_SIZE near the top of this file, because the playback state
 * variables that initialise from PB_FRAC_ONE are declared before this
 * point. See the record-side block for the full rationale. */

/* -----------------------------------------------------------------------
 * AudioIF_SetPlaybackActive — called from usbd_audio.c on SET_INTERFACE
 * for the SPK streaming interface (wIndex==2), mirroring out_streaming.
 *
 * This exists because usbd_audio_if.c has no access to haudio->out_streaming
 * (that lives in the USBD_AUDIO_HandleTypeDef, private to usbd_audio.c).
 * AudioIF_PopSamples uses this flag to output honest silence whenever
 * the host has not got the SPK stream actively running -- rather than
 * trusting pb_write/pb_read arithmetic alone, which can show stale
 * "available" data left over from a previous activation (macOS often
 * toggles alt 1->0->1 a few times while negotiating a stream at first
 * connect; without this guard, leftover frames from an earlier toggle
 * keep circulating through the ring and are heard as a tiny looping
 * tone that persists even after playback stops on the host). */
void AudioIF_SetPlaybackActive(uint8_t active)
{
    if(active) {
        pb_activation_count++;
    } else {
        pb_write    = 0U;
        pb_read     = 0U;
        pb_read_q16 = 0U;
        pb_step_q16 = PB_FRAC_ONE;
        pb_primed   = 0U;
    }
    pb_active = active;
}

/* Read-only accessor for pb_active — see usbd_audio_if.h for rationale. */
uint8_t AudioIF_IsPlaybackActive(void)
{
    return pb_active;
}

/* Read-only accessor for rec_active — mirrors AudioIF_IsPlaybackActive.
 * Diagnostic: does the ring ever reach REC_TARGET and get marked active,
 * or does it stay stuck in pre-fill forever (which would mean either
 * Audio_PeriodicTC never runs, or AudioIF_PushSamples never fills the
 * ring — i.e. mic alt-interface was never activated by the host, or the
 * loopback call in AudioCallback isn't reaching it). */
uint8_t AudioIF_IsRecordActive(void)
{
    return rec_active;
}

/* Volume and mute, SPK Feature Unit ID=5.
 *
 * The gain is applied in software in AudioIF_PopSamples, not in a codec
 * register: the PCM3060 on this board is strapped in hardware control
 * mode, so its I2C attenuation registers are not listening. The board's
 * headphone wheel is a separate analog stage after the DAC and is
 * untouched by any of this; these controls are what the USB host adjusts.
 *
 *   vol_raw_dB           last value the host set, in UAC1's signed 1/256
 *                        dB format. Stored verbatim because GET_CUR must
 *                        echo it back exactly, or the host's slider drifts
 *                        from what it believes it set.
 *   vol_muted            independent of vol_raw_dB — UAC1 keeps Mute and
 *                        Volume separate on purpose, so this is a second
 *                        multiply rather than being folded into the dB.
 *   vol_gain_q16_target  linear gain derived from vol_raw_dB, Q16,
 *                        recomputed only when SET_CUR changes it.
 *   vol_gain_q16_current what is actually applied this callback, ramping
 *                        toward target by VOL_RAMP_STEP_Q16 per block so a
 *                        slider move cannot produce a same-sample jump. */
/* AUDIO_VOL_DEFAULT_DB_RAW must match AUDIO_VOL_MAX in usbd_audio.h
 * (0dB) -- defined here rather than included from there to avoid a
 * circular header dependency; usbd_audio.c's AUDIO_VOL_MAX is the
 * authoritative one advertised to the host, this is just the power-on
 * default applied before any SET_CUR has ever arrived. */
#define AUDIO_VOL_DEFAULT_DB_RAW ((int16_t)0x0000)

static volatile int16_t  vol_raw_dB           = AUDIO_VOL_DEFAULT_DB_RAW;
static volatile uint8_t  vol_muted            = 0U;
static volatile int32_t  vol_gain_q16_target  = (1 << 16); /* unity */
static volatile int32_t  vol_gain_q16_current = (1 << 16);

/* ~10ms full-scale ramp at 48kHz/48-frame blocks (48000/48 = 1000 calls/
 * sec -> 10 calls = 10ms). Small enough to be inaudible as a distinct
 * "fade" for normal slider drags, large enough that no single AudioCallback
 * block ever sees a full 0dB<->-96dB jump in one shot. */
#define VOL_RAMP_BLOCKS   10
#define VOL_RAMP_STEP_Q16 ((1 << 16) / VOL_RAMP_BLOCKS)

/* Output headroom, Q16. 65536 = unity = no attenuation.
 *
 * This is a pad for the DAC, not for the interpolator. fd_read_frame()
 * clamps to 24-bit range internally and this runs further down the
 * chain, so it cannot affect that clamp either way -- attenuating after
 * a clamp only makes an already-clipped signal quieter.
 *
 * It does not need to. Real programme material normalised to 0 dBFS,
 * pushed through every phase of the polyphase kernel, peaks at +0.01 dB
 * over full scale with 0.0004% of samples exceeding it. What does need a
 * pad is the PCM3060: it oversamples by 8x and its reconstruction filter
 * produces intersample peaks on material sitting at 0 dBFS, as any
 * oversampling DAC does. A small digital pad ahead of the converter is
 * the standard answer, and after the clamp is where it belongs.
 *
 * If the interpolator itself ever needs limiting, the attenuation has to
 * move inside fd_read_frame() ahead of fd_clamp24() -- raising it here
 * would only turn the output down further while clipping just as much.
 * That means a new argument on a reader shared with the record path and
 * kept in step across three copies by tests/check_reader_copies.py, so
 * it wants a measured reason first.
 *
 * 61869 = -0.5 dB. 65536 for unity if the DAC pad is not wanted. */
#define PB_HEADROOM_Q16   61869

/* dB (Q8, i.e. raw/256.0) -> Q16 linear gain. Uses the standard
 * gain = 10^(dB/20) relation. Table-free: STM32H7 has a hardware FPU,
 * so powf/log-free polynomial tricks aren't needed here -- a direct
 * expf-based computation is cheap enough to run once per SET_CUR
 * (which happens at human slider-drag rates, not per-sample). */
#include <math.h>
static int32_t db_raw_to_gain_q16(int16_t raw_db_q8)
{
    float db = (float)raw_db_q8 / 256.0f;
    float linear = powf(10.0f, db / 20.0f);
    if(linear < 0.0f) linear = 0.0f;
    if(linear > 4.0f) linear = 4.0f; /* generous headroom clamp; our
                                       * advertised range tops out at
                                       * 0dB (linear=1.0) anyway, this
                                       * just guards the float->fixed
                                       * conversion below from overflow
                                       * if something ever sends an
                                       * out-of-spec raw value that
                                       * slipped past the clamp in
                                       * usbd_audio.c. */
    return (int32_t)(linear * 65536.0f);
}

void AudioIF_SetVolumeRaw(int16_t raw_db_q8)
{
    vol_raw_dB          = raw_db_q8;
    vol_gain_q16_target = db_raw_to_gain_q16(raw_db_q8);
}

int16_t AudioIF_GetVolumeRaw(void)
{
    return vol_raw_dB;
}

void AudioIF_SetMuteState(uint8_t muted)
{
    vol_muted = muted ? 1U : 0U;
}

uint8_t AudioIF_GetMuteState(void)
{
    return vol_muted;
}

/* -----------------------------------------------------------------------
 * SPK per-channel (Left/Right) gain -- independent state from the master
 * block above. Combined with it as master_gain * channel_gain in
 * AudioIF_PopSamples (mute is master_mute || channel_mute). This is what
 * lets Audio MIDI Setup show working, independent L/R sliders instead of
 * the greyed-out ones the old bmaControls(1)=0x00 / bmaControls(2)=0x00
 * feature-unit descriptor forced. */
static volatile int16_t  vol_raw_dB_l           = AUDIO_VOL_DEFAULT_DB_RAW;
static volatile uint8_t  vol_muted_l            = 0U;
static volatile int32_t  vol_gain_q16_target_l  = (1 << 16);
static volatile int32_t  vol_gain_q16_current_l = (1 << 16);

static volatile int16_t  vol_raw_dB_r           = AUDIO_VOL_DEFAULT_DB_RAW;
static volatile uint8_t  vol_muted_r            = 0U;
static volatile int32_t  vol_gain_q16_target_r  = (1 << 16);
static volatile int32_t  vol_gain_q16_current_r = (1 << 16);

void AudioIF_SetVolumeRawL(int16_t raw_db_q8)
{
    vol_raw_dB_l          = raw_db_q8;
    vol_gain_q16_target_l = db_raw_to_gain_q16(raw_db_q8);
}

int16_t AudioIF_GetVolumeRawL(void)
{
    return vol_raw_dB_l;
}

void AudioIF_SetMuteStateL(uint8_t muted)
{
    vol_muted_l = muted ? 1U : 0U;
}

uint8_t AudioIF_GetMuteStateL(void)
{
    return vol_muted_l;
}

void AudioIF_SetVolumeRawR(int16_t raw_db_q8)
{
    vol_raw_dB_r          = raw_db_q8;
    vol_gain_q16_target_r = db_raw_to_gain_q16(raw_db_q8);
}

int16_t AudioIF_GetVolumeRawR(void)
{
    return vol_raw_dB_r;
}

void AudioIF_SetMuteStateR(uint8_t muted)
{
    vol_muted_r = muted ? 1U : 0U;
}

uint8_t AudioIF_GetMuteStateR(void)
{
    return vol_muted_r;
}

/* -----------------------------------------------------------------------
 * MIC/record-side gain -- independent state from the SPK block above.
 *
 * Applied where the record path actually assembles the outgoing USB
 * packet (Audio_PeriodicTC, via copy_frames_from_rec_ring), not where
 * samples enter rec_ring (AudioIF_PushSamples) -- same reasoning as
 * SPK: keep the ring itself holding raw, ungained ADC data, and apply
 * gain only once, right before it leaves the device, so nothing
 * upstream (sigma-delta drift math, overrun/underrun accounting) ever
 * has to reason about a scaled signal. */
static volatile int16_t  vol_raw_dB_mic           = AUDIO_VOL_DEFAULT_DB_RAW;
static volatile uint8_t  vol_muted_mic            = 0U;
static volatile int32_t  vol_gain_q16_target_mic  = (1 << 16);
static volatile int32_t  vol_gain_q16_current_mic = (1 << 16);

void AudioIF_SetVolumeRawMic(int16_t raw_db_q8)
{
    vol_raw_dB_mic          = raw_db_q8;
    vol_gain_q16_target_mic = db_raw_to_gain_q16(raw_db_q8);
}

int16_t AudioIF_GetVolumeRawMic(void)
{
    return vol_raw_dB_mic;
}

void AudioIF_SetMuteStateMic(uint8_t muted)
{
    vol_muted_mic = muted ? 1U : 0U;
}

uint8_t AudioIF_GetMuteStateMic(void)
{
    return vol_muted_mic;
}

/* -----------------------------------------------------------------------
 * Audio_Init / stubs */
static int8_t Audio_Init(uint32_t AudioFreq, uint32_t Volume, uint32_t options)
{
    (void)AudioFreq; (void)Volume; (void)options;
    drift_corrections_up   = 0U;
    drift_corrections_down = 0U;
    rec_underrun_events    = 0U;
    pb_overrun_events      = 0U;
    pb_underrun_events     = 0U;
    vol_raw_dB             = AUDIO_VOL_DEFAULT_DB_RAW;
    vol_muted              = 0U;
    vol_gain_q16_target    = (1 << 16);
    vol_gain_q16_current   = (1 << 16);
    vol_raw_dB_l            = AUDIO_VOL_DEFAULT_DB_RAW;
    vol_muted_l             = 0U;
    vol_gain_q16_target_l   = (1 << 16);
    vol_gain_q16_current_l  = (1 << 16);
    vol_raw_dB_r            = AUDIO_VOL_DEFAULT_DB_RAW;
    vol_muted_r             = 0U;
    vol_gain_q16_target_r   = (1 << 16);
    vol_gain_q16_current_r  = (1 << 16);
    vol_raw_dB_mic             = AUDIO_VOL_DEFAULT_DB_RAW;
    vol_muted_mic              = 0U;
    vol_gain_q16_target_mic    = (1 << 16);
    vol_gain_q16_current_mic   = (1 << 16);
    AudioIF_Reset();
    return USBD_OK;
}

static int8_t Audio_DeInit   (uint32_t o)                        { (void)o; return USBD_OK; }
static int8_t Audio_AudioCmd (uint8_t *p, uint32_t s, uint8_t c) { (void)p;(void)s;(void)c; return USBD_OK; }
/* Audio_VolumeCtl/Audio_MuteCtl (the USBD_AUDIO_ItfTypeDef fops callbacks)
 * are intentionally left as no-op stubs. Their signatures predate this
 * fix (VolumeCtl takes a single uint8_t, which cannot carry a signed
 * 16-bit dB value) and usbd_audio.c now calls AudioIF_SetVolumeRaw() /
 * AudioIF_SetMuteState() directly instead -- see the comment at that
 * call site. Kept as stubs rather than removed because
 * USBD_AUDIO_ItfTypeDef (usbd_audio.h) requires non-NULL function
 * pointers for every fops member. */
static int8_t Audio_VolumeCtl(uint8_t v)                         { (void)v; return USBD_OK; }
static int8_t Audio_MuteCtl  (uint8_t c)                         { (void)c; return USBD_OK; }
static int8_t Audio_GetState (void)                              { return USBD_OK; }

/* -----------------------------------------------------------------------
 * 24-bit <-> byte-stream helpers (tightly packed, 3 bytes/sample, LE --
 * matches bSubframeSize=3 in both Format Type descriptors in
 * usbd_audio.c). Used at both USB boundaries: unpacking incoming SPK
 * bytes in Audio_PlaybackData, and packing outgoing MIC bytes in
 * copy_frames_from_rec_ring. The ring buffers themselves (pb_ring,
 * rec_ring) hold the *unpacked* value as a sign-extended int32_t the
 * rest of this file works with -- these two functions are the only
 * places that ever touch the 3-byte wire representation directly. */
static inline int32_t unpack24_le(const uint8_t *p)
{
    /* Assemble the 24-bit value into the top 3 bytes of a uint32_t, then
     * arithmetic-shift right by 8 -- this sign-extends bit 23 into bits
     * 31..24 in one step, cheaper and less error-prone than a manual
     * "if (v & 0x800000) v |= 0xFF000000" branch. */
    uint32_t u = ((uint32_t)p[0])       |
                 ((uint32_t)p[1] << 8)  |
                 ((uint32_t)p[2] << 16);
    return ((int32_t)(u << 8)) >> 8;
}

static inline void pack24_le(uint8_t *p, int32_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
    p[2] = (uint8_t)((v >> 16) & 0xFF);
}

/* Q16 gain multiply for a 24-bit-range sample. Needs a 64-bit
 * intermediate: sample range is +-2^23, gain up to 4.0 in Q16
 * (db_raw_to_gain_q16's headroom clamp) = +-2^18, product up to
 * +-2^41 -- overflows int32_t (this is why the old 16-bit version's
 * plain "(int32_t)sample * gain_q16" was safe at +-2^15 * +-2^18 =
 * +-2^33... which actually already overflowed a 32-bit signed int too,
 * technically undefined behaviour that happened to work because the
 * true result always fit back into int32 range after the >>16 -- worth
 * being explicit and correct here now that the margins are tighter). */
static inline int32_t apply_gain_q16_24bit(int32_t sample, int32_t gain_q16)
{
    int64_t product = (int64_t)sample * (int64_t)gain_q16;
    int32_t out = (int32_t)(product >> 16);
    if(out >  8388607)  out =  8388607;
    if(out < -8388608)  out = -8388608;
    return out;
}

/* -----------------------------------------------------------------------
 * copy_frames_from_rec_ring — internal
 *
 * Was a pair of memcpy() calls before this fix -- switched to an
 * explicit per-frame loop so MIC input gain (vol_gain_q16_current_mic)
 * can be applied here, the same way SPK gain is applied in
 * AudioIF_PopSamples. Ramp advances once per call (i.e. once per 1ms
 * USB DataIn tick), not once per frame within the call, for the same
 * reason as the SPK side: a per-frame ramp step would make the
 * intended ~10ms transition take roughly num_frames times longer than
 * designed. */
static void copy_frames_from_rec_ring(uint8_t *dst, uint32_t n)
{
    /* Mute folded into the target, same fix as the playback side -- see
     * the comment in AudioIF_PopSamples. The mic path had the identical
     * unramped `vol_muted_mic ? 0 : ...` gate below, putting a click into
     * the RECORDED stream for the same reason. */
    const int32_t tgt_mic = vol_muted_mic ? 0 : vol_gain_q16_target_mic;

    if(vol_gain_q16_current_mic < tgt_mic) {
        vol_gain_q16_current_mic += VOL_RAMP_STEP_Q16;
        if(vol_gain_q16_current_mic > tgt_mic)
            vol_gain_q16_current_mic = tgt_mic;
    } else if(vol_gain_q16_current_mic > tgt_mic) {
        vol_gain_q16_current_mic -= VOL_RAMP_STEP_Q16;
        if(vol_gain_q16_current_mic < tgt_mic)
            vol_gain_q16_current_mic = tgt_mic;
    }
    int32_t gain_q16 = vol_gain_q16_current_mic;  /* mute already in the target */

    /* Fractional read through fd_read_frame(). The position advances by
     * rec_step_q16 per frame -- nominally exactly one -- so a rate
     * correction shows up as every output sample being a slightly
     * different blend rather than as one sample being dropped or
     * repeated. See the fd_read_frame block above for the interpolator
     * choice and what it costs. */
    for(uint32_t i = 0; i < n; i++) {
        uint32_t idx  = (rec_read_q16 >> REC_FRAC_BITS) % REC_RING_SIZE;
        uint32_t frac = rec_read_q16 & (REC_FRAC_ONE - 1U);

        int32_t frame[AUDIO_CHANNELS];
        fd_read_frame(rec_ring, REC_RING_SIZE - 1U, AUDIO_CHANNELS,
                      idx, frac, frame);
        int32_t l = frame[0];
        int32_t r = frame[1];

        int32_t out_l = apply_gain_q16_24bit(l, gain_q16);
        int32_t out_r = apply_gain_q16_24bit(r, gain_q16);
        pack24_le(&dst[(i * AUDIO_CHANNELS + 0) * BYTES_PER_SAMPLE], out_l);
        pack24_le(&dst[(i * AUDIO_CHANNELS + 1) * BYTES_PER_SAMPLE], out_r);

        rec_read_q16 = (rec_read_q16 + rec_step_q16)
                       % (REC_RING_SIZE << REC_FRAC_BITS);
    }
    /* Integer rec_read is what the fill-level maths and the diagnostics
     * read, so keep it in step with the real fractional position. */
    rec_read = (rec_read_q16 >> REC_FRAC_BITS) % REC_RING_SIZE;
}

/* -----------------------------------------------------------------------
 * Audio_PeriodicTC — USB DataIn ISR, every 1ms
 *
 * Proportional fractional-rate controller:
 *   error = available - REC_TARGET  (signed)
 *   rec_step_q16 = FRAC_ONE + clamp(error / REC_RATE_DIV, +-REC_RATE_MAX_Q16)
 * The packet returned is always SAMPLES_NOM frames.
 *
 * Correction rate proportional to error magnitude (unlike bang-bang which
 * always corrects at the same rate once outside the deadband).
 * Underrun threshold corrected to SAMPLES_MIN (47) — allows sending a
 * 47-frame packet when exactly 47 frames are available instead of
 * wasting them with a reset at SAMPLES_NOM (48). */
static uint16_t Audio_PeriodicTC(uint8_t *pbuf, uint32_t size, uint8_t cmd)
{
    (void)size;
    if(cmd != AUDIO_IN_TC) return BYTES_NOM;

    /* CRS sync-event polling -- see crs_sync_ok_count etc. declarations
     * above for why this needs to happen here (~1kHz) rather than only
     * when a host control-request comes in. Read-and-clear, same
     * pattern HAL_RCCEx_CRSWaitSynchronization itself uses. */
    crs_poll_checked_count++;
    /* CRS->CR bits [13:8] = TRIM (6-bit HSI48 trim value, default 32 at
     * reset/HSI48CalibrationValue, hardware-adjusted when AUTOTRIMEN is
     * set -- confirmed against the STM32H743 SVD register map). Shifting
     * by the literal bit position (8) rather than a *_Pos/_BITNUMBER
     * macro name, since those aren't consistently named across HAL
     * versions/vendors and getting this wrong would silently read the
     * wrong bits instead of failing to compile. Not read-and-clear
     * (unlike the ISR flags below), so reading it here is always safe
     * regardless of what Audio_PeriodicTC's own flag polling below
     * finds on this particular call. */
    crs_trim_snapshot = (CRS->CR >> 8U) & 0x3FU;
    if(__HAL_RCC_CRS_GET_FLAG(RCC_CRS_FLAG_SYNCOK))   { crs_sync_ok_count++;   __HAL_RCC_CRS_CLEAR_FLAG(RCC_CRS_FLAG_SYNCOK); }
    if(__HAL_RCC_CRS_GET_FLAG(RCC_CRS_FLAG_SYNCWARN)) { crs_sync_warn_count++; __HAL_RCC_CRS_CLEAR_FLAG(RCC_CRS_FLAG_SYNCWARN); }
    if(__HAL_RCC_CRS_GET_FLAG(RCC_CRS_FLAG_SYNCMISS)) { crs_sync_miss_count++; __HAL_RCC_CRS_CLEAR_FLAG(RCC_CRS_FLAG_SYNCMISS); }
    if(__HAL_RCC_CRS_GET_FLAG(RCC_CRS_FLAG_TRIMOVF))  { crs_trim_ovf_count++;  __HAL_RCC_CRS_CLEAR_FLAG(RCC_CRS_FLAG_TRIMOVF); }

    uint32_t available = (rec_write - rec_read + REC_RING_SIZE) % REC_RING_SIZE;

    /* Pre-fill: output silence until ring reaches target level */
    if(!rec_active) {
        if(available >= REC_TARGET)
            rec_active = 1U;
        else {
            memset(pbuf, 0, BYTES_NOM);
            return BYTES_NOM;
        }
    }

    /* Underrun: fewer frames than minimum packet — re-prime.
     *
     * The fractional read position is deliberately NOT reset here, for the
     * same reason the playback side gives at its own underrun gate.
     *
     * rec_read_q16 used to be zeroed on this path while rec_read was left
     * where it was. Those two are one pointer in two representations:
     * copy_frames_from_rec_ring reads from rec_read_q16 and then writes
     * rec_read back from it, precisely so the fill-level arithmetic and
     * the diagnostics have an integer copy that matches (see the comment
     * at that assignment). Zeroing one and not the other breaks that
     * correspondence for the whole silence window: `available` above goes
     * on being measured from the old rec_read while the reader would
     * restart from slot 0, so the pre-fill gate opens on a frame count
     * that is not the one the reader actually has in front of it.
     *
     * Measured on the record-path host simulation in this tree, forcing
     * an underrun by stalling the writer while the host keeps draining:
     * with the reset, the gate opened on 288 frames when the reader had
     * 224, and the ring re-primed at 176 against a target of 256. Without
     * it, `available` and the reader's true distance to the writer agree
     * throughout, and the ring re-primes at 240. Same underrun count,
     * across stall lengths from 4 to 25 ms; no state appears that the
     * previous version did not also reach.
     *
     * Leaving the position put costs nothing: rec_active is already 0, so
     * copy_frames_from_rec_ring is not called until the REC_TARGET gate
     * above re-opens, and rec_read_q16 is wrapped modulo the ring on
     * every advance, so it cannot be stale in any way that matters. Only
     * rec_step_q16 needs clearing, because the controller's correction is
     * meaningless once the stream has broken continuity. */
    if(available < SAMPLES_MIN) {
        rec_active   = 0U;
        rec_step_q16 = REC_FRAC_ONE;
        rec_underrun_events++;
        memset(pbuf, 0, BYTES_NOM);
        return BYTES_NOM;
    }

    /* Diagnostics: track this call's fill level regardless of which
     * branch the controller below takes. Done here (after the pre-fill/
     * underrun early-returns above) so these reflect steady-state
     * operation, not the pre-fill silence period. */
    rec_last_avail = available;
    if(available > rec_max_avail_seen) rec_max_avail_seen = available;
    if(available < rec_min_avail_seen) rec_min_avail_seen = available;

    /* Proportional rate tilt. No deadband and no hysteresis: those
     * existed only to schedule WHEN discrete skips happened, and there
     * are no discrete skips left to schedule. */
    int32_t error = (int32_t)available - (int32_t)REC_TARGET;
    int32_t tilt  = error / REC_RATE_DIV;
    if(tilt >  REC_RATE_MAX_Q16) tilt =  REC_RATE_MAX_Q16;
    if(tilt < -REC_RATE_MAX_Q16) tilt = -REC_RATE_MAX_Q16;
    /* error > 0 means the ring is filling: read slightly faster. */
    rec_step_q16 = (uint32_t)((int32_t)REC_FRAC_ONE + tilt);

    /* These counters now report how hard the corrector is currently
     * pulling, not how many discontinuities were emitted -- there are
     * none to emit. */
    if(tilt > 0)      { drift_corrections_up++;   rec_drift_corrections_up++; }
    else if(tilt < 0) { drift_corrections_down++; rec_drift_corrections_down++; }

    /* The packet is ALWAYS exactly SAMPLES_NOM frames now -- that is the
     * whole point of the fractional reader, and what makes the Adaptive
     * descriptor honest. */
    uint32_t tx_samples = SAMPLES_NOM;

    if(tx_samples > available) {
        tx_samples = available;      /* safety clamp */
        tx_size_deviations++;        /* see the declaration -- must stay 0 */
    }

    copy_frames_from_rec_ring(pbuf, tx_samples);
    return (uint16_t)(tx_samples * AUDIO_CHANNELS * BYTES_PER_SAMPLE);
}

/* -----------------------------------------------------------------------
 * Audio_PlaybackData — DataOut ISR callback (host → speaker)
 *
 * Converts raw USB bytes to int16 frames and pushes into pb_ring.
 * Drops frames when ring is full (counts pb_overrun_events). */
static void Audio_PlaybackData(uint8_t *pbuf, uint16_t size)
{
    uint32_t num_frames = (uint32_t)size / (AUDIO_CHANNELS * BYTES_PER_SAMPLE);
    pb_last_packet_frames = num_frames;

    /* pbuf is 3-byte-packed 24-bit samples now, not a natural C type --
     * can't pointer-cast the way the 16-bit version did. unpack24_le()
     * reads each sample explicitly, once per frame. */
    for(uint32_t i = 0; i < num_frames; i++) {
        const uint8_t *frame = &pbuf[i * AUDIO_CHANNELS * BYTES_PER_SAMPLE];
        int32_t l = unpack24_le(&frame[0 * BYTES_PER_SAMPLE]);
        int32_t r = unpack24_le(&frame[1 * BYTES_PER_SAMPLE]);

        uint32_t pos  = pb_write % PB_RING_SIZE;
        uint32_t next = (pos + 1U) % PB_RING_SIZE;

        if(next == (pb_read % PB_RING_SIZE)) {
            pb_overrun_events++;
            return;  /* full — drop remaining frames */
        }

        pb_ring[pos * AUDIO_CHANNELS + 0] = l;
        pb_ring[pos * AUDIO_CHANNELS + 1] = r;
        pb_write = (pb_write + 1U) % PB_RING_SIZE;
    }
}

/* -----------------------------------------------------------------------
 * USBD_AUDIO_fops */
USBD_AUDIO_ItfTypeDef USBD_AUDIO_fops = {
    Audio_Init,
    Audio_DeInit,
    Audio_AudioCmd,
    Audio_VolumeCtl,
    Audio_MuteCtl,
    Audio_PeriodicTC,
    Audio_GetState,
    Audio_PlaybackData,
};

/* -----------------------------------------------------------------------
 * AudioIF_PushSamples — record path, call from AudioCallback (DMA ISR) */
void AudioIF_PushSamples(int32_t *samples, uint32_t num_frames)
{
    for(uint32_t i = 0; i < num_frames; i++) {
        uint32_t pos  = rec_write % REC_RING_SIZE;
        uint32_t next = (pos + 1U) % REC_RING_SIZE;
        if(next == (rec_read % REC_RING_SIZE)) return;  /* full */

        rec_ring[pos * AUDIO_CHANNELS + 0] = samples[i * AUDIO_CHANNELS + 0];
        rec_ring[pos * AUDIO_CHANNELS + 1] = samples[i * AUDIO_CHANNELS + 1];
        rec_write = (rec_write + 1U) % REC_RING_SIZE;
    }
}

/* One block of silence, ramping out of the last frame emitted.
 *
 * Three paths produce silence -- host closed the interface, ring not yet
 * primed, ring ran dry -- and all three share this so they cannot drift
 * apart. pb_fade_q16 ramps silence -> audio; this is its counterpart.
 * Without it the output steps from whatever sample it was on to digital
 * zero inside one sample period, which is a click.
 *
 * The ramp spans the whole block so it lands exactly on zero at the last
 * sample; the anchor is then cleared, so further silent blocks cost two
 * compares and a memset. */
static void pb_emit_silence(int32_t *dst, uint32_t num_frames)
{
    if(pb_last_out_l == 0 && pb_last_out_r == 0) {
        memset(dst, 0, num_frames * AUDIO_CHANNELS * sizeof(int32_t));
        return;
    }

    const int32_t l0   = pb_last_out_l;
    const int32_t r0   = pb_last_out_r;
    const int32_t step = (int32_t)((1U << 16) / num_frames);
    int32_t       g    = (int32_t)(1U << 16) - step;

    for(uint32_t i = 0; i < num_frames; i++) {
        if(g < 0) g = 0;
        dst[i * AUDIO_CHANNELS + 0] = apply_gain_q16_24bit(l0, g);
        dst[i * AUDIO_CHANNELS + 1] = apply_gain_q16_24bit(r0, g);
        g -= step;
    }
    pb_last_out_l = 0;
    pb_last_out_r = 0;
}

/* -----------------------------------------------------------------------
 * AudioIF_PopSamples — playback path, call from AudioCallback (DMA ISR)
 *
 * Always fills dst with num_frames. Zero-pads on underrun so AudioCallback
 * needs no conditional — just call and use the output. */
void AudioIF_PopSamples(int32_t *dst, uint32_t num_frames)
{
    if(!pb_active) {
        pb_emit_silence(dst, num_frames);
        return;
    }

    uint32_t avail = (pb_write - pb_read + PB_RING_SIZE) % PB_RING_SIZE;
    if(avail > pb_max_avail_seen) pb_max_avail_seen = avail;

    /* Pre-fill: output silence until ring reaches target level at least
     * once. Mirrors the record-side rec_active gate — without this,
     * PopSamples starts draining the ring the instant the first USB
     * packet lands, so the very first few ms are built from a ring
     * that's nowhere near steady-state, guaranteeing an immediate
     * underrun before the drift controller below has any data to work
     * with. */
    if(!pb_primed) {
        if(avail >= PB_TARGET) {
            pb_primed = 1U;
            pb_fade_q16 = 0U;      /* ramp in from silence */
            pb_gain_seeded = 0U;   /* no previous block to glide from */
        } else {
            pb_emit_silence(dst, num_frames);
            return;
        }
    }

    /* Ring ran dry, or nearly dry, despite priming. That is a real
     * underrun rather than drift, and it has to be caught before the rate
     * controller below runs.
     *
     * The gate must be a full block plus the kernel's reach, not merely
     * "some data present". At 0 < avail <= num_frames there is nothing
     * left to read partway through the block and the reader would hold
     * its last sample for the remaining slots: irregular repeat runs
     * whose length tracks whatever avail happened to be, measured from
     * 13 to 300 samples, which reads as harsh crackle rather than as a
     * change in pitch.
     *
     * Why avail gets that low is a separate question of host pacing. The
     * response is the same either way: stop, re-prime, and let the
     * PB_TARGET gate rebuild a healthy buffer. Never synthesise output by
     * holding a stale sample.
     *
     * The threshold covers FD_LOOKAHEAD beyond the block because that is
     * how far the interpolation kernel reaches above the read position;
     * see FD_LOOKAHEAD above.
     *
     * The read position is deliberately left where it is. Resetting it
     * would move the reader an arbitrary distance from pb_write while
     * avail is still measured from the old position, so the PB_TARGET
     * gate would then be judging a buffer that is not the one being
     * read. Leaving it put, avail keeps its faulted value, the gate holds
     * silence until the host refills, and playback resumes at the exact
     * fractional position it stopped at.
     *
     * pb_step_q16 does reset: the tilt in force belonged to a fill level
     * that no longer exists. */
    if(avail < num_frames + FD_LOOKAHEAD) {
        pb_primed   = 0U;
        pb_step_q16 = PB_FRAC_ONE;
        pb_fade_q16 = 0U;   /* re-prime fades in, same as a fresh start */
        pb_gain_seeded = 0U;
        pb_underrun_events++;

        pb_emit_silence(dst, num_frames);
        return;
    }


    /* avail >= num_frames+1 is guaranteed by the gate above, so the read
     * position below can never outrun the write position. The old safety
     * clamp that silently created the repeat-run bug is gone: the case is
     * structurally unreachable now, not merely guarded. */
    pb_last_avail = avail;
    if(avail < pb_min_avail_seen) pb_min_avail_seen = avail;
    /* pb_max_avail_seen already tracked above, before the primed/underrun
     * gates -- left as-is so its long-standing meaning doesn't shift. */

    /* Proportional rate tilt -- playback mirror of Audio_PeriodicTC.
     *
     * This replaces the read_steps skip/repeat machinery entirely, and
     * with it a whole class of bug: the old "+1" branch could not work,
     * because the output loop is bounded by `i < num_frames` and can
     * never iterate more times than that no matter what read_steps says.
     * The accumulator and the counters were updated as if the correction
     * had happened while pb_read only ever advanced by num_frames, so
     * only one of the corrector's two directions was structurally
     * capable of doing anything. A fractional position has no such
     * asymmetry: both directions are the same arithmetic. */
    int32_t error = (int32_t)avail - (int32_t)PB_TARGET;
    int32_t tilt  = error / PB_RATE_DIV;
    if(tilt >  PB_RATE_MAX_Q16) tilt =  PB_RATE_MAX_Q16;
    if(tilt < -PB_RATE_MAX_Q16) tilt = -PB_RATE_MAX_Q16;
    pb_step_q16 = (uint32_t)((int32_t)PB_FRAC_ONE + tilt);

    if(tilt > 0)      { drift_corrections_up++;   pb_drift_corrections_up++; }
    else if(tilt < 0) { drift_corrections_down++; pb_drift_corrections_down++; }

    int32_t last_l = 0, last_r = 0;
    int32_t combined_l_q16 = 0, combined_r_q16 = 0;
    int32_t combined_l_step = 0, combined_r_step = 0;
    for(uint32_t i = 0; i < num_frames; i++) {
        {
            uint32_t idx  = (pb_read_q16 >> PB_FRAC_BITS) % PB_RING_SIZE;
            uint32_t frac = pb_read_q16 & (PB_FRAC_ONE - 1U);

            int32_t frame[AUDIO_CHANNELS];
            fd_read_frame(pb_ring, PB_RING_SIZE - 1U, AUDIO_CHANNELS,
                          idx, frac, frame);
            last_l = frame[0];
            last_r = frame[1];

            uint32_t before = pb_read_q16 >> PB_FRAC_BITS;
            pb_read_q16 = (pb_read_q16 + pb_step_q16)
                          % (PB_RING_SIZE << PB_FRAC_BITS);
            uint32_t after = pb_read_q16 >> PB_FRAC_BITS;
            if(after != before) pb_read_advances++;
            pb_read = after;
        }
        /* Volume/mute gain -- see the vol_* block above Audio_Init for
         * the full rationale (no hardware attenuation path on this
         * board's PCM3060, so this is genuinely the only place volume
         * can be applied). Ramp vol_gain_q16_current toward
         * vol_gain_q16_target by at most one step per audio block
         * (not per sample -- one ramp step per 48-frame block is
         * already a ~10ms full-scale transition, doing it per-sample
         * would make an inaudibly slow ramp take 48x longer than
         * intended). Multiply happens in 32-bit before truncating back
         * to int16, so the intermediate product isn't clipped to 16
         * bits before the final >>16 shift -- this is the "compute in
         * higher precision, only round down once at the very end"
         * practice described earlier, done here in fixed-point since
         * this runs in an ISR context where an FPU round-trip per
         * sample would be wasteful (the dB->linear conversion above is
         * the only place float actually gets used, once per SET_CUR). */
        if(i == 0) {
            /* Mute is folded into the ramp TARGET, never applied to the
             * ramp's output. Gating the output instead --
             *
             *     master_gain_q16 = vol_muted ? 0 : vol_gain_q16_current;
             *
             * -- discards the ramp entirely for the mute case and takes
             * the output from full scale to digital zero between one
             * 48-frame block and the next, and back on unmute. That is a
             * hard step edge on every toggle, and hosts write the Feature
             * Unit's mute while bringing a stream up, not only when a
             * user moves a control, so it fires during ordinary stream
             * setup rather than only on user action.
             *
             * No counter can see it either way: this is downstream of the
             * ring, so no frame is dropped, repeated or re-primed.
             *
             * As a target, mute walks to zero over VOL_RAMP_BLOCKS
             * (~10 ms) and unmute walks back the same way. Zero is still
             * reached exactly, just over 10 ms instead of instantly. */
            const int32_t tgt_master = vol_muted   ? 0 : vol_gain_q16_target;
            const int32_t tgt_l      = vol_muted_l ? 0 : vol_gain_q16_target_l;
            const int32_t tgt_r      = vol_muted_r ? 0 : vol_gain_q16_target_r;

            if(vol_gain_q16_current < tgt_master) {
                vol_gain_q16_current += VOL_RAMP_STEP_Q16;
                if(vol_gain_q16_current > tgt_master)
                    vol_gain_q16_current = tgt_master;
            } else if(vol_gain_q16_current > tgt_master) {
                vol_gain_q16_current -= VOL_RAMP_STEP_Q16;
                if(vol_gain_q16_current < tgt_master)
                    vol_gain_q16_current = tgt_master;
            }

            /* Per-channel ramps run independently of the master ramp and
             * of each other, same VOL_RAMP_STEP_Q16 rate, once per block
             * (i==0), for the same zipper-noise reasons as master above. */
            if(vol_gain_q16_current_l < tgt_l) {
                vol_gain_q16_current_l += VOL_RAMP_STEP_Q16;
                if(vol_gain_q16_current_l > tgt_l)
                    vol_gain_q16_current_l = tgt_l;
            } else if(vol_gain_q16_current_l > tgt_l) {
                vol_gain_q16_current_l -= VOL_RAMP_STEP_Q16;
                if(vol_gain_q16_current_l < tgt_l)
                    vol_gain_q16_current_l = tgt_l;
            }

            if(vol_gain_q16_current_r < tgt_r) {
                vol_gain_q16_current_r += VOL_RAMP_STEP_Q16;
                if(vol_gain_q16_current_r > tgt_r)
                    vol_gain_q16_current_r = tgt_r;
            } else if(vol_gain_q16_current_r > tgt_r) {
                vol_gain_q16_current_r -= VOL_RAMP_STEP_Q16;
                if(vol_gain_q16_current_r < tgt_r)
                    vol_gain_q16_current_r = tgt_r;
            }

            /* Combined gain (master x channel) recomputed once per block,
             * not per sample -- both operands only change here (at i==0),
             * so recomputing this inside the per-sample loop below was
             * pure wasted ISR time: a 64-bit multiply x2 on every one of
             * num_frames samples, every ~1ms callback, instead of once.
             * On Cortex-M7 a 64-bit multiply is several 32-bit UMULL/
             * UMLAL instructions, not a single-cycle op, so that was
             * measurable extra time spent inside the audio ISR every
             * block -- almost certainly the cause of the intermittent
             * audio stutter. */
            /* Mute is already folded into the ramp targets above, so the
             * current values ARE the muted values once the ramp settles.
             * The old `vol_muted ? 0 : ...` gate that lived here is what
             * bypassed the ramp; it is gone deliberately. */
            const int32_t m1 = vol_gain_q16_current;
            const int32_t l1 = vol_gain_q16_current_l;
            const int32_t r1 = vol_gain_q16_current_r;

            /* The gain glides per SAMPLE, not per block.
             *
             * Updating once per block and holding it constant for all 48
             * samples makes the gain a staircase: with VOL_RAMP_STEP_Q16
             * = 65536/10 that is ten jumps of 10% of full scale, one
             * every 1 ms. Ten percent is not a small step -- it is a
             * 1 kHz train of amplitude discontinuities, i.e. zipper
             * noise, on every volume change. Dragging a slider
             * continuously re-targets the ramp and keeps the staircase
             * running for the length of the drag, which is why fast
             * slider movement breaks the sound up where a slow move
             * mostly does not.
             *
             * Per-sample is not a slower ramp. Step SIZE and update RATE
             * are different things: interpolating the same per-block gain
             * change across that block's samples keeps the transition at
             * exactly the same ~10 ms and only removes the staircase.
             *
             * Hold the block's start and end gain and walk between them
             * one sample at a time; cost is one add per sample. The next
             * block restarts from vol_gain_q16_current, so the truncating
             * division below cannot accumulate drift. */
            int32_t end_l = (int32_t)(((int64_t)m1 * (int64_t)l1) >> 16);
            int32_t end_r = (int32_t)(((int64_t)m1 * (int64_t)r1) >> 16);
            end_l = (int32_t)(((int64_t)end_l * (int64_t)PB_HEADROOM_Q16) >> 16);
            end_r = (int32_t)(((int64_t)end_r * (int64_t)PB_HEADROOM_Q16) >> 16);

            if(!pb_gain_seeded) {
                /* Nothing to glide from on the first block after a prime:
                 * jump straight to the settled gain and let the prime
                 * fade above be the only ramp in play. */
                combined_l_q16  = end_l;
                combined_r_q16  = end_r;
                combined_l_step = 0;
                combined_r_step = 0;
                pb_gain_seeded  = 1U;
            } else {
                combined_l_q16  = pb_gain_l_q16;
                combined_r_q16  = pb_gain_r_q16;
                combined_l_step = (end_l - pb_gain_l_q16) / (int32_t)num_frames;
                combined_r_step = (end_r - pb_gain_r_q16) / (int32_t)num_frames;
            }

            /* Where the next block must start from. */
            pb_gain_l_q16 = end_l;
            pb_gain_r_q16 = end_r;
        }

        /* combined_l_q16 / combined_r_q16 already fold in master, channel,
         * and both mute flags -- computed once above at i==0, reused
         * unchanged for every sample in this block (ramping only updates
         * once per block anyway, so recomputing per-sample bought nothing
         * but ISR time). */
        int32_t out_l = apply_gain_q16_24bit(last_l, combined_l_q16);
        int32_t out_r = apply_gain_q16_24bit(last_r, combined_r_q16);

        /* Prime fade-in -- runs only for the first PB_FADE_FRAMES frames
         * after a 0->1 prime; once saturated this is one compare per
         * sample. Applied after the volume gain so the two stay
         * independent: a host volume change mid-ramp behaves normally. */
        if(pb_fade_q16 < (1U << 16)) {
            out_l = apply_gain_q16_24bit(out_l, (int32_t)pb_fade_q16);
            out_r = apply_gain_q16_24bit(out_r, (int32_t)pb_fade_q16);
            pb_fade_q16 += PB_FADE_STEP_Q16;
            if(pb_fade_q16 > (1U << 16)) pb_fade_q16 = (1U << 16);
        }

        dst[i * AUDIO_CHANNELS + 0] = out_l;
        dst[i * AUDIO_CHANNELS + 1] = out_r;

        /* Anchor for the underrun fade-out. Kept post-gain and post-fade
         * so the ramp starts from what the codec actually received, not
         * from the raw ring value. */
        pb_last_out_l = out_l;
        pb_last_out_r = out_r;

        /* Glide the gain toward this block's end value -- see the comment
         * at i == 0. Held flat when the ramp has settled (step == 0). */
        combined_l_q16 += combined_l_step;
        combined_r_q16 += combined_r_step;
    }
}
/* -----------------------------------------------------------------------
 * AudioIF_PushMIDI — MIDI path, called from DataOut ISR
 *
 * USB MIDI event format (4 bytes per event):
 *   byte 0: [Cable Number 7:4][Code Index Number 3:0]
 *   byte 1: MIDI status  (e.g. 0xBn = CC on channel n)
 *   byte 2: CC number    (0–127)
 *   byte 3: CC value     (0–127)
 *
 * Only CIN = 0x0B (Control Change) is processed. All other event types
 * (Note On/Off, Program Change, SysEx, etc.) are silently ignored. */
void AudioIF_PushMIDI(uint8_t *buf, uint32_t num_bytes)
{
    uint32_t num_events = num_bytes / MIDI_EVENT_SIZE;

    for(uint32_t i = 0; i < num_events; i++) {
        uint8_t *ev  = &buf[i * MIDI_EVENT_SIZE];
        uint8_t  cin = ev[0] & 0x0FU;

        if(cin == MIDI_CIN_CONTROL_CHANGE) {
            uint8_t cc_num = ev[2] & 0x7FU;
            uint8_t cc_val = ev[3] & 0x7FU;
            dsp_params[cc_num] = cc_val;
        }
    }
}

/* -----------------------------------------------------------------------
 * AudioIF_GetCCParam — read a stored CC value from AudioCallback (DMA ISR)
 *
 * Example:
 *   float thresh = -40.f + AudioIF_GetCCParam(1) / 127.f * 40.f; */
uint8_t AudioIF_GetCCParam(uint8_t cc_number)
{
    return dsp_params[cc_number & 0x7FU];
}
