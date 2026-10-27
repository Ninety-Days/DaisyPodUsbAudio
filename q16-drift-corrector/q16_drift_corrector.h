/* =============================================================================
 * Q16 fractional drift corrector — single-header reference implementation
 * =============================================================================
 *
 * MIT License. Copyright (c) 2026 NinetyDays. Full text in LICENSE alongside this
 * file; it is unscoped MIT, not the scoped licence of the parent repository.
 *
 * Extracted from DaisyPodUsbAudio (github.com/Ninety-Days/DaisyPodUsbAudio),
 * where it corrects clock drift on a USB Audio Class 1 full-duplex stream
 * on an STM32H750. Nothing in this file is specific to that: no USB, no
 * HAL, no vendor headers, no fixed channel count, no fixed sample rate.
 * It is standalone on purpose — it will outlive the USB class version and
 * the MCU it was built for, and it is small enough to read in one sitting.
 *
 * WHAT THIS SOLVES
 * -----------------------------------------------------------------------
 * Two clocks that are nominally the same rate never are: a host's audio
 * clock and a device's crystal always differ by some small offset,
 * typically tens of parts-per-million. Left alone, a ring buffer between
 * them either fills or drains without bound. The classic fix on an
 * ADAPTIVE isochronous endpoint with no separate feedback endpoint is to
 * vary the number of samples sent per packet — nominal-1 / nominal /
 * nominal+1 — chosen from ring fill level. That works, but every
 * nominal±1 packet is a discrete jump in the waveform: a dropped or
 * repeated sample, audible as a click if the offset is large enough or
 * frequent enough.
 *
 * This corrector removes the discontinuity by moving the correction from
 * the packet layer to the sample layer. The packet size is always exactly
 * nominal. The correction instead adjusts a FRACTIONAL read position
 * inside the ring, advancing by slightly more or less than 1.0 per output
 * sample, with the ring entries around that position interpolated. The
 * same total correction is applied either way — the
 * difference is that it is now spread across every sample by a fraction
 * of a sample, instead of being concentrated into one whole-sample jump.
 * A fraction of a sample of interpolation error is far below what's
 * audible; a dropped sample is not.
 *
 * WHICH INTERPOLATOR — READ THIS BEFORE PICKING ONE
 * -----------------------------------------------------------------------
 * Three readers are provided. They are NOT equivalent in audio quality,
 * and the difference is not the fixed tilt you might expect.
 *
 * The read phase does not sit still. It sweeps 0 -> 1 continuously at the
 * clock offset, so whatever the interpolator does to high frequencies
 * arrives as a slow AMPLITUDE MODULATION, not as a static roll-off. At
 * 30 ppm the sweep period is 0.69 s, i.e. about 1.4 Hz of AM.
 *
 * Worst-case output error against an ideal fractional delay:
 *
 *              linear   Catmull-Rom   polyphase sinc
 *    1 kHz     -53.4       -88.8         -101.2  dBFS
 *    5 kHz     -25.5       -45.9          -99.8
 *   10 kHz     -13.7       -24.5          -97.2
 *   15 kHz      -7.0       -12.0          -88.0
 *   20 kHz      -2.6        -4.4          -37.5
 *
 * As AM depth: linear puts roughly 2 dB of ~1.4 Hz modulation on
 * everything near 10 kHz and 5 dB at 15 kHz, which is audible on cymbals
 * and other HF-dense material. Catmull-Rom cuts that to 0.53 dB and
 * 2.53 dB. The sinc reader is flat to 0.000 dB through 18 kHz.
 *
 *   q16_read_sinc_i32()          recommended; needs q16_sinc_table.h
 *                                (16 KB const float32) and roughly
 *                                32 MACs per sample per channel
 *   q16_read_cubic_i32()         no table, 4 taps, good enough if the
 *                                material has little energy above ~8 kHz
 *   q16_read_interpolated_i32()  linear; the minimal illustration of the
 *                                idea, not a recommended default
 *
 * None is transparent near Nyquist, and no tap count fixes that: an ideal
 * fractional delay at w = pi is e^(-j*pi*d), purely imaginary at d = 0.5,
 * while any real FIR has a purely real response there, so |H| must be
 * zero. The 20 kHz figures above are that limit, not a design shortfall.
 *
 * ONE USAGE CONSTRAINT YOU MUST HONOUR
 * -----------------------------------------------------------------------
 * The linear reader touches TWO ring entries: the one at the current
 * integer position and the one after it. The cubic reader touches FOUR:
 * one before and two after. The sinc reader touches THIRTY-TWO: fifteen
 * before and sixteen after. Reading past the write position blends stale
 * or zero data into the output.
 *
 * In practice this means: keep headroom beyond what you read, and how much
 * depends on the reader. If `q16_update_tilt()` returns `available`, read
 * at most `available - 1` frames with the linear reader, `available - 2`
 * with the cubic, and `available - 16` with the sinc — and gate on that in
 * your own pre-fill / underrun logic. The reference firmware does this
 * implicitly by holding the ring near half full and treating anything near
 * empty as an underrun to be reset, so the boundary is never reached in
 * normal operation — but it IS reachable if you drive the ring closer to
 * empty than that, and nothing in this file stops you.
 *
 * WHAT THIS DOES NOT SOLVE
 * -----------------------------------------------------------------------
 * This is a rate corrector, not a clock source and not a resampler in the
 * general sense: it assumes the two clocks are close (parts-per-million,
 * not percent) and that "ring fill level relative to a target" is a
 * meaningful, regularly-updated error signal, which is what a USB audio
 * transfer schedule naturally provides. Do not reach for this if the rate
 * mismatch is large, irregular, or unmeasured.
 *
 * WHY IT IS CORRECT, IN THREE LINES
 * -----------------------------------------------------------------------
 * Every callback (once per packet, once per audio block — whatever your
 * transport's natural cadence is):
 *
 *     error    = available_frames - target
 *     tilt     = clamp(error / RATE_DIV, +-RATE_MAX_Q16)
 *     step_q16 = FRAC_ONE + tilt
 *
 * Then the ring is read at a position that advances by step_q16 (Q16
 * fixed point, FRAC_ONE = 1.0) per output frame, interpolating between
 * the two ring entries either side of that fractional position. That is
 * the entire algorithm. Everything else in this file is bookkeeping
 * (ring indices, overflow-safe fixed-point math) and configuration.
 *
 * CONTROL LOOP CHARACTER
 * -----------------------------------------------------------------------
 * This is a plain proportional controller: no integrator, no derivative
 * term. Consequence, worth knowing before you tune it: each ring settles
 * at a small STANDING offset proportional to the clock mismatch, rather
 * than hunting back to the exact target. That standing offset consumes
 * ring headroom — see q16_authority_used_percent_x10() below — so if a
 * real clock error is large, it shows up here, as consumed headroom, well
 * before it shows up as an underrun or overrun. Widening RATE_MAX_Q16 to
 * chase a large offset is treating the symptom; the standing offset is
 * telling you to fix the clock.
 *
 * A second, less obvious consequence: `tilt` is an INTEGER number of Q16
 * units (one unit = 1e6/2^FRAC_BITS ppm, ~15.259 ppm at the default
 * FRAC_BITS=16), so the set of read rates the corrector can actually hold
 * is a discrete ladder — 0, 15.259, 30.518, 45.776, 61.035 ppm, and so on
 * — not a continuum. A true offset that falls BETWEEN two rungs (which is
 * the common case — real crystal offsets are not conveniently multiples
 * of 15.259 ppm) cannot be matched exactly: the controller settles on the
 * rung at or below the true rate, and holds a standing offset large
 * enough to make that rung's average throughput match the producer's
 * real rate. This settled offset can run noticeably higher than the
 * naive estimate of `error / RATE_DIV` alone would suggest, because the
 * integer floor in that division also creates a dead zone within each
 * rung. Simulated at a 58.5 ppm true offset with the reference constants
 * (RING_SIZE=512, TARGET=256, RATE_DIV=8, RATE_MAX_Q16=32): the
 * controller settled on the 45.776 ppm rung (three Q16 units, the nearest
 * rung below 58.5) with a standing offset of 31 frames — not the 24
 * frames a naive `tilt * RATE_DIV` estimate predicts, and not exactly
 * 58.5 ppm. This is expected, bounded, and still correction with no
 * discontinuity — see test_q16.c for the reproduction and the actual
 * invariants this file is checked against (output smoothness and ring
 * stability, not exact ppm tracking).
 *
 * TUNING
 * -----------------------------------------------------------------------
 *   RING_SIZE     Ring capacity in frames. A power of two is RECOMMENDED
 *                 but not required: every wrap in this file goes through
 *                 `%`, which a compiler reduces to a mask only when the
 *                 divisor is a constant power of two. Verified by direct
 *                 simulation — a 500-frame ring behaves identically, so
 *                 this is a performance choice, not a correctness one.
 *                 The one HARD constraint is that RING_SIZE << FRAC_BITS
 *                 must fit in uint32_t (checked in q16_init()). Bigger
 *                 gives more headroom against transport jitter at the cost
 *                 of latency; the reference firmware uses 512 frames
 *                 (10.67 ms @ 48 kHz) for a target of 256 (half the ring).
 *   TARGET        Fill level the controller holds the ring at. Usually
 *                 RING_SIZE / 2.
 *   RATE_DIV      Loop stiffness. Smaller = stiffer (corrects faster,
 *                 tracks fill-level noise more). The reference value is 8.
 *   RATE_MAX_Q16  Correction authority, in Q16 units. One unit is
 *                 1e6 / 65536 ~= 15.259 ppm. The reference value is 32,
 *                 i.e. +-488 ppm of authority — far more than any real
 *                 crystal pairing needs (typical offsets are tens of ppm),
 *                 and, at maximum, still under a hundredth of a semitone:
 *                 inaudible as a pitch shift. Compare to one skipped
 *                 sample, which is audible as a click regardless of how
 *                 rarely it happens.
 *
 * FIXED-POINT NOTE
 * -----------------------------------------------------------------------
 * The interpolation multiply needs a 64-bit intermediate: a sample
 * difference can span the full sample range and the fractional part is
 * 2^16, so the product can exceed 32 bits before the final shift. On any
 * core with a fast 32x32->64 multiply (e.g. Cortex-M7's SMULL) this is a
 * single instruction, which is what makes doing it per sample per channel
 * affordable at audio rates. On a core without one, budget for a
 * multi-instruction 64-bit multiply and profile before assuming this is
 * free.
 *
 * WHY NOT A BANG-BANG / SOF-DERIVED CONTROLLER
 * -----------------------------------------------------------------------
 * A natural-seeming alternative is to derive the rate ratio from the
 * host's Start-of-Frame timing instead of from ring fill level. Don't:
 * the SOF-derived ratio and the fill-level error are the same information
 * algebraically (both are proxies for the same clock-offset integral),
 * and replacing the closed-loop fill-level feedback with an open-loop
 * feedforward estimate removes the thing that bounds position drift. If
 * you want an SOF-based measurement, use it as a diagnostic cross-check
 * alongside this controller, not as a replacement for it.
 *
 * PROVENANCE
 * -----------------------------------------------------------------------
 * This file is original work extracted from a larger project. It has no
 * dependency on that project's transport, USB class version, MCU, or
 * build system. License: see LICENSE in this directory (MIT).
 * =============================================================================
 */

#ifndef Q16_DRIFT_CORRECTOR_H
#define Q16_DRIFT_CORRECTOR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------------------------------------------------------------
 * Configuration. Override any of these with a -D before including this
 * header, or just edit them — this is a reference implementation meant
 * to be copied into a project and adapted, not linked as a library.
 * ------------------------------------------------------------------- */

#ifndef Q16_RING_SIZE
#define Q16_RING_SIZE      512U     /* frames; power of two recommended, not required */
#endif
#ifndef Q16_TARGET
#define Q16_TARGET         (Q16_RING_SIZE / 2U)
#endif
#ifndef Q16_RATE_DIV
#define Q16_RATE_DIV       8U       /* error frames -> Q16 rate tilt; smaller = stiffer */
#endif
#ifndef Q16_RATE_MAX
#define Q16_RATE_MAX       32U      /* +-authority in Q16 units; 32 ~= +-488 ppm */
#endif
#ifndef Q16_FRAC_BITS
#define Q16_FRAC_BITS      16U
#endif

#define Q16_FRAC_ONE       (1U << Q16_FRAC_BITS)

/* One Q16 unit in parts-per-million, for reporting/diagnostics:
 *   ppm_per_unit = 1e6 / 2^Q16_FRAC_BITS
 * At the default FRAC_BITS=16 this is ~15.2588 ppm/unit. Kept as a
 * compile-time constant expression rather than a float so this header
 * has no FPU dependency; convert to a human ppm figure at the call site
 * with a float or with fixed-point scaling as your project prefers. */
#define Q16_PPM_PER_UNIT_NUM 1000000U
#define Q16_PPM_PER_UNIT_DEN Q16_FRAC_ONE

/* ---------------------------------------------------------------------
 * State. One instance per corrected stream (e.g. one for record, one
 * for playback, if your transport is full-duplex like the reference
 * firmware's). Storage for the ring itself is NOT owned by this struct
 * — pass your own buffer to q16_read_interpolated_i32() so this file
 * stays agnostic to
 * channel count, sample format, and where the buffer lives (SRAM,
 * SDRAM, DTCM, ...).
 * ------------------------------------------------------------------- */

typedef struct {
    uint32_t read_q16;   /* fractional read position, Q16 frames */
    uint32_t step_q16;   /* current advance per output frame, Q16 */
    uint32_t write;      /* integer write index into the caller's ring */
    uint32_t read;       /* integer read index, kept in step with read_q16 */
    int      active;     /* 0 = pre-fill (waiting to reach target), 1 = steady state */
} q16_corrector_t;

/* q16_init — zero a corrector to its startup state.
 * Call once, and again after any full ring reset (e.g. after an
 * underrun you choose to handle by re-priming from empty). */
static inline void q16_init(q16_corrector_t *c)
{
    /* The one hard constraint: the fractional position space
     * (RING_SIZE << FRAC_BITS) must fit in uint32_t, or the wrap in
     * q16_read_interpolated_i32() is wrong. This is a configuration
     * error, not a runtime condition -- reduce Q16_RING_SIZE or
     * Q16_FRAC_BITS.
     *
     * Deliberately NOT trapped here with an infinite loop: an earlier
     * version of this file did that, and also trapped on RING_SIZE not
     * being a power of two -- which would have hung a perfectly valid
     * configuration, since power-of-two is only a performance
     * preference (see TUNING above). Fail at compile time instead if
     * your toolchain has C11; otherwise this is documented and left to
     * the integrator. */
#if defined(__STDC_VERSION__) && __STDC_VERSION__ >= 201112L
    _Static_assert(((uint64_t)Q16_RING_SIZE << Q16_FRAC_BITS) <= 0xFFFFFFFFULL,
                   "Q16_RING_SIZE << Q16_FRAC_BITS must fit in uint32_t");
#endif

    c->read_q16 = 0U;
    c->step_q16 = Q16_FRAC_ONE;
    c->write    = 0U;
    c->read     = 0U;
    c->active   = 0;
}

/* q16_frames_available — how many unread frames are in the ring right
 * now. Wrap-safe for the single-writer/single-reader case. */
static inline uint32_t q16_frames_available(const q16_corrector_t *c)
{
    return (c->write - c->read + Q16_RING_SIZE) % Q16_RING_SIZE;
}

/* q16_advance_write — call once per frame written into the caller's
 * ring (e.g. once per incoming sample-frame from the producer side of
 * your transport). Kept separate from the read path because production
 * and consumption typically happen in different callbacks/ISRs. */
static inline void q16_advance_write(q16_corrector_t *c)
{
    c->write = (c->write + 1U) % Q16_RING_SIZE;
}

/* q16_update_tilt — the controller. Call once per output callback
 * (once per USB packet, once per audio block -- whatever cadence your
 * transport naturally provides), BEFORE reading frames for that
 * callback. Returns the number of frames currently available, so the
 * caller can also drive pre-fill / underrun logic from the same value
 * without a second ring-arithmetic call.
 *
 * This function does not handle underrun/overrun itself -- ring
 * management (pre-fill gating, resetting after starvation, clamping a
 * read that would run past the writer) is transport-specific and stays
 * in the caller. The corrector's only job is the rate tilt. */
static inline uint32_t q16_update_tilt(q16_corrector_t *c)
{
    uint32_t available = q16_frames_available(c);

    int32_t error = (int32_t)available - (int32_t)Q16_TARGET;
    int32_t tilt  = error / (int32_t)Q16_RATE_DIV;
    if (tilt >  (int32_t)Q16_RATE_MAX) tilt =  (int32_t)Q16_RATE_MAX;
    if (tilt < -(int32_t)Q16_RATE_MAX) tilt = -(int32_t)Q16_RATE_MAX;

    /* error > 0 means the ring is filling: read slightly faster. */
    c->step_q16 = (uint32_t)((int32_t)Q16_FRAC_ONE + tilt);

    return available;
}

/* q16_read_interpolated_i32 — read one interpolated frame from a
 * caller-owned ring of interleaved int32_t samples, and advance the
 * fractional read position by step_q16.
 *
 *   ring          pointer to the ring buffer, interleaved samples,
 *                 Q16_RING_SIZE frames long
 *   channels      samples per frame (e.g. 2 for stereo)
 *   out           destination, `channels` int32_t samples
 *
 * Uses a 64-bit intermediate for the interpolation multiply -- see the
 * FIXED-POINT NOTE in the file header for why. Adapt sample type/width
 * as needed; the shape of the computation does not change for 16-bit,
 * 24-bit-in-32, or float samples, only the intermediate type and the
 * final store. */
static inline void q16_read_interpolated_i32(q16_corrector_t *c,
                                              const int32_t *ring,
                                              uint32_t channels,
                                              int32_t *out)
{
    uint32_t idx  = (c->read_q16 >> Q16_FRAC_BITS) % Q16_RING_SIZE;
    uint32_t nxt  = (idx + 1U) % Q16_RING_SIZE;
    uint32_t frac = c->read_q16 & (Q16_FRAC_ONE - 1U);

    const int32_t *A = &ring[idx * channels];
    const int32_t *B = &ring[nxt * channels];

    for (uint32_t ch = 0; ch < channels; ch++) {
        out[ch] = A[ch] + (int32_t)(((int64_t)(B[ch] - A[ch]) * frac) >> Q16_FRAC_BITS);
    }

    c->read_q16 = (c->read_q16 + c->step_q16) % ((uint32_t)Q16_RING_SIZE << Q16_FRAC_BITS);
    /* Keep the integer read index in step with the fractional position,
     * for callers that also need it directly (fill-level math, a status
     * display, diagnostics). */
    c->read = (c->read_q16 >> Q16_FRAC_BITS) % Q16_RING_SIZE;
}

/* q16_read_cubic_i32 — table-free middle option. Same contract as
 * q16_read_interpolated_i32() above, but Catmull-Rom (cubic Hermite)
 * across four ring entries instead of linear across two. See "WHICH
 * INTERPOLATOR" in the file header for the response table and for why
 * the difference shows up as amplitude modulation rather than as a fixed
 * roll-off.
 *
 * Touches idx-1 through idx+2, so keep two frames of headroom ahead of
 * the read position, not one.
 *
 * Computed in float rather than fixed point on purpose: the cubic's
 * coefficients exceed the input range (2.5x, 1.5x), which in fixed point
 * needs headroom analysis for no benefit, and any core fast enough to
 * run this at audio rates has an FPU. Measured worst-case arithmetic
 * error against a double-precision reference is about 7 LSB of a 24-bit
 * sample (~ -122 dBFS) -- far below the interpolation error itself,
 * which is the thing that actually limits quality here (~ -89 dBFS at
 * 1 kHz, ~ -26 dBFS at 10 kHz). Do not quote the arithmetic figure as if
 * it described the interpolator's transparency; it does not.
 *
 * The clamp is not decorative. A cubic overshoots on material that is
 * already hard-clipped at full scale, by up to about 1 dB in simulation;
 * on band-limited material at 0 dBFS it never fires. It costs one
 * comparison per sample. Keep it.
 *
 * SAMPLE_MIN/MAX default to a 24-bit range because that is what the
 * reference firmware carries. Redefine for your sample width. */
#ifndef Q16_SAMPLE_MAX
#define Q16_SAMPLE_MAX  8388607.0f
#endif
#ifndef Q16_SAMPLE_MIN
#define Q16_SAMPLE_MIN (-8388608.0f)
#endif

static inline void q16_read_cubic_i32(q16_corrector_t *c,
                                      const int32_t *ring,
                                      uint32_t channels,
                                      int32_t *out)
{
    uint32_t idx  = (c->read_q16 >> Q16_FRAC_BITS) % Q16_RING_SIZE;
    uint32_t nxt  = (idx + 1U) % Q16_RING_SIZE;
    uint32_t prv  = (idx + Q16_RING_SIZE - 1U) % Q16_RING_SIZE;
    uint32_t nx2  = (idx + 2U) % Q16_RING_SIZE;
    uint32_t frac = c->read_q16 & (Q16_FRAC_ONE - 1U);
    const float d = (float)frac * (1.0f / (float)Q16_FRAC_ONE);

    const int32_t *P = &ring[prv * channels];
    const int32_t *A = &ring[idx * channels];
    const int32_t *B = &ring[nxt * channels];
    const int32_t *C = &ring[nx2 * channels];

    for (uint32_t ch = 0; ch < channels; ch++) {
        const float a = (float)P[ch], b = (float)A[ch];
        const float e = (float)B[ch], f = (float)C[ch];
        const float c1 = 0.5f * (e - a);
        const float c2 = a - 2.5f * b + 2.0f * e - 0.5f * f;
        const float c3 = 0.5f * (f - a) + 1.5f * (b - e);
        float y = ((c3 * d + c2) * d + c1) * d + b;
        if (y > Q16_SAMPLE_MAX) y = Q16_SAMPLE_MAX;
        if (y < Q16_SAMPLE_MIN) y = Q16_SAMPLE_MIN;
        out[ch] = (int32_t)y;
    }

    c->read_q16 = (c->read_q16 + c->step_q16) % ((uint32_t)Q16_RING_SIZE << Q16_FRAC_BITS);
    c->read = (c->read_q16 >> Q16_FRAC_BITS) % Q16_RING_SIZE;
}

/* q16_read_sinc_i32 — HIGHEST QUALITY reader. Same contract as the two
 * above. Polyphase windowed-sinc fractional delay: 32 taps per phase,
 * 128 phase rows plus a guard row, with linear blending between adjacent
 * rows. Requires q16_sinc_table.h (16 KB of const float32); include it
 * before this header's consumer, or define Q16_NO_SINC to drop this
 * function if you do not want the table.
 *
 * Worst-case output error against an ideal fractional delay, measured on
 * the shipped float32 coefficients:
 *
 *              linear   Catmull-Rom   polyphase sinc
 *    1 kHz     -53.4       -88.9         -101.2  dBFS
 *    5 kHz     -25.5       -45.9          -99.8
 *   10 kHz     -13.8       -24.5          -97.2
 *   15 kHz      -7.2       -12.1          -88.0
 *   20 kHz      -2.9        -4.4          -37.5
 *
 * Expressed as AM depth -- the thing you actually hear, since the phase
 * sweeps -- the sinc reader is flat to 0.000 dB through 18 kHz and
 * 0.002 dB at 20 kHz, against 0.53 dB at 10 kHz and 2.53 dB at 15 kHz
 * for Catmull-Rom.
 *
 * THE ROW BLEND IS NOT OPTIONAL. Quantising the phase to 128 rows and
 * snapping to the nearest is worse than Catmull-Rom below ~3 kHz
 * (-59.8 dBFS at 1 kHz), because a stepped delay reintroduces exactly
 * the discontinuity this whole corrector exists to avoid -- just 60 dB
 * quieter. Blending between rows removes that floor. If you shrink the
 * table, shrink P and keep the blend, not the other way round.
 *
 * The top of the band cannot be improved by adding taps. An ideal
 * fractional delay at Nyquist is e^(-j*pi*d), purely imaginary at
 * d = 0.5, while any real FIR has a purely real response at w = pi, so
 * |H| there is necessarily zero. The 20 kHz figure is that limit.
 *
 * Row 0 is not a delta: it is a lowpass with its corner at 0.97 x
 * Nyquist, so a fixed -0.12 dB sits at 20 kHz on every sample regardless
 * of phase. Static tilt at the very top of the band, not modulation.
 *
 * Cost: the coefficients depend only on the phase, so the row blend is
 * done once per frame and reused across channels -- 32 blends plus
 * 32 MACs per channel. The row index advances by at most 1/2048 per
 * sample at full tilt authority, so the same two rows are reused for 16+
 * consecutive samples and the table stays hot in cache.
 *
 * Touches idx-15 through idx+16. Keep sixteen frames of headroom either
 * side of the read position.
 *
 * Requires a power-of-two Q16_RING_SIZE (this function masks rather than
 * dividing) and Q16_FRAC_BITS == 16. */
#if !defined(Q16_NO_SINC) && defined(Q16_SINC_TABLE_H)

#if ((Q16_RING_SIZE) & ((Q16_RING_SIZE) - 1U)) != 0U
#error "q16_read_sinc_i32 needs a power-of-two Q16_RING_SIZE"
#endif
#if Q16_FRAC_BITS != 16
#error "q16_read_sinc_i32 assumes a Q16 fractional read position"
#endif

#define Q16_FD_SUB_BITS  (16 - Q16_FD_PHASE_BITS)
#define Q16_FD_SUB_ONE   (1U << Q16_FD_SUB_BITS)

static inline void q16_read_sinc_i32(q16_corrector_t *c,
                                     const int32_t *ring,
                                     uint32_t channels,
                                     int32_t *out)
{
    const uint32_t mask = (uint32_t)Q16_RING_SIZE - 1U;
    const uint32_t idx  = (c->read_q16 >> Q16_FRAC_BITS) & mask;
    const uint32_t frac = c->read_q16 & (Q16_FRAC_ONE - 1U);

    const uint32_t p   = frac >> Q16_FD_SUB_BITS;
    const uint32_t sub = frac & (Q16_FD_SUB_ONE - 1U);
    const float    fr  = (float)sub * (1.0f / (float)Q16_FD_SUB_ONE);
    const float   *a   = &q16_sinc_table[p * Q16_FD_TAPS];
    const float   *b   = &q16_sinc_table[(p + 1U) * Q16_FD_TAPS];

    float co[Q16_FD_TAPS];
    for (uint32_t k = 0; k < Q16_FD_TAPS; k++)
        co[k] = a[k] + fr * (b[k] - a[k]);

    const uint32_t base = (idx + (mask + 1U) - Q16_FD_CENTER) & mask;
    for (uint32_t ch = 0; ch < channels; ch++) {
        float acc = 0.0f;
        for (uint32_t k = 0; k < Q16_FD_TAPS; k++)
            acc += co[k] * (float)ring[(((base + k) & mask) * channels) + ch];
        if (acc > Q16_SAMPLE_MAX) acc = Q16_SAMPLE_MAX;
        if (acc < Q16_SAMPLE_MIN) acc = Q16_SAMPLE_MIN;
        out[ch] = (int32_t)acc;
    }

    c->read_q16 = (c->read_q16 + c->step_q16) % ((uint32_t)Q16_RING_SIZE << Q16_FRAC_BITS);
    c->read = (c->read_q16 >> Q16_FRAC_BITS) % Q16_RING_SIZE;
}

#endif /* sinc reader */

/* q16_tilt_ppm_x1000 — current tilt as parts-per-million, scaled by
 * 1000 to stay integer (e.g. a return of 58234 means 58.234 ppm).
 * Positive = ring filling, reader speeding up; negative = ring
 * draining, reader slowing down. Useful for logging/diagnostics without
 * pulling in a float formatter. */
static inline int32_t q16_tilt_ppm_x1000(const q16_corrector_t *c)
{
    int32_t tilt = (int32_t)c->step_q16 - (int32_t)Q16_FRAC_ONE;
    /* tilt is at most +-Q16_RATE_MAX (32 by default), so this product
     * comfortably fits in int64_t with headroom to spare. */
    return (int32_t)(((int64_t)tilt * Q16_PPM_PER_UNIT_NUM * 1000)
                      / Q16_PPM_PER_UNIT_DEN);
}

/* q16_authority_used_percent_x10 — how much of the configured
 * correction authority is currently in use, as a percentage scaled by
 * 10 (e.g. 123 means 12.3%). If this sits near 1000 (100%) in normal
 * operation, RATE_MAX_Q16 is undersized for the real clock offset, or
 * something upstream is producing a much larger drift than a crystal
 * mismatch would -- see the STANDING OFFSET note in the file header. */
static inline int32_t q16_authority_used_percent_x10(const q16_corrector_t *c)
{
    int32_t tilt = (int32_t)c->step_q16 - (int32_t)Q16_FRAC_ONE;
    if (tilt < 0) tilt = -tilt;
    return (int32_t)(((int64_t)tilt * 1000) / (int32_t)Q16_RATE_MAX);
}

#ifdef __cplusplus
}
#endif

#endif /* Q16_DRIFT_CORRECTOR_H */
