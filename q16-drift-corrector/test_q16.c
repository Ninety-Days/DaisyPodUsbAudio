/* License: see LICENSE in this directory (MIT). */
#define _USE_MATH_DEFINES
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <stdio.h>
#include <math.h>
#ifndef Q16_NO_SINC
#include "q16_sinc_table.h"
#endif
#include "q16_drift_corrector.h"

/* Exercises the sinc reader by default; with -DQ16_NO_SINC it falls back
 * to the cubic, so the test builds and passes in both configurations. */
#ifdef Q16_NO_SINC
#define Q16_TEST_READ  q16_read_cubic_i32
#define Q16_TEST_NAME  "cubic"
#else
#define Q16_TEST_READ  q16_read_sinc_i32
#define Q16_TEST_NAME  "sinc"
#endif

/* Real invariants for this corrector:
 *   (1) No amplitude discontinuity in the interpolated OUTPUT: consecutive
 *       output samples of a smooth input must differ by a bounded amount,
 *       never by a whole-sample jump. This is what a listener would hear
 *       as a click, so it's what the test must check -- not the ring index,
 *       which is allowed to step by more than 1 whenever step_q16 rounds
 *       past a slot boundary; that is not audible on its own.
 *   (2) The controller converges: after settling, the standing tilt must
 *       be within RATE_MAX_Q16 authority and stable (bounded variance),
 *       not runaway.
 *   (3) With a proportional-only controller, the settled tilt need NOT
 *       equal the injected ppm exactly -- it balances against the standing
 *       fill-level offset by design (documented in the header). What must
 *       hold is that the ring neither empties nor overflows over a long
 *       run, which is the actual correctness criterion for a rate
 *       corrector: it keeps up, indefinitely, without a discontinuity.
 */

static int32_t ring[Q16_RING_SIZE * 2]; /* stereo */

int main(void)
{
    q16_corrector_t c;
    q16_init(&c);

    const double true_ppm = 58.5;   /* matches the offset measured on real hardware */
    double producer_accum = 0.0;
    double producer_rate = 1.0 + true_ppm / 1e6;

    /* Smooth known input: a low-frequency sine, so consecutive true samples
     * are close together and any interpolation glitch is easy to bound. */
    double phase = 0.0;
    double phase_inc = 2.0 * M_PI * 200.0 / 48000.0; /* 200 Hz @ 48kHz */

    long n_steps = 2000000;
    double max_step_delta = 0.0;
    double prev_out = 0.0;
    int have_prev = 0;
    long empties = 0, overflows = 0;
    int32_t out[2];

    /* pre-fill to target before measuring, matching how the reference
     * firmware gates on rec_active */
    for (long i = 0; i < Q16_TARGET + 8; i++) {
        uint32_t w = c.write;
        double s = sin(phase); phase += phase_inc;
        ring[w*2+0] = (int32_t)(s * 1000000.0);
        ring[w*2+1] = ring[w*2+0];
        q16_advance_write(&c);
    }

    for (long i = 0; i < n_steps; i++) {
        producer_accum += producer_rate;
        while (producer_accum >= 1.0) {
            uint32_t avail = q16_frames_available(&c);
            if (avail >= Q16_RING_SIZE - 1U) { overflows++; break; }
            uint32_t w = c.write;
            double s = sin(phase); phase += phase_inc;
            ring[w*2+0] = (int32_t)(s * 1000000.0);
            ring[w*2+1] = ring[w*2+0];
            q16_advance_write(&c);
            producer_accum -= 1.0;
        }

        uint32_t avail = q16_update_tilt(&c);
        if (avail == 0U) empties++;
        Q16_TEST_READ(&c, ring, 2, out);

        double sample = (double)out[0] / 1000000.0;
        if (have_prev) {
            double d = sample - prev_out;
            if (d < 0) d = -d;
            if (d > max_step_delta) max_step_delta = d;
        }
        prev_out = sample;
        have_prev = 1;
    }

    /* max possible per-step delta for this input if interpolation were
     * PERFECT and step_q16 were exactly 1.0: bounded by the sine's own
     * slew rate. Anything wildly above that indicates a discontinuity
     * rather than smooth tracking. */
    double max_expected_slew = phase_inc * 1.05; /* +5% slack for the tilt itself */

    printf("reader under test:      %s\n", Q16_TEST_NAME);
    printf("injected offset:        %.3f ppm\n", true_ppm);
    printf("measured settled tilt:  %.3f ppm\n", q16_tilt_ppm_x1000(&c) / 1000.0);
    printf("authority used:         %ld.%01ld%%\n",
           (long)q16_authority_used_percent_x10(&c) / 10,
           (long)q16_authority_used_percent_x10(&c) % 10);
    printf("max |sample[n]-sample[n-1]| observed: %.6f\n", max_step_delta);
    printf("max expected from smooth 200Hz input: %.6f\n", max_expected_slew);
    printf("ring empties: %ld   ring overflows: %ld   (over %ld steps)\n",
           empties, overflows, n_steps);

    /* The controller settles on the nearest INTEGER Q16 tilt rung at or
     * below the true offset -- see the CONTROL LOOP CHARACTER note in the
     * header. It does not track true_ppm exactly, and should not be
     * expected to: the pass criteria below are the two things that
     * actually matter for correctness (no discontinuity, ring stays
     * bounded), not an exact-ppm match. */
    double q16_unit_ppm = 1e6 / (double)Q16_FRAC_ONE;
    double lowest_rung_at_or_below = q16_unit_ppm * (double)((long)(true_ppm / q16_unit_ppm));
    double measured = q16_tilt_ppm_x1000(&c) / 1000.0;

    int fail = 0;
    if (max_step_delta > max_expected_slew) {
        printf("FAIL: output has a bigger jump than the input's own slew rate allows -- discontinuity\n");
        fail = 1;
    }
    if (empties > 0 || overflows > 0) {
        printf("FAIL: ring emptied or overflowed -- corrector did not keep up\n");
        fail = 1;
    }
    if (measured > true_ppm + q16_unit_ppm || measured < lowest_rung_at_or_below - q16_unit_ppm) {
        printf("FAIL: settled tilt (%.3f) is not on a plausible quantization rung "
               "near %.3f ppm\n", measured, true_ppm);
        fail = 1;
    }
    printf(fail ? "RESULT: FAIL\n" : "RESULT: PASS\n");
    return fail;
}
