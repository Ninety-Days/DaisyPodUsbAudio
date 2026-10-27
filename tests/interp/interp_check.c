/* MIT License (see LICENSE at repository root) applies to this file. */
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "fd_sinc_table.h"

#define FD_SUB_BITS  (16 - FD_PHASE_BITS)
#define FD_SUB_ONE   (1U << FD_SUB_BITS)

static inline int32_t fd_clamp24(float y){
    if(y >  8388607.0f) return  8388607;
    if(y < -8388608.0f) return -8388608;
    return (int32_t)y;
}
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

#define RS 512U
#ifndef NPH
#define NPH 8          /* signal phases swept -- see the note in main() */
#endif
static int32_t ring[RS*2];

int main(void){
    const double fs=48000.0, A=4000000.0;
    /* Reported relative to the tone, not to full scale -- the tone here sits
     * 6.4 dB below 24-bit FS, and worst-case error scales with amplitude, so
     * dBc is the amplitude-independent figure. This printed "dBFS" before,
     * which was the wrong label for what is computed.
     *
     * NPH: the signal phase MUST be swept. For a linear filter the error is
     * Re{A * (H(w) - e^(jwd)) * e^(jphi)} -- at a single phase you measure one
     * projection of a complex error rather than its magnitude, and read near
     * zero wherever that error happens to be mostly imaginary. This loop
     * previously ran at phi = 0 only (the ring was rebuilt relative to idx, so
     * varying idx moved the read position without changing the phase presented
     * to the filter) and reported up to 4.6 dB better than the truth. Eight
     * phases converge to within 0.1 dB of a 64-phase sweep, of gen_table.py,
     * and of the closed-form max|H(w,d) - e^(-jwd)|.
     *
     * The idx sweep is kept because it does test something real -- that the
     * indexing arithmetic is position-independent -- just not what the phase
     * sweep tests. Shortened to 8 positions to pay for the phase loop. */
    /* Bounds for the error sweep, one per tone, taken from the first table in
     * docs/INTERPOLATION.md with 3 dB of slack. The slack is for compiler and
     * libm differences, not for regressions: the measured figures sit well
     * inside it, and anything that drifts past these has changed the reader's
     * numerical behaviour rather than merely rounded differently.
     *
     * Without these the test printed numbers and always exited 0, so CI went
     * green on any regression short of a compile error -- while tests.yml
     * claimed the documented numbers were checked on every push rather than
     * taken on trust. They are now. */
#if FD_USE_SINC
    const double bound_dbc[5] = { -98.2, -96.8, -94.2, -85.0, -34.5 };
#else
    const double bound_dbc[5] = { -85.8, -42.9, -21.5,  -9.0,  -1.4 };
#endif
    int fail = 0;

    printf("== worst-case output error vs ideal fractional delay ==\n");
    double fr[]={1000,5000,10000,15000,20000};
    for(int fi=0;fi<5;fi++){
        double f=fr[fi], w=2*M_PI*f/fs, worst=0;
        for(int di=0; di<1000; di++){
            uint32_t frac = (uint32_t)((double)di/1000.0*65536.0);
            for(int pi=0; pi<NPH; pi++){
                const double ph = 2.0*M_PI*(double)pi/(double)NPH;
                for(int t=0;t<8;t++){
                    uint32_t idx = 64u + (uint32_t)t;
                    for(uint32_t j=0;j<RS;j++){
                        double s=A*sin(ph + w*((double)j-(double)idx));
                        ring[j*2+0]=(int32_t)llround(s);
                        ring[j*2+1]=(int32_t)llround(-s);
                    }
                    int32_t o[2];
                    fd_read_frame(ring, RS-1u, 2u, idx, frac, o);
                    double d=(double)frac/65536.0;
                    double ideal=A*sin(ph + w*d);
                    double e0=fabs((double)o[0]-ideal)/A;
                    double e1=fabs((double)o[1]+ideal)/A;
                    if(e0>worst) worst=e0;
                    if(e1>worst) worst=e1;
                }
            }
        }
        double dbc = 20*log10(worst+1e-15);
        int over = dbc > bound_dbc[fi];
        if(over) fail = 1;
        printf("  %6.0f Hz : %7.1f dBc  (bound %6.1f)  %s\n",
               f, dbc, bound_dbc[fi], over ? "FAIL" : "OK");
    }
    printf("\n== channel independence (L must not leak into R) ==\n");
    for(uint32_t j=0;j<RS;j++){ ring[j*2+0]=(int32_t)(8000000.0*sin(0.3*j)); ring[j*2+1]=0; }
    int32_t o[2]; int bad=0;
    for(uint32_t fq=0; fq<65536u; fq+=997u){
        fd_read_frame(ring, RS-1u, 2u, 100u, fq, o);
        if(o[1]!=0) bad++;
    }
    if(bad) fail = 1;
    printf("  R non-zero on %d/66 probes  %s\n", bad, bad? "FAIL":"OK");

    printf("\n== ring wrap: reading across index 0 must match an unwrapped copy ==\n");
    static int32_t r2[RS*2];
    for(uint32_t j=0;j<RS;j++){ double s=6000000.0*sin(0.11*j)+2000000.0*sin(0.9*j);
        ring[j*2]=ring[j*2+1]=(int32_t)llround(s); }
    memcpy(r2,ring,sizeof(ring));
    int mism=0; long worstd=0;
    for(uint32_t idx=0; idx<RS; idx++){
        for(uint32_t fq=0; fq<65536u; fq+=8191u){
            int32_t a1[2],b1[2];
            fd_read_frame(ring, RS-1u, 2u, idx, fq, a1);
            /* same read, ring rotated by 256 -> identical result expected */
            static int32_t rot[RS*2];
            for(uint32_t j=0;j<RS;j++){ uint32_t src=(j+256u)&(RS-1u);
                rot[j*2]=r2[src*2]; rot[j*2+1]=r2[src*2+1]; }
            fd_read_frame(rot, RS-1u, 2u, (idx+RS-256u)&(RS-1u), fq, b1);
            long d0=labs((long)a1[0]-(long)b1[0]);
            if(d0>worstd) worstd=d0;
            if(d0) mism++;
        }
    }
    if(worstd) fail = 1;
    printf("  worst mismatch %ld LSB over %d probes  %s\n", worstd, RS*8, worstd? "FAIL":"OK");

    printf("\nRESULT: %s\n", fail ? "FAIL" : "PASS");
    return fail;
}
