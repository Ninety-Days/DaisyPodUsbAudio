/* AM-envelope probe: drives the REAL fd_read_frame() from the shipped
 * firmware at a fixed integer tilt and dumps the output stream.
 * fd_read_frame body copied verbatim from tests/interp/interp_check.c. */

/* MIT License (see LICENSE at repository root) applies to this file. */
#include <stdio.h>
#include <stdint.h>
#include <math.h>
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
static int32_t ring[RS*2];

/* argv: freq  tilt  seconds */
int main(int argc,char**argv){
    const double fs=48000.0;
    double f     = argc>1? atof(argv[1]) : 12000.0;
    int    tilt  = argc>2? atoi(argv[2]) : 4;
    double secs  = argc>3? atof(argv[3]) : 10.0;
    const double A = 2097152.0;            /* -12 dBFS of 24-bit */
    long   N = (long)(fs*secs);

    uint32_t wptr = 0;
    /* read position in Q16, starts 256 frames behind the writer */
    uint64_t rq = 0;
    /* prime the ring with 256 frames so the reader has history */
    for(uint32_t j=0;j<RS;j++){ ring[j*2]=ring[j*2+1]=0; }
    for(long n=0;n<256;n++){
        double s=A*sin(2*M_PI*f*(double)n/fs);
        ring[wptr*2+0]=(int32_t)llround(s);
        ring[wptr*2+1]=(int32_t)llround(s);
        wptr=(wptr+1u)&(RS-1u);
    }
    long nin=256;
    for(long m=0;m<N;m++){
        /* writer: one input frame per output frame (nominal) */
        double s=A*sin(2*M_PI*f*(double)nin/fs);
        ring[wptr*2+0]=(int32_t)llround(s);
        ring[wptr*2+1]=(int32_t)llround(s);
        wptr=(wptr+1u)&(RS-1u); nin++;
        /* reader: fractional position advancing by 1 + tilt/65536 */
        uint32_t idx  = (uint32_t)((rq>>16) & (RS-1u));
        uint32_t frac = (uint32_t)(rq & 0xFFFFu);
        int32_t o[2];
        fd_read_frame(ring, RS-1u, 2u, idx, frac, o);
        float v=(float)o[0];
        fwrite(&v,sizeof(float),1,stdout);
        rq += (uint64_t)65536 + (int64_t)tilt;
    }
    return 0;
}
