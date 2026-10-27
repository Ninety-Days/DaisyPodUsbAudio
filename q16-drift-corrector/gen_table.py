#!/usr/bin/env python3
"""
gen_table.py — generates q16_sinc_table.h, the polyphase windowed-sinc
fractional-delay coefficient table used by the standalone corrector.

32 taps per phase, 128 phase rows plus a guard row, Kaiser window
beta=10, cutoff 0.97 x Nyquist. Prints the measured worst-case error and
four invariant checks; the figures go into the generated file's header
comment and into this directory's README.

Do not edit q16_sinc_table.h by hand -- regenerate it here. Regenerate it here if you change the
tap count, phase count or window.

MIT License (see LICENSE in this directory) applies to this file. That
licence is unscoped MIT and is not the scoped root LICENSE of the parent
repository; this subdirectory is self-contained and separately
redistributable.
"""
import numpy as np
fs=48000.0
N,P,BETA,FC = 32,128,10.0,0.97
CENTER = N//2-1                      # 15

def fd_taps(d):
    k=np.arange(N); x=k-CENTER-d
    h=FC*np.sinc(FC*x)
    a=(N-1)/2.0
    t=np.clip((k-(CENTER+d))/a,-1,1)
    h=h*np.i0(BETA*np.sqrt(1-t*t))/np.i0(BETA)
    return h/np.sum(h)

T=np.array([fd_taps(p/P) for p in range(P+1)], dtype=np.float32)   # float32!

# --- remeasure on the same float32 coefficients ---
def err(f,nd=1001,ncyc=48):
    w=2*np.pi*f/fs; worst=0.0
    for d in np.linspace(0,1,nd,endpoint=False):
        x=d*P; p=int(np.floor(x)); fr=np.float32(x-p)
        h=((np.float32(1.0)-fr)*T[p]+fr*T[p+1]).astype(np.float32)
        k=np.arange(N)
        for t in range(ncyc):
            ph=w*t
            y=float(np.dot(h.astype(np.float64),np.sin(ph+w*(k-CENTER))))
            e=abs(y-np.sin(ph+w*d))
            if e>worst: worst=e
    return 20*np.log10(worst+1e-15)

print("Worst-case output error on the final float32 table:")
res={}
for f in (1000,5000,10000,15000,20000):
    res[f]=err(f); print(f"  {f/1000:>5.0f} kHz   {res[f]:7.1f} dB FS")

# --- invariant checks ---
print(f"\n  tap sums: min {T.sum(axis=1).min():.9f}  max {T.sum(axis=1).max():.9f}")
print(f"  row 0 is a lowpass, not a delta: centre tap {T[0][CENTER]:.9f}, largest other {np.abs(np.delete(T[0],CENTER)).max():.2e}")
print(f"  guard row consistency (row P vs row 0 shifted): max|diff| = {np.abs(T[P][1:]-T[0][:-1]).max():.2e}")
print(f"  peak sum|h| (overload headroom): {np.abs(T).sum(axis=1).max():.4f}")

hdr=[]
hdr.append("/* q16_sinc_table.h -- polyphase windowed-sinc fractional-delay table")
hdr.append(" *")
hdr.append(f" * GENERATED FILE. Produced by gen_table.py; do not edit by hand.")
hdr.append(" * MIT License (see LICENSE in this directory) applies to this file.")
hdr.append(f" *   taps per phase (N)  : {N}")
hdr.append(f" *   phase rows (P)      : {P}  (+1 guard row for interpolation)")
hdr.append(f" *   window              : Kaiser, beta = {BETA}")
hdr.append(f" *   sinc cutoff         : {FC} x Nyquist")
hdr.append(f" *   size                : {(P+1)*N*4} bytes ({(P+1)*N*4/1024:.1f} KB) of float32")
hdr.append(" *")
hdr.append(" * Row p holds the taps for fractional delay d = p/P. Row P is the")
hdr.append(" * guard row (d = 1.0), needed so the blend between rows p and p+1 is")
hdr.append(" * defined at p = P-1. Taps run oldest-to-newest; tap k applies to the")
hdr.append(f" * ring sample at offset (k - {CENTER}) from the integer read position.")
hdr.append(" *")
hdr.append(" * Worst-case output error vs an ideal fractional delay, measured on")
hdr.append(" * these exact float32 coefficients with row interpolation:")
for f in (1000,5000,10000,15000,20000):
    hdr.append(f" *   {f//1000:>2} kHz   {res[f]:6.1f} dBFS")
hdr.append(" *")
hdr.append(" * The 20 kHz figure is limited by theory, not by this design: an ideal")
hdr.append(" * fractional delay at Nyquist is e^(-j*pi*d), which at d = 0.5 is purely")
hdr.append(" * imaginary, while any real FIR has a purely real response there. No tap")
hdr.append(" * count fixes that. */")
hdr.append("#ifndef Q16_SINC_TABLE_H")
hdr.append("#define Q16_SINC_TABLE_H")
hdr.append("")
hdr.append(f"#define Q16_FD_TAPS   {N}")
hdr.append(f"#define Q16_FD_CENTER {CENTER}")
hdr.append(f"#define Q16_FD_PHASES {P}")
hdr.append(f"#define Q16_FD_PHASE_BITS {P.bit_length()-1}")
hdr.append("")
hdr.append(f"static const float q16_sinc_table[({P}+1)*{N}] = {{")
flat=T.reshape(-1)
for p in range(P+1):
    hdr.append(f"    /* d = {p}/{P} */")
    row=T[p]
    for i in range(0,N,8):
        hdr.append("    "+" ".join(f"{v:.9g}f," for v in row[i:i+8]))
hdr.append("};")
hdr.append("")
hdr.append("#endif /* Q16_SINC_TABLE_H */")
open('q16_sinc_table.h','w').write("\n".join(hdr)+"\n")
import os
print(f"\n  written q16_sinc_table.h  ({os.path.getsize('q16_sinc_table.h')/1024:.0f} KB of text)")
