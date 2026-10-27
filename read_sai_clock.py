#!/usr/bin/env python3
"""
read_sai_clock.py -- work out what this board's audio sample rate ACTUALLY is.

Reads VENDOR_REQ_GET_CLOCK_TREE (0x45) and derives the SAI sample rate
from the live PLL/SAI registers, then compares it against the nominal
48000 Hz and against the offset the drift corrector measures.

WHY READ THE REGISTERS AT ALL
----------------------------
A ratio of integer dividers off a single HSE does not drift with
temperature, so a sample-rate error introduced there is a fixed
configuration error rather than drift -- and it is visible directly in
these registers. That is what this script checks: it is the verification
step for the libDaisy PLL3 patch described in the README and in
docs/CLOCKING.md.

Small offsets are a different matter and are NOT visible here: a physical
crystal deviating from the compile-time HSE_VALUE scales every derived
clock without changing any register. Only the drift corrector, which
compares against the host's actual SOF timing, can see that -- see
read_transport_diagnostics.py.

TWO INDEPENDENT PATHS, DELIBERATELY
-----------------------------------
The SAI kernel clock is computed twice: once here from the raw register
fields, and once on-target by HAL_RCCEx_GetPeriphCLKFreq (word 15).
A third, independent path is the corrector tilt reported by
read_transport_diagnostics.py CHECK 4.
If they disagree, the decode below is wrong and NOTHING it prints about
sample rate should be believed -- the script says so rather than
quietly preferring one.

Requires pyusb. Usage:
    python3 read_sai_clock.py
"""

# MIT License (see LICENSE at repository root) applies to this file.
import struct
import sys

try:
    import usb.core
    import usb.util
except ImportError:
    sys.exit("pyusb not installed -- run: pip install pyusb")

VID, PID = 0x0483, 0x5743
REQ_CLOCK_TREE = 0x45
NOMINAL_FS = 48000.0


def ctrl_read(dev, request, nwords):
    t = usb.util.build_request_type(
        usb.util.CTRL_IN, usb.util.CTRL_TYPE_VENDOR, usb.util.CTRL_RECIPIENT_DEVICE)
    return struct.unpack("<%dI" % nwords, bytes(dev.ctrl_transfer(t, request, 0, 0, nwords * 4)))


def bits(v, hi, lo):
    return (v >> lo) & ((1 << (hi - lo + 1)) - 1)


def vco_out(ref_hz, divm, divn, fracn, fracen):
    """STM32H7 PLL VCO output. DIVN/FRACN are the N.frac multiplier."""
    if divm == 0:
        return 0.0
    ref = ref_hz / divm
    mult = (divn + 1) + (fracn / 8192.0 if fracen else 0.0)
    return ref * mult


def main():
    dev = usb.core.find(idVendor=VID, idProduct=PID)
    if dev is None:
        sys.exit("Device not found (VID=0x%04x PID=0x%04x)." % (VID, PID))

    try:
        v = ctrl_read(dev, REQ_CLOCK_TREE, 16)
    except usb.core.USBError as e:
        sys.exit("Control transfer failed: %s\n"
                 "A STALL here means the running firmware predates vendor request "
                 "0x45 -- reflash before using this script." % e)

    (pllckselr, pllcfgr, pll1divr, pll1fracr, pll2divr, pll2fracr,
     pll3divr, pll3fracr, d2ccip1r, cfgr, cr, hse_value,
     sai1a_cr1, sai1b_cr1, sai1a_frcr, hal_sai_clk) = v

    print("RAW REGISTERS")
    for name, val in (("PLLCKSELR", pllckselr), ("PLLCFGR", pllcfgr),
                      ("PLL2DIVR", pll2divr), ("PLL2FRACR", pll2fracr),
                      ("PLL3DIVR", pll3divr), ("PLL3FRACR", pll3fracr),
                      ("D2CCIP1R", d2ccip1r), ("CR", cr),
                      ("SAI1_A_CR1", sai1a_cr1), ("SAI1_A_FRCR", sai1a_frcr)):
        print("  %-12s 0x%08X" % (name, val))
    print("  %-12s %d Hz (compile-time constant)" % ("HSE_VALUE", hse_value))
    print()

    # --- PLL source -----------------------------------------------------
    pllsrc = bits(pllckselr, 1, 0)
    src_name = {0: "HSI", 1: "CSI", 2: "HSE", 3: "none"}[pllsrc]
    if pllsrc == 2:
        ref = float(hse_value)
    elif pllsrc == 0:
        ref = 64e6 / (1 << bits(cr, 4, 3))     # HSI with its divider
    elif pllsrc == 1:
        ref = 4e6
    else:
        ref = 0.0
    print("PLL source: %s -> %.0f Hz reference" % (src_name, ref))

    # --- which PLL feeds SAI1 -------------------------------------------
    sai1sel = bits(d2ccip1r, 2, 0)
    sai_src = {0: "PLL1Q", 1: "PLL2P", 2: "PLL3P", 3: "I2S_CKIN",
               4: "PER_CK"}.get(sai1sel, "reserved(%d)" % sai1sel)
    print("SAI1 kernel source: %s (SAI1SEL=%d)" % (sai_src, sai1sel))

    # --- recompute the kernel clock from raw fields ----------------------
    # RM0433 PLLCKSELR field map: PLLSRC[1:0], DIVM1[9:4], DIVM2[17:12],
    # DIVM3[25:20]. An earlier version of this script used [13:8] and
    # [21:16], which read DIVM3 as 32 instead of 6 and put the computed
    # VCO out by a factor of ~5.3. The on-target HAL cross-check caught
    # it -- which is exactly why that cross-check is here.
    divm2, divm3 = bits(pllckselr, 17, 12), bits(pllckselr, 25, 20)
    pll2_fracen, pll3_fracen = bits(pllcfgr, 4, 4), bits(pllcfgr, 8, 8)

    pll2_vco = vco_out(ref, divm2, bits(pll2divr, 8, 0),
                       bits(pll2fracr, 15, 3), pll2_fracen)
    pll3_vco = vco_out(ref, divm3, bits(pll3divr, 8, 0),
                       bits(pll3fracr, 15, 3), pll3_fracen)
    pll2p = pll2_vco / (bits(pll2divr, 15, 9) + 1)
    pll3p = pll3_vco / (bits(pll3divr, 15, 9) + 1)

    derived = {1: pll2p, 2: pll3p}.get(sai1sel)
    if derived:
        which = "PLL2P" if sai1sel == 1 else "PLL3P"
        print("  %s VCO = %.3f Hz -> P output = %.3f Hz"
              % (which, pll2_vco if sai1sel == 1 else pll3_vco, derived))

    print("  HAL on-target reports SAI1 kernel clock = %d Hz" % hal_sai_clk)

    # --- cross-check the two paths --------------------------------------
    if derived is None:
        print("\n  Cannot recompute this source independently; trusting HAL only.")
        kernel = float(hal_sai_clk)
        cross_ok = None
    else:
        disagree = abs(derived - hal_sai_clk)
        cross_ok = disagree < max(1.0, hal_sai_clk * 1e-6)
        if cross_ok:
            print("  CROSS-CHECK OK: the two paths agree to %.3f Hz." % disagree)
            kernel = float(hal_sai_clk)
        else:
            print("\n  CROSS-CHECK FAILED: independent decode gives %.3f Hz but HAL "
                  "reports %d Hz (%.1f Hz apart)." % (derived, hal_sai_clk, disagree))
            print("  The register decode in this script is therefore wrong. Do NOT "
                  "trust the sample rate below -- fix the decode first.")
            kernel = float(hal_sai_clk)

    # --- kernel clock -> sample rate -------------------------------------
    mckdiv = bits(sai1a_cr1, 26, 20)
    nomck  = bits(sai1a_cr1, 19, 19)
    print()
    print("SAI1 block A: MCKDIV=%d, NOMCK=%d" % (mckdiv, nomck))

    if mckdiv == 0:
        print("  MCKDIV=0 (bypass) -- sample rate cannot be derived from these "
              "fields alone; inspect the SAI configuration directly.")
        return

    fs = kernel / (mckdiv * 256.0)
    ppm = (fs - NOMINAL_FS) / NOMINAL_FS * 1e6

    print()
    print("=" * 68)
    print("  Computed sample rate : %.4f Hz" % fs)
    print("  Nominal              : %.0f Hz" % NOMINAL_FS)
    print("  Error                : %+.4f Hz  (%+.2f ppm)" % (fs - NOMINAL_FS, ppm))
    print("=" * 68)
    print()
    if abs(ppm) < 1.0:
        print("  The libDaisy PLL3 patch is in effect: the rate is configured")
        print("  exactly. Any residual offset against the host is real crystal")
        print("  tolerance, which the drift corrector absorbs.")
    else:
        print("  This is a configuration error, not drift -- it comes from a ratio")
        print("  of integer dividers and does not move with temperature. Around")
        print("  +298 ppm means the libDaisy PLL3 patch is not in effect: see the")
        print("  build section of the README and docs/CLOCKING.md. Note also that")
        print("  a board whose actual crystal differs from the compile-time")
        print("  HSE_VALUE above would scale every derived clock without showing")
        print("  up in any register.")
    if cross_ok is False:
        print()
        print("  (Reminder: the cross-check FAILED, so the number above is not "
              "trustworthy yet.)")


if __name__ == "__main__":
    main()
