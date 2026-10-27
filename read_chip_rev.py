#!/usr/bin/env python3
"""
read_chip_rev.py -- one-shot query of VENDOR_REQ_GET_CHIP_REV (0x43).

Reads:
  word0 = raw DBGMCU_IDC register (0x5C001000) -- decodes DEV_ID/REV_ID
          to settle which STM32H750 silicon revision this board has.
  word1 = live CRS->CFGR SYNCSRC[1:0] field -- decodes which physical
          USB core the CRS peripheral is actually configured to listen
          to for SOF pulses, to cross-check against which core this
          firmware's USB stack (usbd_conf.c's "USB_OTG_FS") actually is
          on STM32H7 (which has two USB cores).

See ES0392 errata sheet §2.2.3 ("Clock recovery system synchronization
with USB SOF does not work") for the revision-scoped hardware erratum
this was originally added to check for; this also checks for a
config-level SYNCSRC mismatch, which produces the exact same observable
symptom (crs_sync_ok_count stuck at 0) but is a firmware fix, not a
silicon limitation.

Note on SYNCSRC's bit position: it's bits [29:28] of CRS_CFGR, confirmed
against ST's official SVD for STM32H7 (via docs.rs/stm32ral). An earlier
version of the firmware's diagnostic read (and of this script's
expectations) assumed bits [15:14] -- that was never checked against a
primary source and turned out to overlap RELOAD[15:0]'s own bits, which
is why readbacks kept showing "SYNCSRC=2" no matter what crs.Source was
actually set to in main.cpp (2 happens to be bits [15:14] of this
firmware's fixed ReloadValue=47999=0xBB7F). A related dead end: at one
point this looked like it might be a RevY-vs-RevV SYNCSRC encoding
change per ST's AN5312 migration note -- that hypothesis only existed
because the [15:14] readback was broken and has been retracted now that
the read is fixed; the standard RCC_CRS_SYNC_SOURCE_* HAL macros were
correct the whole time.

Requires pyusb (`pip install pyusb`) and a libusb backend (e.g.
`brew install libusb` on macOS).

Usage:
    python3 read_chip_rev.py
"""

# MIT License (see LICENSE at repository root) applies to this file.
import sys
import struct

try:
    import usb.core
    import usb.util
except ImportError:
    sys.exit("pyusb not installed -- run: pip install pyusb")

VID = 0x0483   # ST's vendor ID, see usbd_desc.c USBD_VID
PID = 0x5743   # usbd_desc.c USBD_PID_FS

VENDOR_REQ_GET_CHIP_REV = 0x43

# REV_ID -> silicon revision letter, per ES0392 Table 2 (Device variants)
REV_ID_MAP = {
    0x1003: "Y or W",
    0x2001: "X",
    0x2003: "V",
}

# ES0392 §2.2.3 status per revision, from the errata's Table 3 summary row
# "2.2.3 Clock recovery system synchronization with USB SOF does not work":
#   Rev Y,W -> A (limitation present, workaround available)
#   Rev X   -> "-" (limitation absent)
#   Rev V   -> "-" (limitation absent)
CRS_SOF_ERRATUM_STATUS = {
    "Y or W": "AFFECTED -- CRS SOF sync does not work on this silicon (errata 2.2.3)",
    "X":      "NOT listed as affected in ES0392 for this erratum",
    "V":      "NOT listed as affected in ES0392 for this erratum",
}

# CRS->CFGR SYNCSRC[1:0] decode (bits [29:28] of the register). Matches
# RCC_CRS_SYNC_SOURCE_USB1/USB2 in stm32h7xx_hal_rcc_ex.h.
SYNCSRC_MAP = {
    0: "GPIO/PIN",
    1: "LSE",
    2: "USB1",
    3: "USB2",
}

# This firmware's USB stack (usbd_conf.c's "USB_OTG_FS" instance) is the
# physical core ST calls "USB2" on STM32H7.
FIRMWARE_USB_CORE_SYNCSRC = 3


def main():
    dev = usb.core.find(idVendor=VID, idProduct=PID)
    if dev is None:
        sys.exit(f"DaisyPodUsbAudio not found (VID=0x{VID:04x} PID=0x{PID:04x}) -- "
                  f"is it connected and enumerated?")

    # Device-to-Host, Vendor, Device recipient -- same shape as the existing
    # the host diagnostic control transfers.
    bmRequestType = usb.util.build_request_type(
        usb.util.CTRL_IN, usb.util.CTRL_TYPE_VENDOR, usb.util.CTRL_RECIPIENT_DEVICE)

    try:
        raw = dev.ctrl_transfer(bmRequestType, VENDOR_REQ_GET_CHIP_REV, 0, 0, 8)
    except usb.core.USBError as e:
        sys.exit(f"Control transfer failed: {e}\n"
                  f"(if this is 'Pipe error'/STALL, the firmware on this board "
                  f"doesn't have VENDOR_REQ_GET_CHIP_REV yet, or has an older "
                  f"version -- reflash first)")

    idc, syncsrc = struct.unpack("<II", bytes(raw))
    dev_id = idc & 0xFFF
    rev_id = (idc >> 16) & 0xFFFF

    rev_letter = REV_ID_MAP.get(rev_id, f"UNKNOWN (0x{rev_id:04x})")
    status = CRS_SOF_ERRATUM_STATUS.get(rev_letter, "unknown -- check ES0392 manually")
    syncsrc_name = SYNCSRC_MAP.get(syncsrc, f"UNKNOWN ({syncsrc})")

    print(f"DBGMCU_IDC raw     : 0x{idc:08x}")
    print(f"DEV_ID             : 0x{dev_id:03x}  (0x450 expected for STM32H750)")
    print(f"REV_ID             : 0x{rev_id:04x}")
    print(f"Silicon revision   : {rev_letter}")
    print(f"ES0392 §2.2.3 (CRS/SOF): {status}")
    print()
    print(f"CRS->CFGR SYNCSRC  : {syncsrc} ({syncsrc_name})")

    if syncsrc == FIRMWARE_USB_CORE_SYNCSRC:
        print("\nMATCH: CRS is synchronised to USB2 SOF, the core this firmware "
              "uses.")
    else:
        print(f"\nMISMATCH: CRS is listening to SYNCSRC={syncsrc} ({syncsrc_name}),")
        print(f"but this firmware's USB stack uses USB2 (SYNCSRC={FIRMWARE_USB_CORE_SYNCSRC}).")
        print(f"This alone would fully explain crs_sync_ok_count never incrementing:")
        print(f"CRS is correctly configured and running, just listening to the")
        print(f"wrong physical USB core's SOF signal.")

    if dev_id != 0x450:
        print("\nNOTE: DEV_ID doesn't match the expected STM32H750 (0x450) -- "
              "double check this is the right device/part.")


if __name__ == "__main__":
    main()
