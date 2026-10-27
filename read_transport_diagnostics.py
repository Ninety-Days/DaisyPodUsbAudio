#!/usr/bin/env python3
"""
read_transport_diagnostics.py -- long-window validation of the transport
layer, over USB vendor requests.

Checks:
  1  CRS lock                  (is SYNCOK firing at all?)
  2  Packet-size constancy     (the central claim -- decisive)
  3  IN-pipe health            (did the pipe stall / did the watchdog fire?)
  4  Clock offset, measured    (read directly from the tilt, not inferred)
  5  Standing-offset consistency and headroom
  6  Stream health             (underruns/overruns, bounded fill)

The rate tilt at equilibrium must exactly cancel the clock offset, so the
tilt IS the offset and is read directly:

    ppm = tilt_q16 / 65536 * 1e6

No inference from event counts, no dependence on window length. The two
directional counters still separate "the corrector wanted to pull up" from
"down", which is worth seeing, but they are not events and their ratio has
no physical meaning.

Requires pyusb (`pip install pyusb`) and a libusb backend
(`brew install libusb` on macOS).

Usage:
    python3 read_transport_diagnostics.py                # 160s window
    python3 read_transport_diagnostics.py --seconds 300
    python3 read_transport_diagnostics.py --interval 0.5
"""

# MIT License (see LICENSE at repository root) applies to this file.
import argparse
import struct
import sys
import time

try:
    import usb.core
    import usb.util
except ImportError:
    sys.exit("pyusb not installed -- run: pip install pyusb")

VID = 0x0483
PID = 0x5743

REQ_RING      = 0x41
REQ_RING_EX   = 0x42
REQ_HEALTH    = 0x46

# Mirrors usbd_audio_if.c. If you change them there, change them here --
# CHECK 5 is a consistency test between firmware and these values, so a
# stale copy shows up as a spurious failure rather than a silent one.
PB_RING_SIZE = 512
PB_TARGET    = 256
REC_TARGET   = 256
RATE_DIV     = 8              # REC_RATE_DIV / PB_RATE_DIV
RATE_MAX_Q16 = 32             # clamp, both directions
FRAC_ONE     = 65536
SAMPLE_RATE  = 48000.0

CRS_POLL_RATE_MIN = 700.0     # Audio_PeriodicTC runs ~1kHz; wide band


def q16_to_ppm(tilt):
    return tilt / FRAC_ONE * 1e6


def ctrl_read(dev, request, nwords):
    bmRequestType = usb.util.build_request_type(
        usb.util.CTRL_IN, usb.util.CTRL_TYPE_VENDOR, usb.util.CTRL_RECIPIENT_DEVICE)
    raw = dev.ctrl_transfer(bmRequestType, request, 0, 0, nwords * 4)
    return struct.unpack("<%dI" % nwords, bytes(raw))


def s32(u):
    """Reinterpret a uint32 slot as the signed value the firmware bit-cast in."""
    return u - (1 << 32) if u >= (1 << 31) else u


# Spacing between the vendor reads within one sample.
#
# These three requests used to be issued back to back. Under playback that
# burst intermittently killed the control pipe -- a measured ~1.25% per
# sample, spread evenly through a window rather than accumulating, so a
# 160s run finished only about two times in three. The transport itself was
# never implicated: watchdog kicks, packet-size deviations and under/overruns
# stayed at zero through every failure, and the device was never observed to
# stall EP0.
#
# Spacing the reads out removes the burst: 0 failures in 600 polls against
# 7.5 expected at the old rate. It costs nothing here -- every counter this
# script reads is free-running, and the deltas it computes span whole
# intervals, not the gap inside one sample.
#
# Requires --interval to stay above 2 * SAMPLE_SPREAD, which the default of
# 1.0s does comfortably. Below that the loop's remaining sleep hits zero and
# the spread eats the whole period.
SAMPLE_SPREAD = 0.30    # seconds between the three reads of one sample


def sample(dev, have_health):
    b = ctrl_read(dev, REQ_RING, 8)
    time.sleep(SAMPLE_SPREAD)
    e = ctrl_read(dev, REQ_RING_EX, 16)
    if have_health:
        time.sleep(SAMPLE_SPREAD)
        h = ctrl_read(dev, REQ_HEALTH, 4)
    else:
        h = (0, 0, 0, 0)

    packed_crs  = e[14]
    packed_poll = e[15]

    return {
        "t": time.monotonic(),
        "pb_overrun":   b[4],
        "pb_underrun":  b[5],
        "rec_underrun": b[6],
        "rec_avail":     e[0],
        "pb_avail":      e[1],
        "rec_min":       e[2],
        "rec_max":       e[3],
        "pb_min":        e[4],
        "pb_max":        e[5],
        "rec_up":        e[6],
        "rec_down":      e[7],
        "pb_up":         e[8],
        "pb_down":       e[9],
        # these are the fractional rate tilt in Q16, NOT the old
        # sigma-delta accumulator. Same slots, different meaning.
        "rec_tilt":      s32(e[10]),
        "pb_tilt":       s32(e[11]),
        "crs_ok":        packed_crs & 0xFFFF,
        "crs_miss":      (packed_crs >> 16) & 0xFF,
        "crs_trim_ovf":  (packed_crs >> 24) & 0xFF,
        "crs_trim":      packed_poll & 0x3F,
        "crs_polls":     (packed_poll >> 8) & 0xFFFFFF,
        # 0x46
        "datain":        h[0],
        "kicks":         h[1],
        "tx_dev":        h[2],
        "pb_advances":   h[3],
    }


def mean(xs):
    return sum(xs) / len(xs) if xs else 0.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds",  type=float, default=160.0)
    ap.add_argument("--interval", type=float, default=1.0)
    ap.add_argument("--pb-target", type=int, default=PB_TARGET,
                    help="PB_TARGET the firmware was built with.")
    args = ap.parse_args()
    pb_target = args.pb_target

    dev = usb.core.find(idVendor=VID, idProduct=PID)
    if dev is None:
        sys.exit("DaisyPodUsbAudio not found (VID=0x%04x PID=0x%04x) -- "
                 "connected and enumerated?" % (VID, PID))

    # 0x46 exists only in this firmware. A STALL here is the cleanest available
    # signal that the board is running older firmware whose slots 10/11
    # mean something entirely different -- worth detecting explicitly
    # rather than misreading older counters as current tilts.
    have_health = True
    try:
        ctrl_read(dev, REQ_HEALTH, 4)
    except usb.core.USBError:
        have_health = False

    try:
        first = sample(dev, have_health)
    except usb.core.USBError as e:
        sys.exit("Control transfer failed: %s\n"
                 "If this is a STALL/pipe error, the board is running firmware "
                 "without the EX diagnostics -- reflash before testing." % e)

    if not have_health:
        print("WARNING: vendor request 0x46 STALLed. This board is NOT running")
        print("         this firmware. Slots 10/11 on older builds carry the")
        print("         sigma-delta accumulator, not a rate tilt, so CHECK 4")
        print("         below would misreport the clock offset by a large")
        print("         factor. Checks 2 and 3 are skipped entirely.")
        print("         Reflash before trusting anything here.\n")
    else:
        print("Firmware confirmed: vendor requests 0x41, 0x42 and 0x46 respond "
              "(0x46 is present only in this firmware).")

    print("Polling %.0fs at %.2fs intervals. Play audio for the whole window.\n"
          % (args.seconds, args.interval))
    print("   t     rec_av  pb_av  rec_tilt pb_tilt   rec_ppm   pb_ppm   "
          "crs_ok  trim  kicks  txdev")
    print("-" * 100)

    samples = [first]
    t0 = first["t"]
    deadline = t0 + args.seconds

    try:
        while time.monotonic() < deadline:
            # Sleep the remainder of the interval, not the whole of it: a
            # sample now takes SAMPLE_SPREAD * 2 to collect, and sleeping
            # the full interval on top would stretch the effective period
            # and silently change what the printed t column means.
            time.sleep(max(0.0, args.interval - 2 * SAMPLE_SPREAD))
            s = sample(dev, have_health)
            samples.append(s)
            print("%6.1f  %6d %6d  %7d %7d  %+8.1f %+8.1f  %6d  %4d  %5d  %5d"
                  % (s["t"] - t0, s["rec_avail"], s["pb_avail"],
                     s["rec_tilt"], s["pb_tilt"],
                     q16_to_ppm(s["rec_tilt"]), q16_to_ppm(s["pb_tilt"]),
                     s["crs_ok"], s["crs_trim"], s["kicks"], s["tx_dev"]))
    except KeyboardInterrupt:
        print("\n(interrupted -- evaluating on the window collected so far)")
    except usb.core.USBError as e:
        sys.exit("\nControl transfer failed mid-run: %s\n"
                 "A disconnect/STALL mid-window is itself a finding -- note it." % e)

    a, b = samples[0], samples[-1]
    elapsed = b["t"] - a["t"]
    if elapsed <= 0 or len(samples) < 3:
        sys.exit("Window too short to evaluate.")

    d_polls = b["crs_polls"] - a["crs_polls"]
    d_datain = b["datain"] - a["datain"]

    # crs_sync_ok_count arrives as the LOW 16 BITS ONLY (see the packing of
    # vals[14] in usbd_audio.c). At ~1000 SYNCOK/s it wraps every ~65s, so
    # an endpoint difference badly under-reports. Sum per-sample deltas:
    # at a 1s interval each step is ~1000, far below the wrap point, so
    # per-step correction is unambiguous.
    d_ok = 0
    for prev, cur in zip(samples, samples[1:]):
        d_ok += (cur["crs_ok"] - prev["crs_ok"]) % 65536

    verdicts = []

    def emit(tag, ok, msg):
        verdicts.append(ok)
        mark = {True: "PASS", False: "FAIL", None: "INCONCLUSIVE"}[ok]
        print("[%-12s] %s\n               %s" % (mark, tag, msg))

    print("\n" + "=" * 100)
    print("VERDICTS  (window %.1fs, %d samples)" % (elapsed, len(samples)))
    print("=" * 100)

    # --- CHECK 1: CRS ---------------------------------------------------
    # The polling loop lives at the top of Audio_PeriodicTC, the DataIn
    # callback for the isochronous IN endpoint, so it only runs while the
    # host is actively READING the record stream. A playback-only test
    # leaves it at zero: an idle input stream, not a broken loop.
    poll_rate = d_polls / elapsed
    record_idle = (d_polls == 0)

    if record_idle:
        emit("CRS lock (Step 1)", None,
             "crs_poll_checked_count did not move -- the host never opened the "
             "record stream, so Audio_PeriodicTC never ran. Test-setup "
             "condition, not a fault. Standing evidence: crs_sync_ok=%d, "
             "TRIM=%d (32 = never adjusted). Rerun with an input stream active."
             % (b["crs_ok"], b["crs_trim"]))
    elif poll_rate < CRS_POLL_RATE_MIN:
        emit("CRS lock (Step 1)", None,
             "Polling loop running below rate (%.0f/s, expected ~1000/s) -- not "
             "evaluable." % poll_rate)
    else:
        emit("CRS lock (Step 1)", d_ok > 0,
             "crs_sync_ok +%d over %.1fs = %.0f/s against %.0f polls/s "
             "(wrap-corrected; the field is 16 bits). sync_miss=%d, "
             "trim_ovf=%d, TRIM=%d (32=no adjustment, 0/63=rail).%s"
             % (d_ok, elapsed, d_ok / elapsed, poll_rate, b["crs_miss"],
                b["crs_trim_ovf"], b["crs_trim"],
                "" if d_ok > 0 else " SYNCOK never fired despite a "
                "confirmed-running poll loop -- CRS remains an open question."))
        if b["crs_trim"] in (0, 63):
            print("               NOTE: TRIM railed at %d -- CRS is out of "
                  "correction range even if SYNCOK fires." % b["crs_trim"])

    # --- CHECK 2: the central claim --------------------------------
    # This firmware declares the endpoint Adaptive and asserts the IN packet is
    # always exactly SAMPLES_NOM frames. tx_size_deviations counts every
    # packet that was not. There is no benign non-zero value: a short
    # packet means the descriptor is lying to the host.
    if not have_health:
        emit("Packet-size constancy (core claim)", None,
             "Not measurable -- 0x46 unavailable on this firmware.")
    elif record_idle:
        emit("Packet-size constancy (core claim)", None,
             "Record stream never opened, so no IN packets were sent and the "
             "claim was not exercised. tx_size_deviations=%d." % b["tx_dev"])
    else:
        d_dev = b["tx_dev"] - a["tx_dev"]
        emit("Packet-size constancy (core claim)", b["tx_dev"] == 0,
             "tx_size_deviations = %d absolute (+%d this window) over ~%d IN "
             "packets.%s"
             % (b["tx_dev"], d_dev, d_datain,
                "" if b["tx_dev"] == 0 else
                " NON-ZERO: every deviation is the safety clamp firing, i.e. the "
                "ring ran dry mid-packet, so the Adaptive descriptor's "
                "constant-rate promise is being broken -- exactly what this "
                "firmware was supposed to fix. Correlate with the underrun count "
                "in CHECK 6: if underruns are also non-zero, the ring is starving "
                "and the corrector is not keeping up."))

    # --- CHECK 3: IN-pipe health ---------------------------------------
    # An earlier deadlock made a dead pipe indistinguishable from a healthy
    # one from the host side: SET_INTERFACE still succeeded, in_streaming
    # still read 1, and audio silently stopped. datain_completions is the
    # observable that separates them.
    if not have_health:
        emit("IN-pipe health (this firmware deadlock fix)", None,
             "Not measurable -- 0x46 unavailable on this firmware.")
    elif record_idle:
        emit("IN-pipe health (this firmware deadlock fix)", None,
             "Record stream never opened; nothing to stall. kicks=%d."
             % b["kicks"])
    else:
        datain_rate = d_datain / elapsed
        # A frozen counter is the dead-pipe signature.
        stalled = (d_datain == 0)
        # What matters is whether the pipe stalls DURING the measured
        # window (d_kicks), not the absolute total. Enumeration and the
        # first alt-setting open reliably cost a few kicks before the
        # steady state is reached -- the host probes the interface, an
        # early ISO frame is missed, the watchdog re-arms. Those are the
        # watchdog doing exactly its job, not evidence of an unstable
        # link. A non-zero d_kicks (the pipe died while streaming) is the
        # real fault. b["kicks"] is still reported so a large startup
        # count doesn't go unseen -- but it is not itself a failure.
        d_kicks = b["kicks"] - a["kicks"]
        healthy = (not stalled) and d_kicks == 0
        if healthy and b["kicks"] > 0:
            note = (" %d kick(s) happened before this window (startup: "
                    "enumeration / first alt-setting) and none during it -- "
                    "the watchdog recovered the initial open and then stayed "
                    "quiet, which is correct behaviour, not a fault." % b["kicks"])
        elif healthy:
            note = ""
        elif stalled:
            note = (" PIPE FROZE: DataIn stopped firing entirely while the "
                    "stream was open -- the deadlock signature.")
        else:
            note = (" PIPE STALLED MID-STREAM: the watchdog fired +%d time(s) "
                    "DURING this window, so the pipe died while streaming and "
                    "was recovered. Audio survived, but this is a real stall "
                    "-- correlate with a suspend/resume or host-side ISO miss." % d_kicks)
        emit("IN-pipe health (this firmware deadlock fix)", healthy,
             "datain_completions +%d over %.1fs = %.0f/s (expected ~1000/s). "
             "Watchdog kicks: %d absolute, +%d this window.%s"
             % (d_datain, elapsed, datain_rate, b["kicks"], d_kicks, note))

    # LIVE SAMPLES -- used by CHECK 4 and CHECK 5 alike.
    #
    # crs_sync_ok is incremented by Audio_PeriodicTC, which only runs while
    # the IN pipe is delivering, so it is a direct "the stream is running"
    # flag. A sample is trusted only when the counter advanced both into it
    # and out of it: the snapshot inside a sample where the host closed the
    # interface mid-poll carries a fill from before the close and a tilt
    # already zeroed by AudioIF_Reset*, and comparing or averaging those two
    # is meaningless. Excluding it is not hiding a failure -- it is
    # declining to use two numbers that were never sampled at the same
    # instant.
    live = [False] * len(samples)
    for i in range(1, len(samples)):
        adv_in  = ((samples[i]["crs_ok"] - samples[i - 1]["crs_ok"]) & 0xFFFF) > 0
        adv_out = (i + 1 < len(samples)
                   and ((samples[i + 1]["crs_ok"] - samples[i]["crs_ok"]) & 0xFFFF) > 0)
        live[i] = adv_in and adv_out
    n_live = sum(live)

    # --- CHECK 4: clock offset, measured directly ----------------------
    # The tilt at equilibrium must exactly cancel the clock offset, so it
    # IS the offset. This supersedes the older method of counting one-frame
    # corrections -- see WHAT CHANGED in the module docstring.
    #
    # "At equilibrium" is the load-bearing part, and averaging the whole
    # window does not deliver it. A window that contains a stream open
    # contains a prime drain instead: the record ring fills freely to ~500
    # before rec_active arms, and the corrector then sits near its rail for
    # twenty-odd seconds pulling it back down. Averaged in, that transient
    # does not merely add noise -- it flips the sign. Measured on ten
    # consecutive 60 s runs of the same board: the eight windows without an
    # open read rec -55.3 to -59.8 ppm, while the two containing one were
    # reported as +13.3 and +10.8 ppm by the plain mean.
    #
    # So: measure on the largest suffix of the window whose two halves
    # agree to within one Q16 step, and say plainly when that is not the
    # whole window. Splitting in half is a crude stationarity test, but it
    # is one a reader can verify by eye against the printed table, which a
    # fitted trend would not be.
    #
    # A trimmed window is still not equilibrium, only closer to it. In the
    # two runs above the corrector had not finished converging by the end
    # of 60 s -- at a tilt of one to two Q16 steps it moves the fill about
    # 1.5 frames per second and had eighteen frames left to travel -- so
    # the trimmed reading came out at -46.7 ppm against a true -57. That is
    # a floor on the magnitude, not the value, and the note below says so.
    # No statistic recovers equilibrium from a window that does not contain
    # any; the honest move is to ask for another window.
    SETTLE_MIN_SAMPLES = 20
    SETTLE_TOL_Q16     = 1.0

    def settled_from(tilts):
        """Index of the largest suffix whose halves agree, or None."""
        for start in range(0, len(tilts) - SETTLE_MIN_SAMPLES + 1):
            seg = tilts[start:]
            half = len(seg) // 2
            if abs(mean(seg[:half]) - mean(seg[half:])) <= SETTLE_TOL_Q16:
                return start
        return None

    rec_tilts = [s_["rec_tilt"] for s_ in samples]
    pb_tilts  = [s_["pb_tilt"]  for s_ in samples]
    live_rec  = [s_["rec_tilt"] for i, s_ in enumerate(samples) if live[i]]
    live_pb   = [s_["pb_tilt"]  for i, s_ in enumerate(samples) if live[i]]

    print("[%-12s] %s" % ("MEASUREMENT", "Clock-domain offset (from tilt)"))

    if len(live_rec) < SETTLE_MIN_SAMPLES:
        print("               Only %d sample(s) had the stream running -- too few "
              "to measure. Play audio for the whole window." % len(live_rec))
        rec_ppm = pb_ppm = 0.0
    else:
        trimmed = False
        for label, tilts in (("rec", live_rec), ("pb", live_pb)):
            start = settled_from(tilts)
            if start is None:
                print("               %s tilt never settles within this window "
                      "(range %d..%d) -- the corrector is still converging "
                      "throughout; no offset is measurable here."
                      % (label, min(tilts), max(tilts)))
                ppm = 0.0
            else:
                seg = tilts[start:]
                ppm = q16_to_ppm(mean(seg))
                if start:
                    trimmed = True
                print("               %-3s tilt mean %+.2f Q16 -> %+.1f ppm  "
                      "(range %d..%d%s)"
                      % (label, mean(seg), ppm, min(seg), max(seg),
                         "" if not start else
                         ", from %d of %d live samples" % (len(seg), len(tilts))))
            if label == "rec":
                rec_ppm = ppm
            else:
                pb_ppm = ppm

        print("               Read from the applied rate correction, not inferred "
              "from event counts.")
        if trimmed:
            print("               This window contains a stream open. The leading "
                  "prime drain was excluded,")
            print("               but the corrector had not finished converging "
                  "either, so the figure is a")
            print("               floor on the magnitude. Repeat on a window with "
                  "no open in it.")

    # The full reading -- divider error vs physical crystal deviation, and why
    # the latter shows up in no register at all -- lives in docs/CLOCKING.md
    # section 4. Printing it every run said the same fifteen lines regardless
    # of the number above it.
    if max(abs(rec_ppm), abs(pb_ppm)) > 15.0:
        print("               Cross-check against read_sai_clock.py; "
              "docs/CLOCKING.md section 4 explains how to read the pair.")

    # --- CHECK 5: standing offset and headroom -------------------------
    # A proportional-only controller holds a standing offset: to sustain a
    # tilt it must sit at error = tilt * RATE_DIV away from target. That is
    # by design, but it consumes ring headroom, and the amount consumed
    # scales with the clock offset. Worth checking explicitly because the
    # failure mode is an underrun that looks unrelated to the corrector.
    #
    # WHY THIS IS PER-SAMPLE AND NOT ON THE MEANS. The earlier version
    # compared mean(tilt) * RATE_DIV against mean(avail) - target and
    # failed whenever the two disagreed by more than a rounding step. That
    # test reported "the RATE_DIV/target constants in this script no longer
    # match the firmware" on hardware where they matched exactly. The
    # relation tilt = clamp(trunc(err / RATE_DIV), +-RATE_MAX_Q16) is
    # integer-truncating and saturating, so the mean of the relation is not
    # the relation of the means. One long startup transient -- the record
    # ring primes near 511 and the corrector sits at the rail for twenty
    # seconds draining it -- is enough to pull the average off by more than
    # the tolerance, with every individual sample still exact.
    #
    # Checking each sample instead is both correct and strictly stronger:
    # a 60 s window gives ~110 independent comparisons rather than one
    # averaged one. On the run that provoked the false failure, all 110
    # matched.
    #
    # LIVE SAMPLES. crs_sync_ok is incremented by Audio_PeriodicTC, which
    # only runs while the IN pipe is delivering, so it is a direct "the
    # stream is running" flag. A sample is trusted only when the counter
    # advanced both into it and out of it: the snapshot inside a sample
    # where the host closed the interface mid-poll carries a fill from
    # before the close and a tilt already zeroed by AudioIF_Reset*, and
    # comparing those two is meaningless. Excluding it is not hiding a
    # failure -- it is declining to compare two numbers that were never
    # sampled at the same instant.

    def expected_tilt(avail, target):
        err = avail - target
        q = abs(err) // RATE_DIV
        q = q if err >= 0 else -q
        return max(-RATE_MAX_Q16, min(RATE_MAX_Q16, q))

    if n_live < 10:
        emit("Standing offset and headroom", None,
             "Only %d sample(s) had the stream running on both sides -- too "
             "few to evaluate. Play audio for the whole window." % n_live)
    elif not any(rec_tilts) and not any(pb_tilts):
        emit("Standing offset and headroom", None,
             "Both tilts sat at zero all window -- either nothing streamed or "
             "the clocks are matched to within one Q16 step (~15 ppm).")
    else:
        problems = []
        notes = []
        max_frac = 0.0
        for label, key, tkey, target in (
                ("rec", "rec_avail", "rec_tilt", REC_TARGET),
                ("pb",  "pb_avail",  "pb_tilt",  pb_target)):
            mism = []
            rail = 0
            mags = []
            offs = []
            for i, s_ in enumerate(samples):
                if not live[i]:
                    continue
                tilt = s_[tkey]
                mags.append(abs(tilt))
                offs.append(abs(s_[key] - target))
                if tilt != expected_tilt(s_[key], target):
                    mism.append((s_["t"], s_[key], tilt, expected_tilt(s_[key], target)))
                if abs(tilt) >= RATE_MAX_Q16:
                    rail += 1
            if not mags:
                continue
            # Tolerate a couple of stragglers: a poll can still straddle an
            # alt-setting change that crs_sync_ok has not yet reflected. A
            # constants mismatch would show on essentially every sample, not
            # on two of a hundred.
            if len(mism) > max(2, int(0.05 * len(mags))):
                t_, fill_, got_, exp_ = mism[0]
                problems.append(
                    "%s: %d of %d live samples violate tilt = "
                    "clamp(trunc((fill-%d)/%d), +-%d) -- first at t=%.1fs, "
                    "fill=%d gave tilt=%d, expected %d. The RATE_DIV/target "
                    "constants in this script no longer match the firmware"
                    % (label, len(mism), len(mags), target, RATE_DIV,
                       RATE_MAX_Q16, t_, fill_, got_, exp_))
            # Sustained saturation, not a mean. Priming the record ring puts
            # the corrector on the rail for a few seconds at every stream
            # open, which is normal and must not fail the run.
            if rail > 0.25 * len(mags):
                problems.append(
                    "%s: tilt sat at the +-%d clamp for %d of %d live samples "
                    "(%.0f ppm) -- the corrector is out of authority and the "
                    "offset is larger than it can absorb"
                    % (label, RATE_MAX_Q16, rail, len(mags),
                       q16_to_ppm(RATE_MAX_Q16)))
            # Median, not mean, for the same reason the check above is
            # per-sample: the startup transient is a real part of the window
            # but not a description of the steady state.
            mags.sort()
            offs.sort()
            t_med = mags[len(mags) // 2]
            o_med = offs[len(offs) // 2]
            if t_med == 0:
                continue
            frac = float(t_med) / RATE_MAX_Q16
            max_frac = max(max_frac, frac)
            # The ppm figure is deliberately not repeated here: CHECK 4 is
            # the measurement of it. What is new below is what that offset
            # COSTS -- the fraction of the corrector's authority it uses and
            # the ring headroom it consumes.
            notes.append("%s using %.0f%% of authority (of +-%.0f ppm) at the "
                         "median, standing offset %d frames of the %d "
                         "available before empty; %d/%d live samples exact"
                         % (label, frac * 100, q16_to_ppm(RATE_MAX_Q16),
                            o_med, target, len(mags) - len(mism), len(mags)))
        emit("Standing offset and headroom", not problems,
             ("; ".join(notes) if notes else "no sustained tilt")
             + ("" if not problems else "  PROBLEM: " + "; ".join(problems)))
        # The reasoning below only changes what you do when the fraction is
        # large. At a normal ~12% it is five lines of theory attached to a
        # PASS, so it prints from 40% up and stays in docs/CLOCKING.md
        # otherwise.
        if not problems and notes and max_frac >= 0.40:
            print("               A proportional-only controller trades a "
                  "standing offset for stability, and")
            print("               that offset is consumed ring headroom. At "
                  "this fraction of authority, fix")
            print("               the clock rather than widening the clamp -- "
                  "see docs/CLOCKING.md.")

    # --- CHECK 6: stream health ----------------------------------------
    d_under = ((b["pb_underrun"] + b["rec_underrun"])
               - (a["pb_underrun"] + a["rec_underrun"]))
    d_over  = b["pb_overrun"] - a["pb_overrun"]

    # pb_min_avail_seen is initialised to 0xFFFFFFFF and only updated
    # AFTER the pre-fill gate. Still-sentinel means the ring never primed:
    # no USB playback packet ever arrived. Zero underruns in that state is
    # not health, and reporting PASS would be actively misleading.
    if b["pb_min"] == 0xFFFFFFFF:
        emit("Stream health", None,
             "NOTHING STREAMED on the playback side. pb_min_avail_seen is "
             "still its 0xFFFFFFFF sentinel, so the ring never reached "
             "PB_TARGET and AudioIF_PopSamples never passed the pre-fill "
             "gate. The zero underrun count reflects an idle stream, not a "
             "healthy one. Start playback BEFORE launching this script.")
    else:
        emit("Stream health", d_under == 0 and d_over == 0,
             "underruns +%d (pb %d / rec %d absolute), overruns +%d. "
             "rec_avail band %d..%d, pb_avail band %d..%d."
             % (d_under, b["pb_underrun"], b["rec_underrun"], d_over,
                b["rec_min"], b["rec_max"], b["pb_min"], b["pb_max"]))

    # There used to be a CHECK 7 here printing the directional counters
    # (rec_up/rec_down, pb_up/pb_down). It was removed as pure duplication.
    # The firmware increments those once per corrector invocation whenever
    # the tilt is non-zero, so over a steady window they simply equal the
    # number of invocations -- 161005 out of 161005 in a typical run. The
    # only bit of information in them is the SIGN of the tilt, which CHECK 4
    # already prints as a signed number with a magnitude attached. They are
    # a leftover from the sigma-delta corrector, where they counted discrete
    # one-frame corrections and were a real measurement; the fractional
    # reader emits no discontinuities, so there is nothing left to count.
    #
    # The counters themselves stay in the firmware (0x41 slots 2 and 3,
    # documented there as kept for backward compatibility) -- that is a
    # binary change and this is only about what gets printed.

    print("=" * 100)
    hard = [v for v in verdicts if v is not None]
    if not all(hard):
        print("RESULT: at least one check FAILED -- do not record the transport "
              "layer as validated.")
        sys.exit(1)
    if any(v is None for v in verdicts):
        print("RESULT: no failures, but some checks were INCONCLUSIVE. An "
              "inconclusive run is not a pass -- rerun so those branches fire.")
        sys.exit(2)
    print("RESULT: all checks passed for this window, on this board, at this "
          "temperature. Repeat warm before generalising.")


if __name__ == "__main__":
    main()
