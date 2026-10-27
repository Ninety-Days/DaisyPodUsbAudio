#!/usr/bin/env python3
"""
soak_run.py -- repeat-run and thermal-soak harness for DaisyPodUsbAudio.

WHY THIS EXISTS, AND WHY IT IS A WRAPPER

read_transport_diagnostics.py already owns every vendor request, slot number
and Q16 conversion in this project. Reimplementing that here would create a
second place to get slot 10/11 wrong. So this script does not touch USB at
all: it runs that script as a subprocess, echoes its output live, saves the
log, and does the one thing it does not do -- resolve tilt as a function of
TIME.

That gap is the whole point. read_transport_diagnostics.py reports one tilt
mean per run. A 35-minute thermal soak averaged into a single number hides
precisely the drift the soak is meant to expose. This script buckets the
per-second rows, fits a slope, and compares the first bucket against the
last.

USAGE

  Baseline run, board at room temperature:
      python3 soak_run.py --label cold1 --seconds 600

  Thermal soak, with the board warm/enclosed or externally heated:
      python3 soak_run.py --label warm1 --seconds 2100

  A/B between two firmware builds, one run per build:
      python3 soak_run.py --label build_a --seconds 900 --note "what changed in this build"
      python3 soak_run.py --label build_b --seconds 900 --note "what changed in this build"

  A word on reading an A/B like that: one run per arm is rarely enough. Any
  failure that is intermittent rather than systematic needs its rate measured
  before two configurations can be ranked, and at a low per-event rate a
  single clean window is close to uninformative. Use --seconds generously,
  repeat each arm, and treat a lone passing run as an observation rather
  than a result.

  Compare any two completed runs:
      python3 soak_run.py --compare logs/cold1_*.csv logs/warm1_*.csv

INTERPRETING TILT DRIFT

  1 Q16 unit = 15.259 ppm. Observed run-to-run spread on two cold runs was
  0.07 units (rec) and 0.10 units (pb) -- call the noise floor ~1.5 ppm.

  A physical HSE crystal offset should move several ppm to ~15 ppm over a
  20-30 C swing. A fixed divider error cannot move at all. So:

    |delta| >= 3 ppm, monotonic with temperature  -> physical crystal offset
    |delta| <  1.5 ppm across a large temp swing  -> crystal hypothesis in
                                                     trouble; dividers are
                                                     exact, so the offset
                                                     source is then unexplained

  CONFOUND, state it in any writeup: the tilt is a RELATIVE measurement
  against the host's SOF timing. If the host's own crystal warms too, the
  tilt moves for reasons that are not the Daisy's. This test attributes drift
  to the board only insofar as the board is the thing whose temperature you
  actually changed. Log ambient and board temperature by hand -- an IR
  thermometer or a thermistor taped near the crystal is enough, and needs no
  firmware change. Do NOT add a die-temperature vendor request for this
  campaign: modifying the firmware mid-validation means the runs no longer
  describe the build you are validating, and die temperature is a lagging
  proxy for crystal temperature anyway.
"""

# MIT License (see LICENSE at repository root) applies to this file.

import argparse
import csv
import os
import re
import subprocess
import sys
import time
from statistics import mean

Q16_PPM = 1e6 / 65536.0          # 15.2588 ppm per Q16 unit
NOISE_PPM = 1.5                  # measured run-to-run spread, two cold runs

# WHY THIS EXISTS
#
# A 600s run showed a ~17.2s sawtooth in rec_av/pb_av, in lockstep
# with rec_tilt/pb_tilt, repeating identically for 34 consecutive cycles.
# It is the fractional-Q16 reader's own correction rhythm: at a ~58 ppm
# offset, 1e6/58 = ~17240 samples of drift accumulate every ~17.2s of
# playback time, and the interpolator's boundary crossing shows up as a
# step in ring fill level. This is the reader working as designed, not a
# fault -- but it means tilt has a real periodic component,
# and averaging window length matters.
#
# 60s buckets are ~3.5 cycles, not a whole number, so different buckets
# catch different phases of the sawtooth and the phase difference alone
# produced an apparent 3.89 ppm "drift" on a run with none: splitting the
# same run into two 300s halves (17.5 cycles each, phase nearly cancels)
# gave 1.12 ppm, inside the noise floor. Bucket width is therefore locked
# to a whole number of estimated corrector periods rather than to a round
# number of seconds. If the true period differs from the estimate below,
# pass --period to override it once read_sai_clock.py or a longer run
# gives a better number.
EST_CORRECTOR_PERIOD_S = 17.18   # 1e6 / (measured ppm, ~58) -- see above

COLS = ["t", "rec_av", "pb_av", "rec_tilt", "pb_tilt",
        "rec_ppm", "pb_ppm", "crs_ok", "trim", "kicks", "txdev"]

ROW_RE = re.compile(r"^\s*\d+\.\d\s")


def parse_row(line):
    """Return a dict for a data row, or None. The table is fixed-width text
    from read_transport_diagnostics.py; 11 numeric fields, t first."""
    if not ROW_RE.match(line):
        return None
    parts = line.split()
    if len(parts) != len(COLS):
        return None
    try:
        vals = [float(p.replace("+", "")) for p in parts]
    except ValueError:
        return None
    row = dict(zip(COLS, vals))
    for k in ("rec_av", "pb_av", "rec_tilt", "pb_tilt",
              "crs_ok", "trim", "kicks", "txdev"):
        row[k] = int(row[k])
    return row


def linfit(xs, ys):
    """Least-squares slope and intercept. Pure Python: no numpy dependency,
    because this has to run on whatever host is attached to the board."""
    n = len(xs)
    if n < 2:
        return 0.0, (ys[0] if ys else 0.0)
    mx, my = mean(xs), mean(ys)
    sxx = sum((x - mx) ** 2 for x in xs)
    if sxx == 0:
        return 0.0, my
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    b = sxy / sxx
    return b, my - b * mx


def cycle_safe_width(requested, period):
    """Snap a requested bucket width to the nearest whole multiple of the
    corrector period, so every bucket averages the same phase mix and the
    sawtooth cancels identically bucket to bucket. Returns (width, n_cycles,
    warn) -- warn is set if the snap moved the width by more than 10%,
    since that means the requested width was a bad match for this signal
    and the result should be read cautiously."""
    if period <= 0:
        return requested, None, False
    n = max(1, round(requested / period))
    width = n * period
    warn = abs(width - requested) / requested > 0.10
    return width, n, warn


def buckets(rows, width):
    """Group rows into fixed-width time buckets, oldest first."""
    out = []
    if not rows:
        return out
    t0 = rows[0]["t"]
    cur, idx = [], 0
    for r in rows:
        i = int((r["t"] - t0) // width)
        if i != idx and cur:
            out.append((idx, cur))
            cur, idx = [], i
        cur.append(r)
    if cur:
        out.append((idx, cur))
    return out


def summarise(rows, width, label="", period=EST_CORRECTOR_PERIOD_S):
    """Print per-bucket means and the drift verdict. Returns a dict of the
    headline numbers so --compare can reuse them."""
    if len(rows) < 3:
        print("Too few rows (%d) to summarise." % len(rows))
        return None

    snapped, n_cyc, warn = cycle_safe_width(width, period)

    print()
    print("=" * 78)
    print("TILT vs TIME%s  (bucket = %.2fs = %s x %.2fs corrector period)"
          % (("  [" + label + "]") if label else "", snapped,
             n_cyc if n_cyc else "?", period))
    print("=" * 78)
    if warn:
        print("  NOTE: requested %ds did not divide evenly into whole periods;"
              % width)
        print("        snapped to %.2fs. Pass --bucket %.1f directly to silence"
              % (snapped, snapped))
        print("        this, or --period to correct the period estimate.")
    width = snapped
    print("  window        n   rec_tilt    rec_ppm   pb_tilt     pb_ppm"
          "   pb_av_min  kicks  txdev")
    print("-" * 78)

    bs = buckets(rows, width)
    for i, rs in bs:
        rt, pt = mean(r["rec_tilt"] for r in rs), mean(r["pb_tilt"] for r in rs)
        print("%5d-%-5d %5d   %+8.3f  %+8.2f  %+8.3f  %+8.2f  %9d  %5d  %5d"
              % (i * width, (i + 1) * width, len(rs),
                 rt, rt * Q16_PPM, pt, pt * Q16_PPM,
                 min(r["pb_av"] for r in rs),
                 max(r["kicks"] for r in rs),
                 max(r["txdev"] for r in rs)))

    ts = [r["t"] - rows[0]["t"] for r in rows]
    rec_slope, _ = linfit(ts, [r["rec_tilt"] * Q16_PPM for r in rows])
    pb_slope, _ = linfit(ts, [r["pb_tilt"] * Q16_PPM for r in rows])

    first, last = bs[0][1], bs[-1][1]
    d_rec = (mean(r["rec_tilt"] for r in last) -
             mean(r["rec_tilt"] for r in first)) * Q16_PPM
    d_pb = (mean(r["pb_tilt"] for r in last) -
            mean(r["pb_tilt"] for r in first)) * Q16_PPM

    d_kicks = rows[-1]["kicks"] - rows[0]["kicks"]
    d_txdev = rows[-1]["txdev"] - rows[0]["txdev"]

    span_min = (rows[-1]["t"] - rows[0]["t"]) / 60.0

    print("-" * 78)
    print("rec drift  first->last bucket %+.2f ppm    slope %+.3f ppm/min"
          % (d_rec, rec_slope * 60.0))
    print("pb  drift  first->last bucket %+.2f ppm    slope %+.3f ppm/min"
          % (d_pb, pb_slope * 60.0))
    print("           slope x window:  rec %+.2f ppm   pb %+.2f ppm"
          % (rec_slope * 60.0 * span_min, pb_slope * 60.0 * span_min))
    print("           (bucket means are CENTRED, so first->last spans only")
    print("            window-minus-one-bucket and understates total drift by")
    print("            roughly bucket/window. Use slope x window as the estimate;")
    print("            the two agree when the drift is linear.)")
    print("kicks delta in window: %d   txdev delta in window: %d" % (d_kicks, d_txdev))
    print()

    # The two checks that are pass/fail regardless of the thermal question.
    if d_kicks:
        print("  FAIL  in_pipe_kicks GREW by %d during the window. The pipe"
              % d_kicks)
        print("        stalled while streaming -- a real fault, not a startup"
              " artefact.")
    else:
        print("  ok    in_pipe_kicks did not grow in-window (startup kicks are"
              " expected).")
    if d_txdev:
        print("  FAIL  tx_size_deviations GREW by %d -- the safety clamp fired,"
              % d_txdev)
        print("        so a short IN packet went out. Must stay 0.")
    else:
        print("  ok    tx_size_deviations stayed 0: every IN packet was full"
              " length.")

    # The thermal question. Deliberately does not claim a cause: this script
    # cannot see temperature, only drift.
    worst = max(abs(d_rec), abs(d_pb))
    print()
    if worst >= 3.0:
        print("  DRIFT %.2f ppm, above the ~%.1f ppm noise floor. If the board's"
              % (worst, NOISE_PPM))
        print("        temperature actually changed across this window, that is"
              " consistent")
        print("        with a physical crystal offset and NOT with a")
        print("        fixed divider error, which cannot move at all.")
    elif worst >= NOISE_PPM:
        print("  DRIFT %.2f ppm -- marginal, between the noise floor and the 3 ppm"
              % worst)
        print("        threshold. Not evidence either way. Repeat with a larger"
              " temperature")
        print("        swing before concluding anything.")
    else:
        print("  DRIFT %.2f ppm, at or below the ~%.1f ppm noise floor: no"
              % (worst, NOISE_PPM))
        print("        movement resolved. If the temperature swing across this"
              " window was")
        print("        large, that argues AGAINST the crystal explanation and the"
              " offset")
        print("        source becomes unexplained -- the dividers are exact.")

    return {"d_rec": d_rec, "d_pb": d_pb,
            "rec_slope_min": rec_slope * 60.0, "pb_slope_min": pb_slope * 60.0,
            "rec_ppm": mean(r["rec_tilt"] for r in rows) * Q16_PPM,
            "pb_ppm": mean(r["pb_tilt"] for r in rows) * Q16_PPM,
            "pb_av_min": min(r["pb_av"] for r in rows),
            "pb_av_mean": mean(r["pb_av"] for r in rows),
            "rec_av_min": min(r["rec_av"] for r in rows),
            "d_kicks": d_kicks, "d_txdev": d_txdev, "n": len(rows)}


def load_csv(path):
    rows = []
    with open(path, newline="") as f:
        for d in csv.DictReader(f):
            if d.get("t") in (None, ""):
                continue
            r = {k: float(d[k]) for k in COLS if k in d}
            for k in ("rec_av", "pb_av", "rec_tilt", "pb_tilt",
                      "crs_ok", "trim", "kicks", "txdev"):
                if k in r:
                    r[k] = int(r[k])
            rows.append(r)
    return rows


def compare(path_a, path_b, width, period):
    a, b = load_csv(path_a), load_csv(path_b)
    ra = summarise(a, width, os.path.basename(path_a), period)
    rb = summarise(b, width, os.path.basename(path_b), period)
    if not (ra and rb):
        return 1

    print()
    print("=" * 78)
    print("A/B COMPARISON")
    print("=" * 78)
    print("  A = %s  (%d rows)" % (os.path.basename(path_a), ra["n"]))
    print("  B = %s  (%d rows)" % (os.path.basename(path_b), rb["n"]))
    print()
    print("  metric              A          B        B-A")
    print("  " + "-" * 46)
    for key, name in (("rec_ppm", "rec offset ppm"), ("pb_ppm", "pb offset ppm"),
                      ("d_rec", "rec drift ppm"), ("d_pb", "pb drift ppm"),
                      ("pb_av_mean", "pb_avail mean"), ("pb_av_min", "pb_avail MIN"),
                      ("rec_av_min", "rec_avail MIN")):
        print("  %-18s %+8.2f  %+8.2f  %+8.2f"
              % (name, ra[key], rb[key], rb[key] - ra[key]))
    print("  %-18s %8d  %8d" % ("kicks delta", ra["d_kicks"], rb["d_kicks"]))
    print("  %-18s %8d  %8d" % ("txdev delta", ra["d_txdev"], rb["d_txdev"]))

    print()
    print("READING THIS FOR THE RxFIFO A/B")
    print("  The failure mode of an undersized Rx FIFO is not a crash: it is")
    print("  occasionally dropped ISO OUT packets under load. Those show up as")
    print("  the playback ring draining, so pb_avail MIN and the low tail of")
    print("  pb_avail are the sensitive metrics -- more sensitive than waiting")
    print("  for a discrete overrun event, which may never fire in a 15-minute")
    print("  window.")
    print()
    print("  If pb_avail MIN and mean are within noise between the two builds,")
    print("  the honest conclusion is 'no difference detected at this load',")
    print("  NOT '0xC0 is sufficient'. To claim sufficiency you have to push the")
    print("  load until something degrades and show which build degrades first.")
    return 0


def main():
    ap = argparse.ArgumentParser(
        description="Repeat-run / thermal-soak harness wrapping "
                    "read_transport_diagnostics.py")
    ap.add_argument("--label", default="run",
                    help="short name for this run, used in filenames")
    ap.add_argument("--seconds", type=float, default=600.0,
                    help="window length; 600 for repeat runs, ~2100 to match "
                         "a long run")
    ap.add_argument("--interval", type=float, default=1.0)
    ap.add_argument("--bucket", type=float, default=60.0,
                    help="tilt averaging bucket, seconds -- snapped to a "
                         "whole number of corrector periods, see --period")
    ap.add_argument("--period", type=float, default=EST_CORRECTOR_PERIOD_S,
                    help="corrector sawtooth period, seconds. Default is "
                         "derived from a ~58 ppm measured offset (-> ~17.18s). "
                         "Recompute as 1e6/ppm/1000 for your own measured "
                         "offset, or pass 0 to disable snapping entirely and "
                         "use raw --bucket.")
    ap.add_argument("--outdir", default="logs")
    ap.add_argument("--script", default="read_transport_diagnostics.py")
    ap.add_argument("--note", default="",
                    help="conditions: board temperature, enclosure, what else "
                         "is streaming. Recorded in the log header.")
    ap.add_argument("--compare", nargs=2, metavar=("A.csv", "B.csv"),
                    help="compare two finished runs instead of collecting")
    args = ap.parse_args()

    if args.compare:
        sys.exit(compare(args.compare[0], args.compare[1], args.bucket, args.period))

    if not os.path.exists(args.script):
        sys.exit("Cannot find %s -- run this from the firmware directory."
                 % args.script)

    os.makedirs(args.outdir, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    base = os.path.join(args.outdir, "%s_%s" % (args.label, stamp))
    log_path, csv_path = base + ".log", base + ".csv"

    header = [
        "# soak_run.py  label=%s" % args.label,
        "# started    %s" % time.strftime("%Y-%m-%d %H:%M:%S"),
        "# seconds=%.0f interval=%.2f bucket=%.0f" % (args.seconds, args.interval, args.bucket),
        "# note: %s" % (args.note or "(none -- record board temperature and "
                                     "what else was streaming!)"),
        "",
    ]
    print("\n".join(header))

    cmd = [sys.executable, args.script,
           "--seconds", str(args.seconds), "--interval", str(args.interval)]
    rows = []
    with open(log_path, "w") as lf:
        lf.write("\n".join(header) + "\n")
        try:
            p = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                                 stderr=subprocess.STDOUT, text=True,
                                 bufsize=1)
        except OSError as e:
            sys.exit("Could not start %s: %s" % (args.script, e))
        try:
            for line in p.stdout:
                sys.stdout.write(line)
                sys.stdout.flush()
                lf.write(line)
                r = parse_row(line)
                if r:
                    rows.append(r)
        except KeyboardInterrupt:
            p.terminate()
            print("\n(interrupted -- summarising what was collected)")
        p.wait()

    if rows:
        with open(csv_path, "w", newline="") as cf:
            w = csv.DictWriter(cf, fieldnames=COLS)
            w.writeheader()
            w.writerows(rows)

    res = summarise(rows, args.bucket, args.label, args.period)
    print()
    print("log: %s" % log_path)
    if rows:
        print("csv: %s   (%d rows)" % (csv_path, len(rows)))
    if not args.note:
        print()
        print("NOTE WAS EMPTY. Write the board temperature, ambient, enclosure")
        print("state and what else was streaming into the log now, by hand --")
        print("a soak run with no recorded conditions cannot be compared to")
        print("anything later.")
    sys.exit(0 if res and not res["d_kicks"] and not res["d_txdev"] else 1)


if __name__ == "__main__":
    main()
