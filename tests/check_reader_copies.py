#!/usr/bin/env python3
"""
check_reader_copies.py -- the reader the tests measure must be the reader
that ships.

WHY THIS EXISTS

fd_read_frame() lives in usbd_audio_if.c, which includes libDaisy and STM32
headers and therefore does not build on a desktop. tests/interp and
tests/am_probe cannot include it, so each holds a copy of its source text.
That is the normal way to test firmware code on a host, and it has one
failure mode that matters: edit the reader in the firmware, forget the two
copies, and CI stays green while the tests go on proving properties of the
old version. The documented interpolation and AM figures would then describe
code that is no longer shipped -- and nothing anywhere would say so.

fd_sinc_table.h already has a reproducibility check in CI for exactly this
reason. This applies the same idea to the code beside it.

WHAT IT COMPARES

The body of fd_read_frame(), found by name and delimited by brace matching,
with comments removed and whitespace collapsed, compared as a token stream.
Formatting is deliberately ignored: the copies are already wrapped
differently from the firmware (one-line for loop, merged pointer
declarations) and that difference is real but harmless. Requiring byte
equality would make the build fail on a reformat, which trains people to
ignore it -- the worst outcome for a check like this.

A missing or unparseable function is a failure, not a skip. A check that
quietly passes when it cannot find its subject is worse than no check.

    python3 tests/check_reader_copies.py     # from the repository root
"""
import re
import sys
from pathlib import Path

FUNC = "fd_read_frame"

REFERENCE = "usbd_audio_if.c"
COPIES = [
    "tests/interp/interp_check.c",
    "tests/am_probe/am_probe.c",
]


def extract(path):
    """Return the full text of FUNC's definition, or raise."""
    try:
        src = Path(path).read_text(encoding="utf-8")
    except OSError as e:
        raise SystemExit("cannot read %s: %s" % (path, e))

    # Blank out comments first, keeping length so offsets stay valid. Without
    # this the search matches the file-header comment in am_probe.c, which
    # names the function, and the check silently compares the wrong text.
    blind = re.sub(r"/\*.*?\*/", lambda m: " " * len(m.group()), src, flags=re.S)
    blind = re.sub(r"//[^\n]*", lambda m: " " * len(m.group()), blind)

    # The definition, not a call: the name must be followed by an argument
    # list and then a brace.
    m = re.search(r"^[^\n]*\b" + FUNC + r"\s*\([^;{]*\)\s*\{", blind, re.M | re.S)
    if not m:
        raise SystemExit(
            "%s: no definition of %s() found.\n"
            "If it was renamed or moved, update FUNC/REFERENCE/COPIES in this\n"
            "script. Do not delete the check -- that is the failure mode it\n"
            "exists to prevent." % (path, FUNC))

    start = m.start()
    i = blind.index("{", m.start())
    depth = 0
    for j in range(i, len(blind)):
        if blind[j] == "{":
            depth += 1
        elif blind[j] == "}":
            depth -= 1
            if depth == 0:
                return src[start:j + 1]
    raise SystemExit("%s: unbalanced braces in %s()" % (path, FUNC))


def tokens(text):
    """Token stream with comments and all whitespace removed."""
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.S)
    text = re.sub(r"//[^\n]*", "", text)
    return re.findall(r"[A-Za-z_]\w*|\d+\.?\d*[fFuUlL]*|\S", text)


def first_difference(a, b):
    for n, (x, y) in enumerate(zip(a, b)):
        if x != y:
            lo = max(0, n - 6)
            return n, " ".join(a[lo:n + 6]), " ".join(b[lo:n + 6])
    n = min(len(a), len(b))
    longer = a if len(a) > len(b) else b
    return n, " ".join(a[n:n + 12]) if longer is a else "(ends here)", \
              " ".join(b[n:n + 12]) if longer is b else "(ends here)"


def main():
    ref = tokens(extract(REFERENCE))
    print("%s(): %d tokens in %s" % (FUNC, len(ref), REFERENCE))

    fail = False
    for path in COPIES:
        got = tokens(extract(path))
        if got == ref:
            print("  OK    %s  (%d tokens)" % (path, len(got)))
            continue
        fail = True
        n, exp, act = first_difference(ref, got)
        print("  FAIL  %s  (%d tokens, expected %d)" % (path, len(got), len(ref)))
        print("        diverges at token %d" % n)
        print("        firmware: ...%s..." % exp)
        print("        copy:     ...%s..." % act)

    print("\nRESULT: " + ("FAIL" if fail else "PASS"))
    if fail:
        print(
            "\nThe tests are measuring a different reader from the one that\n"
            "ships, so the figures in docs/INTERPOLATION.md no longer describe\n"
            "this firmware. Copy the current body of %s() from %s into the\n"
            "files listed above." % (FUNC, REFERENCE))
    return 1 if fail else 0


if __name__ == "__main__":
    sys.exit(main())
