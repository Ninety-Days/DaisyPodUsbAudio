#!/usr/bin/env bash
# Numeric check of the fractional-delay reader in usbd_audio_if.c.
#
# Rebuilds fd_read_frame() standalone (the copy in interp_check.c must be
# kept in step with the firmware) and verifies, for both interpolators:
#   - worst-case output error against an ideal fractional delay
#   - channels do not leak into each other
#   - reads that wrap the ring match an unwrapped rotation exactly
#
# Run from this directory: bash check.sh
#
# MIT License (see LICENSE at repository root) applies to this file.
set -e
cd "$(dirname "$0")"
cc -O2 -Wall -I../.. -DFD_USE_SINC=1 -o /tmp/interp_sinc interp_check.c -lm
cc -O2 -Wall -I../.. -DFD_USE_SINC=0 -o /tmp/interp_cr   interp_check.c -lm
echo "########## FD_USE_SINC=1 (polyphase windowed-sinc) ##########"
/tmp/interp_sinc
echo
echo "########## FD_USE_SINC=0 (Catmull-Rom) ##########"
/tmp/interp_cr
