#!/usr/bin/env bash
# Live view of the robot's front camera on the Mac.
#
#   camera (ISP mainpath /dev/video0, NV12) → mpi_enc_test (Rockchip hardware H.264 encoder)
#     → stdout over ssh → ffplay on the Mac
#
# Nothing is installed or written on the robot; everything runs from the ssh command line.
# Closing the ffplay window drops the ssh channel, and the robot-side pipeline dies on SIGPIPE.
#
# Usage:  scripts/robot_camera.sh [WIDTH HEIGHT]        (default 640 480)
#         scripts/robot_camera.sh --record out.h264     (save the stream instead of viewing)
# Needs: ffmpeg/ffplay on the Mac (brew install ffmpeg). Robot address from scripts/config.py.
set -euo pipefail

cd "$(dirname "$0")"
ROBOT=$(python3 -c 'from config import ROBOT_SSH; print(ROBOT_SSH)')

RECORD=""
if [[ "${1:-}" == "--record" ]]; then RECORD="${2:?output file}"; shift 2; fi
W=${1:-640}
H=${2:-480}

# mpi_enc_test ignores write errors, so when the ssh client goes away the pipeline would keep
# running and hold /dev/video0. The watchdog loop polls our ssh session process ($PPID) and kills
# the whole process group (kill 0) once it's gone.
REMOTE="v4l2-ctl -d /dev/video0 --set-fmt-video=width=$W,height=$H,pixelformat=NV12 >/dev/null 2>&1;
{ v4l2-ctl -d /dev/video0 --stream-mmap=4 --stream-count=0 --stream-to=- 2>/dev/null \
  | mpi_enc_test -i /dev/stdin -o /dev/stdout -w $W -h $H -f 0 -t 7 -n 0 2>/dev/null; } &
while kill -0 \$PPID 2>/dev/null && [ \$(cut -d' ' -f4 /proc/\$\$/stat) = \$PPID ]; do sleep 1; done
kill 0"

if [[ -n "$RECORD" ]]; then
  echo "Recording to $RECORD — Ctrl-C to stop"
  ssh -o ServerAliveInterval=5 "$ROBOT" "$REMOTE" > "$RECORD"
else
  ssh -o ServerAliveInterval=5 "$ROBOT" "$REMOTE" \
    | ffplay -hide_banner -loglevel error -fflags nobuffer -flags low_delay -framedrop \
             -probesize 32 -analyzeduration 0 -f h264 -window_title "Deebot camera" -
fi
