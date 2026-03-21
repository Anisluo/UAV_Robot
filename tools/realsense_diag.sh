#!/usr/bin/env bash
set -e

LOG=~/realsense_diag_$(date +%F_%H%M%S).log
exec > >(tee -a "$LOG") 2>&1

echo "========== BASIC =========="
date
uname -a
echo
if [ -f /etc/os-release ]; then
    cat /etc/os-release
fi

echo
echo "========== USB DEVICES =========="
lsusb || true
echo
echo "---------- Intel / RealSense related ----------"
lsusb | grep -Ei 'intel|8086|realsense' || true

echo
echo "========== USB TOPOLOGY =========="
lsusb -t || true

echo
echo "========== VIDEO NODES =========="
ls -l /dev/video* 2>/dev/null || echo "No /dev/video* found"

echo
echo "========== V4L2 =========="
if command -v v4l2-ctl >/dev/null 2>&1; then
    v4l2-ctl --list-devices || true
    echo
    for dev in /dev/video*; do
        [ -e "$dev" ] || continue
        echo "----- $dev -----"
        v4l2-ctl -d "$dev" --all 2>/dev/null | sed -n '1,80p' || true
        echo
    done
else
    echo "v4l2-ctl not found"
fi

echo
echo "========== REALSENSE TOOLS =========="
for cmd in rs-enumerate-devices realsense-viewer rs-fw-update; do
    if command -v "$cmd" >/dev/null 2>&1; then
        echo "[FOUND] $cmd -> $(command -v $cmd)"
    else
        echo "[MISS ] $cmd"
    fi
done

echo
echo "========== KERNEL LOG (recent USB/UVC) =========="
dmesg | grep -Ei 'usb|uvc|video|intel|realsense|firmware' | tail -n 200 || true

echo
echo "========== DONE =========="
echo "LOG saved to: $LOG"