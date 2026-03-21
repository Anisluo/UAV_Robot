#!/usr/bin/env bash
set -euo pipefail

LOG_DIR="${HOME}"
LOG_FILE="${HOME}/realsense_verify_$(date +%F_%H%M%S).log"
exec > >(tee -a "${LOG_FILE}") 2>&1

echo "========== BASIC =========="
date
uname -a
echo "LOG_FILE=${LOG_FILE}"
echo

echo "========== TOOL CHECK =========="
for cmd in lsusb v4l2-ctl rs-enumerate-devices; do
    if ! command -v "${cmd}" >/dev/null 2>&1; then
        echo "[MISS ] ${cmd}"
    else
        echo "[FOUND] ${cmd} -> $(command -v "${cmd}")"
    fi
done
echo

echo "========== USB CHECK =========="
lsusb | grep -Ei 'intel|8086|realsense' || echo "No RealSense USB device found"
echo
lsusb -t || true
echo

echo "========== VIDEO NODE CHECK =========="
ls -l /dev/video* 2>/dev/null || echo "No /dev/video* found"
echo

echo "========== V4L2 CHECK =========="
if command -v v4l2-ctl >/dev/null 2>&1; then
    v4l2-ctl --list-devices || true
else
    echo "v4l2-ctl not found"
fi
echo

echo "========== REALSENSE ENUMERATION =========="
if command -v rs-enumerate-devices >/dev/null 2>&1; then
    rs-enumerate-devices || true
else
    echo "rs-enumerate-devices not found"
fi
echo

echo "========== VIEWER NOTE =========="
echo "In SSH or headless environments, realsense-viewer errors can usually be ignored."
echo "The following messages do not affect basic camera verification:"
echo "  - No valid configuration file found"
echo "  - cudaGetDeviceCount failed"
echo "  - failed to open rockchip_dri.so"
echo "Use rs-enumerate-devices as the primary verification command on RK3588."
echo

echo "========== FIRMWARE NOTE =========="
echo "If current firmware is older than the recommended firmware,"
echo "upgrade is optional and does not block basic camera use."
echo

echo "========== RECENT KERNEL LOG =========="
dmesg | grep -Ei 'usb|uvc|video|intel|realsense|firmware' | tail -n 120 || true
echo

echo "========== RESULT GUIDE =========="
echo "If rs-enumerate-devices shows Name, Serial Number, and Stream Profiles, the SDK stack is working."
echo "If lsusb shows the device but rs-enumerate-devices is missing or fails, check librealsense installation and apt source configuration."
echo "If no /dev/video* nodes appear, check USB cable, power, and uvcvideo binding."
echo

echo "========== DONE =========="
echo "Verify log saved to: ${LOG_FILE}"
