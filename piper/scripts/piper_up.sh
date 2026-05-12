#!/usr/bin/env bash
# Bring the AgileX Piper CAN interface (can_piper, via candleLight gs_usb)
# up at 1 Mbps. Called by uav-proc-piper.service as ExecStartPre and safe
# to invoke by hand:
#
#     sudo /opt/uav_robot/piper/scripts/piper_up.sh
#
# Idempotent — if the interface is already up at the right bitrate this is
# a no-op. Tries gs_usb modprobe in case the kernel module wasn't loaded
# automatically (happens on fresh boots before /etc/modules-load.d kicks
# in, or when the adapter was hot-plugged on a kernel without auto-bind).
set -euo pipefail

IFACE="${UAV_PIPER_CAN_IFACE:-can_piper}"
BITRATE="${UAV_PIPER_CAN_BITRATE:-1000000}"

if ! lsmod 2>/dev/null | grep -q '^gs_usb'; then
    modprobe gs_usb 2>/dev/null || true
fi

# Give udev a moment to rename the netdev if the adapter just appeared.
for _ in 1 2 3 4 5; do
    [[ -d /sys/class/net/${IFACE} ]] && break
    sleep 0.2
done

if [[ ! -d /sys/class/net/${IFACE} ]]; then
    echo "piper_up.sh: ${IFACE} not present — is the candleLight adapter plugged in and gs_usb loaded?" >&2
    exit 1
fi

# Re-applying bitrate on an already-UP iface fails, so flap it.
ip link set "${IFACE}" down 2>/dev/null || true
ip link set "${IFACE}" type can bitrate "${BITRATE}" restart-ms 100
ip link set "${IFACE}" up txqueuelen 1000
echo "piper_up.sh: ${IFACE} up @ ${BITRATE} bps"
