#!/usr/bin/env bash
set -euo pipefail

LOG_DIR="${HOME}"
LOG_FILE="${LOG_DIR}/realsense_install_$(date +%F_%H%M%S).log"
exec > >(tee -a "${LOG_FILE}") 2>&1

if [[ "${EUID}" -ne 0 ]]; then
    echo "Please run as root:"
    echo "  sudo bash $0"
    exit 1
fi

if [[ ! -f /etc/os-release ]]; then
    echo "Cannot detect OS release information."
    exit 1
fi

. /etc/os-release

echo "========== BASIC =========="
date
uname -a
echo "ID=${ID:-unknown}"
echo "VERSION_ID=${VERSION_ID:-unknown}"
echo "LOG_FILE=${LOG_FILE}"
echo

echo "========== INSTALL BASE PACKAGES =========="
export DEBIAN_FRONTEND=noninteractive
apt-get update
apt-get install -y \
    ca-certificates \
    curl \
    gnupg \
    lsb-release \
    software-properties-common \
    usbutils \
    v4l-utils
echo

echo "========== CHECK REALSENSE APT SOURCE =========="
if apt-cache policy librealsense2-utils | grep -q "Candidate:" \
    && ! apt-cache policy librealsense2-utils | grep -q "Candidate: (none)"; then
    echo "Intel RealSense apt source is already configured."
else
    echo "Intel RealSense apt source not found, adding it now..."
    apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCD
    add-apt-repository -y "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main"
    apt-get update
fi
echo

echo "========== APT POLICY =========="
apt-cache policy librealsense2-utils || true
echo

echo "========== INSTALL REALSENSE PACKAGES =========="
apt-get update
apt-get install -y \
    librealsense2-utils \
    librealsense2-dev
echo

echo "========== TOOL CHECK =========="
for cmd in rs-enumerate-devices realsense-viewer v4l2-ctl lsusb; do
    if command -v "${cmd}" >/dev/null 2>&1; then
        echo "[FOUND] ${cmd} -> $(command -v "${cmd}")"
    else
        echo "[MISS ] ${cmd}"
    fi
done
echo

echo "========== DONE =========="
echo "Install log saved to: ${LOG_FILE}"
echo "Next step:"
echo "  bash tools/realsense_verify.sh"
