#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PROJECT_ROOT=$(cd "${SCRIPT_DIR}/.." && pwd)
SYSTEMD_SRC_DIR="${PROJECT_ROOT}/systemd"
SYSTEMD_DST_DIR="/etc/systemd/system"
ENV_DST_FILE="/etc/default/uav_robot"
PACKAGES_DIR="${PROJECT_ROOT}/packages"

# Arm backend choice. "piper" = gen-2 (proc_piper drives AgileX Piper).
#                     "legacy" = gen-1 (proc_arm + proc_gripper drive ZDT arm).
# Override via --backend flag or UAV_ARM_BACKEND env var.
BACKEND="${UAV_ARM_BACKEND:-piper}"

# Shared services (backend-independent)
SHARED_SERVICES="uav_robotd,proc_realsense,proc_npu,proc_gateway,proc_car,proc_airport,proc_grasp,battery_tracker,face_tracker"

# Backend-dependent arm services
ARM_SERVICES_PIPER="proc_piper"
ARM_SERVICES_LEGACY="proc_gripper,proc_arm"

# DEFAULT_SERVICES is the union of shared + backend's arm slice, computed below.
SERVICES=""
ORIGINAL_ARGS=("$@")

usage() {
    cat <<'EOF'
Usage:
  sudo ./tools/install_autostart.sh [--services name1,name2,...] [--backend piper|legacy] [--skip-deps]

Options:
  --backend    Arm backend: 'piper' (gen-2 default) or 'legacy' (gen-1).
               'piper'  → starts proc_piper (Python, AgileX Piper SDK).
               'legacy' → starts proc_arm + proc_gripper (ZDT CAN + servo UART).
               Can also be set via UAV_ARM_BACKEND env var.
  --services   Comma-separated service list (overrides backend's defaults).
               Available values:
                 uav_robotd, proc_realsense, proc_npu, proc_gateway,
                 proc_car, proc_piper, proc_gripper, proc_arm,
                 proc_airport, proc_grasp, battery_tracker, face_tracker
  --skip-deps  Skip system dependency setup (librknnrt.so, udev rules,
               cv2/numpy pip install). Useful for incremental rebuilds.
  -h, --help   Show this help message.

What this script does (one-shot deploy):
  1. Install librealsense2 udev rules from packages/udev/*.rules (if present)
  2. Upgrade /usr/lib/librknnrt.so from packages/librknnrt.so (if present and newer)
  3. Install Python deps cv2 + numpy from packages/*.whl (if present)
  4. Wipe stale build/ + *.o (kills cross-arch x86 leftovers)
  5. Build each C++ service (Python sidecars skip this step)
  6. Render + install + enable + restart each systemd unit
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    usage
    exit 0
fi

SKIP_DEPS=0
USER_SET_SERVICES=0
while [[ $# -gt 0 ]]; do
    case "$1" in
        --services)
            SERVICES="${2:-}"
            USER_SET_SERVICES=1
            shift 2
            ;;
        --backend)
            BACKEND="${2:-}"
            shift 2
            ;;
        --skip-deps)
            SKIP_DEPS=1
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "Unknown argument: $1" >&2
            usage >&2
            exit 1
            ;;
    esac
done

# Validate backend choice
case "${BACKEND}" in
    piper|legacy) ;;
    *) echo "Unknown --backend: ${BACKEND} (expected piper|legacy)" >&2; exit 1 ;;
esac

# Compute DEFAULT_SERVICES from backend (unless --services overrode it)
if [[ ${USER_SET_SERVICES} -eq 0 ]]; then
    case "${BACKEND}" in
        piper)  SERVICES="${SHARED_SERVICES},${ARM_SERVICES_PIPER}" ;;
        legacy) SERVICES="${SHARED_SERVICES},${ARM_SERVICES_LEGACY}" ;;
    esac
fi
echo "Arm backend: ${BACKEND}"

# Disable services from the *opposite* backend so they don't fight us
# (only when --services was NOT user-provided, since the user might want
# to mix-and-match explicitly).
if [[ ${USER_SET_SERVICES} -eq 0 ]]; then
    case "${BACKEND}" in
        piper)
            for stale in uav-proc-arm.service uav-proc-gripper.service; do
                if systemctl is-enabled --quiet "${stale}" 2>/dev/null; then
                    echo "Disabling legacy ${stale}"
                    systemctl disable --now "${stale}" 2>/dev/null || true
                fi
            done
            ;;
        legacy)
            if systemctl is-enabled --quiet uav-proc-piper.service 2>/dev/null; then
                echo "Disabling gen-2 uav-proc-piper.service"
                systemctl disable --now uav-proc-piper.service 2>/dev/null || true
            fi
            ;;
    esac
fi

if [[ ${EUID} -ne 0 ]]; then
    exec sudo "$0" "${ORIGINAL_ARGS[@]}"
fi

if [[ ! -d "${SYSTEMD_SRC_DIR}" ]]; then
    echo "systemd template directory not found: ${SYSTEMD_SRC_DIR}" >&2
    exit 1
fi

# C++ services need a build command; Python sidecars don't.
declare -A BUILD_COMMANDS=(
    [uav_robotd]="make dirs build/bin/uav_robotd"
    [proc_realsense]="make -C proc_realsense"
    [proc_npu]="make -C proc_npu"
    [proc_gateway]="make -C proc_gateway"
    [proc_car]="make -C proc_car"
    [proc_gripper]="make -C proc_gripper"
    [proc_arm]="make -C proc_arm"
    [proc_piper]=""
    [proc_airport]="make -C proc_airport"
    [proc_grasp]="make -C proc_grasp"
    [battery_tracker]=""
    [face_tracker]=""
)

declare -A UNIT_NAMES=(
    [uav_robotd]="uav-robotd.service"
    [proc_realsense]="uav-proc-realsense.service"
    [proc_npu]="uav-proc-npu.service"
    [proc_gateway]="uav-proc-gateway.service"
    [proc_car]="uav-proc-car.service"
    [proc_gripper]="uav-proc-gripper.service"
    [proc_arm]="uav-proc-arm.service"
    [proc_piper]="uav-proc-piper.service"
    [proc_airport]="uav-proc-airport.service"
    [proc_grasp]="uav-proc-grasp.service"
    [battery_tracker]="uav-battery-tracker.service"
    [face_tracker]="uav-face-tracker.service"
)

IFS=',' read -r -a REQUESTED_SERVICES <<< "${SERVICES}"

normalize_service_name() {
    local name
    name=$(echo "$1" | tr -d '[:space:]')
    case "${name}" in
        uav_robotd|proc_realsense|proc_npu|proc_gateway|proc_car|proc_gripper|proc_arm|proc_piper|proc_airport|proc_grasp|battery_tracker|face_tracker)
            printf '%s\n' "${name}"
            ;;
        *)
            echo "Unsupported service name: ${name}" >&2
            exit 1
            ;;
    esac
}

render_unit() {
    local src_file=$1
    local dst_file=$2
    sed "s|@PROJECT_ROOT@|${PROJECT_ROOT}|g" "${src_file}" > "${dst_file}"
}

echo "Project root: ${PROJECT_ROOT}"
echo "Requested services: ${SERVICES}"

# ── Step 1: env file ──────────────────────────────────────────────────────
mkdir -p /etc/default
if [[ ! -f "${ENV_DST_FILE}" ]]; then
    install -m 0644 "${SYSTEMD_SRC_DIR}/uav_robot.env.example" "${ENV_DST_FILE}"
    echo "Created default environment file: ${ENV_DST_FILE}"
else
    echo "Keeping existing environment file: ${ENV_DST_FILE}"
fi

# ── Step 2: system + Python dependencies (idempotent) ────────────────────
if [[ ${SKIP_DEPS} -eq 0 ]]; then
    # librealsense2 udev rules — without these, librealsense can't open
    # the camera reliably; symptom = "No device connected" or random
    # USB stalls on D435.
    if [[ -d "${PACKAGES_DIR}/udev" ]]; then
        echo "Installing librealsense2 udev rules ..."
        for rule in "${PACKAGES_DIR}/udev/"*.rules; do
            [[ -f "${rule}" ]] || continue
            install -m 0644 "${rule}" "/lib/udev/rules.d/$(basename "${rule}")"
            echo "  -> /lib/udev/rules.d/$(basename "${rule}")"
        done
        udevadm control --reload-rules 2>/dev/null || true
        udevadm trigger 2>/dev/null || true
    fi

    # librknnrt.so upgrade — board may have v1.5.0 which doesn't load
    # YOLOv8 models exported with newer rknn-toolkit2 (model version 6).
    if [[ -f "${PACKAGES_DIR}/librknnrt.so" ]]; then
        local_size=$(stat -c%s /usr/lib/librknnrt.so 2>/dev/null || echo 0)
        bundled_size=$(stat -c%s "${PACKAGES_DIR}/librknnrt.so")
        if [[ "${local_size}" != "${bundled_size}" ]]; then
            echo "Upgrading /usr/lib/librknnrt.so (was ${local_size}B, bundled ${bundled_size}B) ..."
            [[ -f /usr/lib/librknnrt.so ]] && cp -a /usr/lib/librknnrt.so "/usr/lib/librknnrt.so.bak.$(date +%s)"
            install -m 0644 "${PACKAGES_DIR}/librknnrt.so" /usr/lib/librknnrt.so
            ldconfig 2>/dev/null || true
        else
            echo "librknnrt.so already at bundled version."
        fi
    fi

    # Python deps for the CV sidecars (battery_tracker.py, face_tracker.py).
    # Use --no-index so we don't need network on the target.
    if compgen -G "${PACKAGES_DIR}/*.whl" > /dev/null; then
        if ! python3 -c "import cv2, numpy" 2>/dev/null; then
            echo "Installing Python deps (cv2, numpy) from ${PACKAGES_DIR} ..."
            pip3 install --no-index --find-links="${PACKAGES_DIR}" \
                opencv-python-headless numpy 2>&1 | tail -5
        else
            echo "Python deps cv2 + numpy already present."
        fi
    fi
fi

# ── Step 3: wipe stale build artifacts ───────────────────────────────────
# Cross-arch leftovers (e.g. x86 .o files copied from a dev machine) cause
# linker errors like "Relocations in generic ELF (EM: 62)". Always start
# from a clean slate so the build target's compiler emits everything fresh.
echo "Cleaning stale build artifacts ..."
find "${PROJECT_ROOT}" -type d -name build -exec rm -rf {} + 2>/dev/null || true
find "${PROJECT_ROOT}" -name "*.o" -delete 2>/dev/null || true

# ── Step 4: build + install each service ─────────────────────────────────
for raw_service in "${REQUESTED_SERVICES[@]}"; do
    service=$(normalize_service_name "${raw_service}")
    build_cmd="${BUILD_COMMANDS[${service}]}"

    if [[ -n "${build_cmd}" ]]; then
        echo "Building ${service} ..."
        (
            cd "${PROJECT_ROOT}"
            eval "${build_cmd}"
        )
    else
        echo "Skipping build for ${service} (Python sidecar, no compile)."
    fi

    unit_name="${UNIT_NAMES[${service}]}"
    unit_src="${SYSTEMD_SRC_DIR}/${unit_name}"
    if [[ ! -f "${unit_src}" ]]; then
        echo "  WARN: systemd template missing: ${unit_src} — skipping" >&2
        continue
    fi
    tmp_unit=$(mktemp)
    render_unit "${unit_src}" "${tmp_unit}"
    install -m 0644 "${tmp_unit}" "${SYSTEMD_DST_DIR}/${unit_name}"
    rm -f "${tmp_unit}"

    systemctl daemon-reload
    systemctl enable "${unit_name}"
    systemctl restart "${unit_name}"
    echo "Installed and started ${unit_name}"
done

echo
echo "Autostart configuration completed."
echo "Check status with:"
echo "  systemctl status uav-robotd.service"
echo "  systemctl status uav-proc-gateway.service"
echo "  systemctl status uav-battery-tracker.service"
