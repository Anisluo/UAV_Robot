#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PROJECT_ROOT=$(cd "${SCRIPT_DIR}/.." && pwd)

DEFAULT_SERVICES="uav_robotd,proc_realsense,proc_npu,proc_gateway,proc_car,proc_gripper,proc_arm,proc_airport"
SERVICES="${DEFAULT_SERVICES}"
ORIGINAL_ARGS=("$@")

declare -A UNIT_NAMES=(
    [uav_robotd]="uav-robotd.service"
    [proc_realsense]="uav-proc-realsense.service"
    [proc_npu]="uav-proc-npu.service"
    [proc_gateway]="uav-proc-gateway.service"
    [proc_car]="uav-proc-car.service"
    [proc_gripper]="uav-proc-gripper.service"
    [proc_arm]="uav-proc-arm.service"
    [proc_airport]="uav-proc-airport.service"
)

usage() {
    cat <<'EOF'
Usage:
  ./tools/redeploy_services.sh [--services uav_robotd,proc_gateway,proc_arm]

Behavior:
  1. Stop known UAV_Robot services
  2. Kill leftover manual processes
  3. Remove stale IPC socket files
  4. Rebuild and reinstall selected systemd services
  5. Start selected services again

Options:
  --services   Comma-separated service list. Default: all known services
  -h, --help   Show this help message
EOF
}

normalize_service_name() {
    local name
    name=$(echo "$1" | tr -d '[:space:]')
    case "${name}" in
        uav_robotd|proc_realsense|proc_npu|proc_gateway|proc_car|proc_gripper|proc_arm|proc_airport)
            printf '%s\n' "${name}"
            ;;
        *)
            echo "Unsupported service name: ${name}" >&2
            exit 1
            ;;
    esac
}

require_root() {
    if [[ ${EUID} -ne 0 ]]; then
        exec sudo "$0" "${ORIGINAL_ARGS[@]}"
    fi
}

collect_units() {
    local raw_service
    local service
    local units=()
    IFS=',' read -r -a REQUESTED_SERVICES <<< "${SERVICES}"
    for raw_service in "${REQUESTED_SERVICES[@]}"; do
        service=$(normalize_service_name "${raw_service}")
        units+=("${UNIT_NAMES[${service}]}")
    done
    printf '%s\n' "${units[@]}"
}

stop_known_units() {
    local units=(
        "uav-robotd.service"
        "uav-proc-realsense.service"
        "uav-proc-npu.service"
        "uav-proc-gateway.service"
        "uav-proc-car.service"
        "uav-proc-gripper.service"
        "uav-proc-arm.service"
        "uav-proc-airport.service"
    )

    echo "Stopping known UAV_Robot services ..."
    systemctl stop "${units[@]}" 2>/dev/null || true
}

kill_leftovers() {
    local patterns=(
        "/build/bin/uav_robotd"
        "/proc_realsense/build/bin/proc_realsense"
        "/proc_npu/build/bin/proc_npu"
        "/proc_gateway/build/bin/proc_gateway"
        "/proc_car/build/bin/proc_car"
        "/proc_gripper/build/bin/proc_gripper"
        "/proc_arm/build/bin/proc_arm"
        "/proc_airport/build/bin/proc_airport"
    )
    local pattern

    echo "Killing leftover manual processes ..."
    for pattern in "${patterns[@]}"; do
        pkill -9 -f "${pattern}" 2>/dev/null || true
    done
}

cleanup_ipc_files() {
    echo "Removing stale IPC files ..."
    rm -f \
        /tmp/uav_proc_arm.sock \
        /tmp/uav_proc_realsense.ctrl.sock \
        /tmp/uav_proc_npu.ctrl.sock \
        /tmp/uav_realsense_frame_notify.sock \
        /tmp/uav_gw_npu_rx.sock \
        /tmp/uav_app_npu_rx.sock
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    usage
    exit 0
fi

while [[ $# -gt 0 ]]; do
    case "$1" in
        --services)
            SERVICES="${2:-}"
            shift 2
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

require_root

echo "Project root: ${PROJECT_ROOT}"
echo "Selected services: ${SERVICES}"

stop_known_units
kill_leftovers
cleanup_ipc_files

cd "${PROJECT_ROOT}"
./tools/install_autostart.sh --services "${SERVICES}"

mapfile -t SELECTED_UNITS < <(collect_units)
echo
echo "Final status:"
systemctl --no-pager --full status "${SELECTED_UNITS[@]}" || true
