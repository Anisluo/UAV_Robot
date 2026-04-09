#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PROJECT_ROOT=$(cd "${SCRIPT_DIR}/.." && pwd)

DEFAULT_SERVICES="uav_robotd,proc_realsense,proc_npu,proc_gateway,proc_car,proc_gripper,proc_arm,proc_airport"
SERVICES="${DEFAULT_SERVICES}"
ACTION=""
LINES=100
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
  ./tools/service_manager.sh <action> [--services uav_robotd,proc_gateway] [--lines 200]

Actions:
  install    Build selected modules and install/restart their systemd services
  start      Start selected services
  stop       Stop selected services
  restart    Restart selected services
  status     Show status of selected services
  logs       Show recent logs of selected services
  enable     Enable selected services at boot
  disable    Disable selected services at boot
  reload     Run systemctl daemon-reload

Options:
  --services Comma-separated service list. Available values:
             uav_robotd, proc_realsense, proc_npu, proc_gateway,
             proc_car, proc_gripper, proc_arm, proc_airport
  --lines    Number of log lines for `logs`. Default: 100
  -h, --help Show this help message

Examples:
  ./tools/service_manager.sh stop --services proc_arm,proc_gateway,uav_robotd
  ./tools/service_manager.sh start --services uav_robotd,proc_gateway,proc_arm
  ./tools/service_manager.sh install --services proc_arm
  ./tools/service_manager.sh logs --services proc_arm --lines 200
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

if [[ $# -eq 0 ]]; then
    usage >&2
    exit 1
fi

while [[ $# -gt 0 ]]; do
    case "$1" in
        install|start|stop|restart|status|logs|enable|disable|reload)
            if [[ -n "${ACTION}" ]]; then
                echo "Only one action can be specified" >&2
                exit 1
            fi
            ACTION="$1"
            shift
            ;;
        --services)
            SERVICES="${2:-}"
            shift 2
            ;;
        --lines)
            LINES="${2:-}"
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

if [[ -z "${ACTION}" ]]; then
    echo "Missing action" >&2
    usage >&2
    exit 1
fi

require_root

mapfile -t UNITS < <(collect_units)

case "${ACTION}" in
    install)
        cd "${PROJECT_ROOT}"
        ./tools/install_autostart.sh --services "${SERVICES}"
        ;;
    start)
        systemctl start "${UNITS[@]}"
        ;;
    stop)
        systemctl stop "${UNITS[@]}"
        ;;
    restart)
        systemctl restart "${UNITS[@]}"
        ;;
    status)
        systemctl status "${UNITS[@]}"
        ;;
    logs)
        journalctl --no-pager -n "${LINES}" -u "${UNITS[@]}"
        ;;
    enable)
        systemctl enable "${UNITS[@]}"
        ;;
    disable)
        systemctl disable "${UNITS[@]}"
        ;;
    reload)
        systemctl daemon-reload
        ;;
esac
