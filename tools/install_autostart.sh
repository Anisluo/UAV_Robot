#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PROJECT_ROOT=$(cd "${SCRIPT_DIR}/.." && pwd)
SYSTEMD_SRC_DIR="${PROJECT_ROOT}/systemd"
SYSTEMD_DST_DIR="/etc/systemd/system"
ENV_DST_FILE="/etc/default/uav_robot"

DEFAULT_SERVICES="uav_robotd,proc_realsense,proc_npu,proc_gateway"
SERVICES="${DEFAULT_SERVICES}"
ORIGINAL_ARGS=("$@")

usage() {
    cat <<'EOF'
Usage:
  sudo ./tools/install_autostart.sh [--services uav_robotd,proc_gateway]

Options:
  --services   Comma-separated service list. Available values:
               uav_robotd, proc_realsense, proc_npu, proc_gateway
  -h, --help   Show this help message.

Examples:
  sudo ./tools/install_autostart.sh
  sudo ./tools/install_autostart.sh --services uav_robotd,proc_gateway
EOF
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

if [[ ${EUID} -ne 0 ]]; then
    exec sudo "$0" "${ORIGINAL_ARGS[@]}"
fi

if [[ ! -d "${SYSTEMD_SRC_DIR}" ]]; then
    echo "systemd template directory not found: ${SYSTEMD_SRC_DIR}" >&2
    exit 1
fi

declare -A BUILD_COMMANDS=(
    [uav_robotd]="make dirs build/bin/uav_robotd build/bin/uav_cli"
    [proc_realsense]="make -C proc_realsense"
    [proc_npu]="make -C proc_npu"
    [proc_gateway]="make -C proc_gateway"
)

declare -A UNIT_NAMES=(
    [uav_robotd]="uav-robotd.service"
    [proc_realsense]="uav-proc-realsense.service"
    [proc_npu]="uav-proc-npu.service"
    [proc_gateway]="uav-proc-gateway.service"
)

IFS=',' read -r -a REQUESTED_SERVICES <<< "${SERVICES}"

normalize_service_name() {
    local name
    name=$(echo "$1" | tr -d '[:space:]')
    case "${name}" in
        uav_robotd|proc_realsense|proc_npu|proc_gateway)
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

mkdir -p /etc/default
if [[ ! -f "${ENV_DST_FILE}" ]]; then
    install -m 0644 "${SYSTEMD_SRC_DIR}/uav_robot.env.example" "${ENV_DST_FILE}"
    echo "Created default environment file: ${ENV_DST_FILE}"
else
    echo "Keeping existing environment file: ${ENV_DST_FILE}"
fi

for raw_service in "${REQUESTED_SERVICES[@]}"; do
    service=$(normalize_service_name "${raw_service}")
    echo "Building ${service} ..."
    (
        cd "${PROJECT_ROOT}"
        eval "${BUILD_COMMANDS[${service}]}"
    )

    unit_name="${UNIT_NAMES[${service}]}"
    tmp_unit=$(mktemp)
    render_unit "${SYSTEMD_SRC_DIR}/${unit_name}" "${tmp_unit}"
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
