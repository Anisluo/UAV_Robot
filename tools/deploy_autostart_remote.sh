#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PROJECT_ROOT=$(cd "${SCRIPT_DIR}/.." && pwd)

REMOTE_USER="ubuntu"
REMOTE_HOST=""
REMOTE_PATH=""
REMOTE_PASSWORD=""
SERVICES="uav_robotd,proc_realsense,proc_npu,proc_gateway"

usage() {
    cat <<'EOF'
Usage:
  ./tools/deploy_autostart_remote.sh --host 192.168.1.101 [options]

Options:
  --host        Remote host IP or name.
  --user        Remote user name. Default: ubuntu
  --password    Remote password. Optional, requires sshpass.
  --remote-path Remote UAV_Robot path. Default: /home/<user>/UAV_Robot
  --services    Comma-separated service list.
  -h, --help    Show this help message.

Examples:
  ./tools/deploy_autostart_remote.sh --host 192.168.1.101 --password ubuntu
  ./tools/deploy_autostart_remote.sh --host 192.168.1.101 --remote-path /home/ubuntu/UAV_Robot --services uav_robotd,proc_gateway
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --host)
            REMOTE_HOST="${2:-}"
            shift 2
            ;;
        --user)
            REMOTE_USER="${2:-}"
            shift 2
            ;;
        --password)
            REMOTE_PASSWORD="${2:-}"
            shift 2
            ;;
        --remote-path)
            REMOTE_PATH="${2:-}"
            shift 2
            ;;
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

if [[ -z "${REMOTE_HOST}" ]]; then
    echo "--host is required" >&2
    usage >&2
    exit 1
fi

if [[ -z "${REMOTE_PATH}" ]]; then
    REMOTE_PATH="/home/${REMOTE_USER}/UAV_Robot"
fi

SSH_BASE=(ssh -o StrictHostKeyChecking=no)
SCP_BASE=(scp -o StrictHostKeyChecking=no)
if [[ -n "${REMOTE_PASSWORD}" ]]; then
    if ! command -v sshpass >/dev/null 2>&1; then
        echo "sshpass is required when --password is provided" >&2
        exit 1
    fi
    SSH_BASE=(sshpass -p "${REMOTE_PASSWORD}" ssh -o StrictHostKeyChecking=no)
    SCP_BASE=(sshpass -p "${REMOTE_PASSWORD}" scp -o StrictHostKeyChecking=no)
fi

REMOTE="${REMOTE_USER}@${REMOTE_HOST}"

"${SSH_BASE[@]}" "${REMOTE}" "mkdir -p ${REMOTE_PATH@Q}/tools ${REMOTE_PATH@Q}/systemd"
"${SCP_BASE[@]}" "${PROJECT_ROOT}/tools/install_autostart.sh" "${REMOTE}:${REMOTE_PATH}/tools/install_autostart.sh"
"${SCP_BASE[@]}" "${PROJECT_ROOT}/systemd/"*.service "${REMOTE}:${REMOTE_PATH}/systemd/"
"${SCP_BASE[@]}" "${PROJECT_ROOT}/systemd/uav_robot.env.example" "${REMOTE}:${REMOTE_PATH}/systemd/uav_robot.env.example"

REMOTE_CMD=$(cat <<EOF
set -euo pipefail
cd ${REMOTE_PATH@Q}
chmod +x tools/install_autostart.sh
EOF
)

if [[ -n "${REMOTE_PASSWORD}" ]]; then
    REMOTE_CMD+=$'\n'"printf '%s\n' ${REMOTE_PASSWORD@Q} | sudo -S ./tools/install_autostart.sh --services ${SERVICES@Q}"
else
    REMOTE_CMD+=$'\n'"sudo ./tools/install_autostart.sh --services ${SERVICES@Q}"
fi

"${SSH_BASE[@]}" "${REMOTE}" "bash -lc ${REMOTE_CMD@Q}"

echo "Remote autostart deployment completed for ${REMOTE}"
