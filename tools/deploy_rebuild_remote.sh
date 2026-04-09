#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PROJECT_ROOT=$(cd "${SCRIPT_DIR}/.." && pwd)

REMOTE_USER="ubuntu"
REMOTE_HOST=""
REMOTE_PATH=""
REMOTE_PASSWORD=""
SERVICES="uav_robotd,proc_realsense,proc_npu,proc_gateway,proc_car,proc_gripper,proc_arm,proc_airport"

usage() {
    cat <<'EOF'
Usage:
  ./tools/deploy_rebuild_remote.sh --host 192.168.10.2 [options]

Behavior:
  1. Sync current UAV_Robot project to remote host
  2. Stop old UAV_Robot services and leftover processes
  3. Rebuild selected services on remote host
  4. Reinstall systemd units and restart services

Options:
  --host        Remote host IP or hostname
  --user        Remote user. Default: ubuntu
  --password    Remote password. Optional, requires sshpass
  --remote-path Remote project path. Default: /home/<user>/UAV_Robot
  --services    Comma-separated services. Default: all
  -h, --help    Show this help message

Examples:
  ./tools/deploy_rebuild_remote.sh --host 192.168.10.2 --password ubuntu
  ./tools/deploy_rebuild_remote.sh --host 192.168.10.2 --password ubuntu --services uav_robotd,proc_gateway,proc_arm
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
if [[ -n "${REMOTE_PASSWORD}" ]]; then
    if ! command -v sshpass >/dev/null 2>&1; then
        echo "sshpass is required when --password is provided" >&2
        exit 1
    fi
    SSH_BASE=(sshpass -p "${REMOTE_PASSWORD}" ssh -o StrictHostKeyChecking=no)
fi

REMOTE="${REMOTE_USER}@${REMOTE_HOST}"

echo "Preparing remote path ${REMOTE}:${REMOTE_PATH}"
"${SSH_BASE[@]}" "${REMOTE}" "mkdir -p ${REMOTE_PATH@Q}"

echo "Syncing project files to remote ..."
tar \
    --exclude=".git" \
    --exclude="build" \
    --exclude="*/build" \
    --exclude="*.pyc" \
    --exclude="__pycache__" \
    -cf - \
    -C "${PROJECT_ROOT}" . \
| "${SSH_BASE[@]}" "${REMOTE}" "tar -xf - -C ${REMOTE_PATH@Q}"

REMOTE_CMD=$(cat <<EOF
set -euo pipefail
cd ${REMOTE_PATH@Q}
chmod +x tools/install_autostart.sh
chmod +x tools/service_manager.sh
chmod +x tools/redeploy_services.sh
EOF
)

if [[ -n "${REMOTE_PASSWORD}" ]]; then
    REMOTE_CMD+=$'\n'"printf '%s\n' ${REMOTE_PASSWORD@Q} | sudo -S ./tools/redeploy_services.sh --services ${SERVICES@Q}"
else
    REMOTE_CMD+=$'\n'"sudo ./tools/redeploy_services.sh --services ${SERVICES@Q}"
fi

echo "Running remote rebuild and restart ..."
"${SSH_BASE[@]}" "${REMOTE}" "bash -lc ${REMOTE_CMD@Q}"

echo
echo "Remote deploy completed for ${REMOTE}"
