#!/usr/bin/env bash
#
# start_all.sh - one-click start of all UAV_Robot services on the target.
#
# Starts every systemd unit then VERIFIES each one actually came up
# (systemd state == active AND a real process is running). Exits non-zero
# if anything failed to start.
#
#   sudo ./tools/start_all.sh
#   sudo ./tools/start_all.sh --services uav_robotd,proc_arm,proc_gateway
#
set -uo pipefail

ORIGINAL_ARGS=("$@")
SERVICES_FILTER=""

# Pairs of "<unit>:<process pattern for pgrep -f>". The process pattern
# verifies that the binary is actually executing - systemd reports the
# unit as "active" even if the process crashed and is mid-restart.
declare -A UNIT_TO_PATTERN=(
    [uav-robotd.service]="build/bin/uav_robotd"
    [uav-proc-realsense.service]="proc_realsense/build/bin/proc_realsense"
    [uav-proc-npu.service]="proc_npu/build/bin/proc_npu"
    [uav-proc-gateway.service]="proc_gateway/build/bin/proc_gateway"
    [uav-proc-car.service]="proc_car/build/bin/proc_car"
    [uav-proc-gripper.service]="proc_gripper/build/bin/proc_gripper"
    [uav-proc-arm.service]="proc_arm/build/bin/proc_arm"
    [uav-proc-airport.service]="proc_airport/build/bin/proc_airport"
    [uav-proc-grasp.service]="proc_grasp/build/bin/proc_grasp"
)

# short-name -> systemd unit name
declare -A SHORT_TO_UNIT=(
    [uav_robotd]="uav-robotd.service"
    [proc_realsense]="uav-proc-realsense.service"
    [proc_npu]="uav-proc-npu.service"
    [proc_gateway]="uav-proc-gateway.service"
    [proc_car]="uav-proc-car.service"
    [proc_gripper]="uav-proc-gripper.service"
    [proc_arm]="uav-proc-arm.service"
    [proc_airport]="uav-proc-airport.service"
    [proc_grasp]="uav-proc-grasp.service"
)

# Order matters - bring up the daemons that own hardware first, then the
# gateway / NPU which depend on them.
UNIT_ORDER=(
    "uav-robotd.service"
    "uav-proc-arm.service"
    "uav-proc-car.service"
    "uav-proc-gripper.service"
    "uav-proc-airport.service"
    "uav-proc-realsense.service"
    "uav-proc-npu.service"
    "uav-proc-grasp.service"
    "uav-proc-gateway.service"
)

VERIFY_TIMEOUT=15  # seconds to wait for each unit to be active
C_GREEN=$'\033[1;32m'
C_RED=$'\033[1;31m'
C_YELLOW=$'\033[1;33m'
C_RESET=$'\033[0m'

require_root() {
    if [[ ${EUID} -ne 0 ]]; then
        exec sudo "$0" "${ORIGINAL_ARGS[@]}"
    fi
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        -h|--help)
            cat <<EOF
Usage: sudo ./tools/start_all.sh [--services name1,name2,...]

Starts UAV_Robot systemd services in dependency order, then waits up to
${VERIFY_TIMEOUT}s per unit for systemd to report it active and verifies
the underlying process is actually running. Exits non-zero on failure.

Without --services this starts every installed unit. With --services it
only starts (and only verifies) the listed subset.

Service names: uav_robotd, proc_realsense, proc_npu, proc_gateway,
               proc_car, proc_gripper, proc_arm, proc_airport, proc_grasp
EOF
            exit 0
            ;;
        --services)
            SERVICES_FILTER="${2:-}"
            shift 2
            ;;
        *)
            echo "Unknown argument: $1" >&2
            exit 1
            ;;
    esac
done

# If --services was passed, narrow UNIT_ORDER to that set, preserving order.
if [[ -n "${SERVICES_FILTER}" ]]; then
    declare -A FILTER_SET=()
    IFS=',' read -r -a names <<< "${SERVICES_FILTER}"
    for n in "${names[@]}"; do
        n=$(echo "${n}" | tr -d '[:space:]')
        unit="${SHORT_TO_UNIT[${n}]:-}"
        if [[ -z "${unit}" ]]; then
            echo "Unknown service name: ${n}" >&2
            exit 1
        fi
        FILTER_SET[${unit}]=1
    done
    new_order=()
    for u in "${UNIT_ORDER[@]}"; do
        if [[ -n "${FILTER_SET[${u}]:-}" ]]; then
            new_order+=("${u}")
        fi
    done
    UNIT_ORDER=("${new_order[@]}")
fi

require_root

unit_installed() {
    systemctl list-unit-files "$1" --no-legend 2>/dev/null | grep -q "^$1"
}

unit_active() {
    [[ "$(systemctl is-active "$1" 2>/dev/null || true)" == "active" ]]
}

process_alive() {
    pgrep -f "$1" >/dev/null 2>&1
}

# ── 1. start everything ────────────────────────────────────────────────────
echo "[1/3] Starting systemd units ..."
systemctl daemon-reload

started=()
skipped=()
for unit in "${UNIT_ORDER[@]}"; do
    if ! unit_installed "${unit}"; then
        echo "  ${C_YELLOW}skip${C_RESET}  ${unit} (not installed)"
        skipped+=("${unit}")
        continue
    fi
    echo "  start ${unit}"
    systemctl start "${unit}" 2>/dev/null || true
    started+=("${unit}")
done

# ── 2. verify systemd thinks they are active ───────────────────────────────
echo "[2/3] Verifying systemd state (timeout ${VERIFY_TIMEOUT}s/unit) ..."
failed_systemd=()
for unit in "${started[@]}"; do
    deadline=$(( SECONDS + VERIFY_TIMEOUT ))
    while (( SECONDS < deadline )); do
        if unit_active "${unit}"; then
            break
        fi
        sleep 1
    done
    if unit_active "${unit}"; then
        echo "  ${C_GREEN}active${C_RESET}  ${unit}"
    else
        state=$(systemctl is-active "${unit}" 2>/dev/null || true)
        echo "  ${C_RED}${state:-unknown}${C_RESET}  ${unit}"
        failed_systemd+=("${unit}")
    fi
done

# ── 3. verify the actual binaries are running ──────────────────────────────
echo "[3/3] Verifying processes ..."
failed_proc=()
for unit in "${started[@]}"; do
    pat="${UNIT_TO_PATTERN[${unit}]:-}"
    if [[ -z "${pat}" ]]; then
        continue
    fi
    if process_alive "${pat}"; then
        pid=$(pgrep -f "${pat}" | head -n 1)
        echo "  ${C_GREEN}running${C_RESET} ${unit} (pid=${pid})"
    else
        echo "  ${C_RED}missing${C_RESET} ${unit} (no process matching '${pat}')"
        failed_proc+=("${unit}")
    fi
done

# ── final report ───────────────────────────────────────────────────────────
echo
if (( ${#failed_systemd[@]} == 0 && ${#failed_proc[@]} == 0 )); then
    echo "${C_GREEN}OK${C_RESET}: all installed UAV_Robot services are running."
    if (( ${#skipped[@]} > 0 )); then
        echo "Skipped (not installed): ${skipped[*]}"
        echo "  Run: sudo ./tools/install_autostart.sh --services <name>"
    fi
    exit 0
fi

echo "${C_RED}FAILED${C_RESET}: some services did not come up cleanly."
if (( ${#failed_systemd[@]} > 0 )); then
    echo "  ${C_YELLOW}systemd not active:${C_RESET}"
    for u in "${failed_systemd[@]}"; do
        echo "    - ${u}"
        echo "      sudo journalctl -u ${u} -n 50 --no-pager"
    done
fi
if (( ${#failed_proc[@]} > 0 )); then
    echo "  ${C_YELLOW}no process running:${C_RESET}"
    for u in "${failed_proc[@]}"; do
        echo "    - ${u}"
    done
fi
exit 1
