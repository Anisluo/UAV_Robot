#!/usr/bin/env bash
#
# stop_all.sh - one-click stop of every UAV_Robot service on the target.
#
# Stops the systemd units, kills any leftover hand-launched processes
# (including those from previous deploys whose paths/PIDs we don't know),
# removes stale IPC sockets, and then VERIFIES that nothing is still
# running before exiting. Exits non-zero if anything refused to die.
#
#   sudo ./tools/stop_all.sh
#
set -uo pipefail

ORIGINAL_ARGS=("$@")

UNITS=(
    "uav-robotd.service"
    "uav-proc-realsense.service"
    "uav-proc-npu.service"
    "uav-proc-grasp.service"
    "uav-proc-gateway.service"
    "uav-proc-car.service"
    "uav-proc-gripper.service"
    "uav-proc-arm.service"
    "uav-proc-airport.service"
    "uav-proc-door.service"
)

# Patterns for `pkill -f` to catch processes started by hand or under a
# different install path. We match by binary name, not full path, so this
# also catches leftovers from a previous deploy at /opt/, /root/, etc.
PROCESS_PATTERNS=(
    "uav_robotd"
    "proc_realsense"
    "proc_npu"
    "proc_grasp"
    "proc_gateway"
    "proc_car"
    "proc_gripper"
    "proc_arm"
    "proc_airport"
    "proc_door"
)

# Stale unix sockets / FIFOs we want to wipe between runs.
IPC_FILES=(
    /tmp/uav_proc_arm.sock
    /tmp/uav_proc_realsense.ctrl.sock
    /tmp/uav_proc_npu.ctrl.sock
    /tmp/uav_proc_grasp.ctrl.sock
    /tmp/uav_proc_door.sock
    /tmp/uav_realsense_frame_notify.sock
    /tmp/uav_gw_npu_rx.sock
    /tmp/uav_app_npu_rx.sock
    /tmp/uav_grasp_npu_rx.sock
)

# Maximum time we wait for processes to actually exit after SIGKILL (seconds).
KILL_WAIT_TIMEOUT=10

C_GREEN=$'\033[1;32m'
C_RED=$'\033[1;31m'
C_YELLOW=$'\033[1;33m'
C_RESET=$'\033[0m'

require_root() {
    if [[ ${EUID} -ne 0 ]]; then
        exec sudo "$0" "${ORIGINAL_ARGS[@]}"
    fi
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    cat <<EOF
Usage: sudo ./tools/stop_all.sh

Stops every UAV_Robot systemd service, kills any leftover processes,
removes stale IPC socket files, and verifies that nothing is left
running. Exits non-zero on failure.
EOF
    exit 0
fi

require_root

# ── 1. systemctl stop ──────────────────────────────────────────────────────
echo "[1/5] Stopping systemd units ..."
systemctl stop "${UNITS[@]}" 2>/dev/null || true
systemctl reset-failed "${UNITS[@]}" 2>/dev/null || true

# ── 2. kill any leftover processes (multiple passes) ───────────────────────
echo "[2/5] Killing leftover processes ..."

# Pass 2a: polite TERM via pkill -f (full command line match).
for pat in "${PROCESS_PATTERNS[@]}"; do
    pkill -TERM -f "${pat}" 2>/dev/null || true
done
sleep 1

# Pass 2b: forced KILL via pkill -f.
for pat in "${PROCESS_PATTERNS[@]}"; do
    pkill -9 -f "${pat}" 2>/dev/null || true
done

# Pass 2c: cgroup-based kill via systemd. Catches processes pkill missed
# because they were started outside the systemd unit's expected pattern
# but are still tracked in the unit's cgroup.
for unit in "${UNITS[@]}"; do
    systemctl kill -s SIGKILL "${unit}" 2>/dev/null || true
done

# Pass 2d: brute-force ps + kill -9 by binary path. Catches stragglers
# whose argv0 doesn't match pkill's pattern (e.g. wrapped under bash -c
# or daemonized at boot outside systemd's view, like the long-running
# processes that pkill -f sometimes can't reach).
mapfile -t straggler_pids < <(
    ps -eo pid,cmd --no-headers 2>/dev/null \
      | grep -E "uav_robotd|/build/bin/proc_(realsense|npu|gateway|grasp|arm|car|gripper|airport)" \
      | grep -v grep \
      | awk '{print $1}'
)
if (( ${#straggler_pids[@]} > 0 )); then
    echo "        kill -9 stragglers: ${straggler_pids[*]}"
    kill -9 "${straggler_pids[@]}" 2>/dev/null || true
fi
sleep 1

# ── 3. remove stale IPC files ─────────────────────────────────────────────
echo "[3/5] Removing stale IPC sockets ..."
rm -f "${IPC_FILES[@]}"

# ── 4. wait & verify nothing is still alive ───────────────────────────────
echo "[4/5] Waiting for processes to exit (timeout ${KILL_WAIT_TIMEOUT}s) ..."

list_alive() {
    local pat
    local found=()
    for pat in "${PROCESS_PATTERNS[@]}"; do
        # pgrep -f matches against the full command line
        local pids
        pids=$(pgrep -f "${pat}" 2>/dev/null || true)
        if [[ -n "${pids}" ]]; then
            for pid in ${pids}; do
                # exclude this script itself and the parent shell
                if [[ "${pid}" != "$$" && "${pid}" != "${PPID}" ]]; then
                    found+=("${pid}:${pat}")
                fi
            done
        fi
    done
    if (( ${#found[@]} > 0 )); then
        printf '%s\n' "${found[@]}"
    fi
}

deadline=$(( SECONDS + KILL_WAIT_TIMEOUT ))
while (( SECONDS < deadline )); do
    alive=$(list_alive)
    if [[ -z "${alive}" ]]; then
        break
    fi
    # extra forced kill on whatever is still up
    for entry in ${alive}; do
        pid="${entry%%:*}"
        kill -9 "${pid}" 2>/dev/null || true
    done
    sleep 1
done

# ── 5. final check + report ───────────────────────────────────────────────
echo "[5/5] Verifying ..."
remaining_procs=$(list_alive)
remaining_units=()
for unit in "${UNITS[@]}"; do
    state=$(systemctl is-active "${unit}" 2>/dev/null || true)
    if [[ "${state}" == "active" || "${state}" == "activating" ]]; then
        remaining_units+=("${unit}")
    fi
done

stale_files=()
for f in "${IPC_FILES[@]}"; do
    if [[ -e "${f}" ]]; then
        stale_files+=("${f}")
    fi
done

echo
if [[ -z "${remaining_procs}" && ${#remaining_units[@]} -eq 0 && ${#stale_files[@]} -eq 0 ]]; then
    echo "${C_GREEN}OK${C_RESET}: all UAV_Robot services stopped, no stray processes or sockets."
    exit 0
fi

echo "${C_RED}FAILED${C_RESET}: cleanup incomplete."
if [[ -n "${remaining_procs}" ]]; then
    echo "  ${C_YELLOW}Still running:${C_RESET}"
    while IFS= read -r line; do
        echo "    - ${line}"
    done <<< "${remaining_procs}"
fi
if (( ${#remaining_units[@]} > 0 )); then
    echo "  ${C_YELLOW}Still active units:${C_RESET}"
    for u in "${remaining_units[@]}"; do
        echo "    - ${u}"
    done
fi
if (( ${#stale_files[@]} > 0 )); then
    echo "  ${C_YELLOW}Stale IPC files:${C_RESET}"
    for f in "${stale_files[@]}"; do
        echo "    - ${f}"
    done
fi
exit 1
