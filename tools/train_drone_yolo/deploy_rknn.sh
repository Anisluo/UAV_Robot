#!/usr/bin/env bash
# Push a freshly-converted mavic3_drone.rknn to the rk3588 and restart
# the NPU service so it picks up the new weights.
#
#   bash deploy_rknn.sh mavic3_drone.rknn ubuntu@192.168.1.101
#
# After deploy, send `npu.set_strategy {strategy: 5}` from HostGUI's
# NPU dropdown (or the script `/tmp/uav_proc_npu.ctrl.sock` SSE) to
# force the new model to load. proc_npu auto-detects nc, output
# count, and bbox decode style — no source change required if you
# kept the standard YOLOv8 export.

set -euo pipefail

RKNN="${1:?usage: $0 <local.rknn> <user@host>}"
HOST="${2:?usage: $0 <local.rknn> <user@host>}"

[[ -f "$RKNN" ]] || { echo "ERROR: $RKNN not found"; exit 1; }
sz=$(stat -c%s "$RKNN" 2>/dev/null || stat -f%z "$RKNN")
echo "[deploy] $RKNN ($((sz / 1024)) KB) → $HOST:/home/ubuntu/UAV_Robot/mavic3_drone.rknn"

# Backup the existing rknn so we can roll back if the new model regresses.
ssh "$HOST" \
  'cp /home/ubuntu/UAV_Robot/mavic3_drone.rknn \
      /home/ubuntu/UAV_Robot/mavic3_drone.rknn.bak_$(date +%Y%m%d_%H%M%S) 2>/dev/null || true'

scp "$RKNN" "$HOST":/home/ubuntu/UAV_Robot/mavic3_drone.rknn
ssh "$HOST" 'sudo systemctl restart uav-proc-npu.service'

# Re-arm the strategy so proc_npu loads the new weights.
ssh "$HOST" 'sleep 3 && sudo python3 -c "
import socket
s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM); s.settimeout(5)
s.connect(\"/tmp/uav_proc_npu.ctrl.sock\")
s.sendall(b\"{\\\"id\\\":1,\\\"method\\\":\\\"npu.set_strategy\\\",\\\"params\\\":{\\\"strategy\\\":5}}\\n\")
print(s.recv(512).decode())
"'

echo "[deploy] done. tail -f the NPU log to see the new model's nc / outputs:"
echo "  ssh $HOST 'sudo journalctl -u uav-proc-npu --no-pager -n 20'"
