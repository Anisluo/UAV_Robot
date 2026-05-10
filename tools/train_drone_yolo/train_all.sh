#!/usr/bin/env bash
# One-shot training pipeline. Run on a Linux x86_64 PC with NVIDIA GPU.
# Assumes drone_dataset/ has been rsync'd from the rk3588 to ../.. .
#
#   bash train_all.sh                   # default: 100 epochs, yolov8n
#   EPOCHS=200 BATCH=32 bash train_all.sh
#
# After training, run deploy_rknn.sh on the same machine (or another
# Linux box with rknn-toolkit2) to convert + ship the .rknn to the
# rk3588.

set -euo pipefail

EPOCHS="${EPOCHS:-100}"
BATCH="${BATCH:-16}"
MODEL="${MODEL:-yolov8n.pt}"
RUN_NAME="${RUN_NAME:-mavic3_drone}"

cd "$(dirname "$0")"
DATASET="$(realpath ../../drone_dataset)"
[[ -d "$DATASET/images" ]] || {
  echo "ERROR: $DATASET/images not found — rsync from rk3588 first"
  echo "  rsync -avz ubuntu@<rk3588>:/home/ubuntu/drone_dataset $(realpath ../..)/"
  exit 1
}
echo "[train_all] dataset: $DATASET"

# 0. Install deps if missing.
if ! python3 -c "import ultralytics" 2>/dev/null; then
  echo "[train_all] installing ultralytics ..."
  pip install --user ultralytics
fi

# 1. Split (idempotent).
echo "[train_all] splitting train/val ..."
python3 split_dataset.py "$DATASET" --val-ratio 0.15

# 2. Train.
echo "[train_all] training $RUN_NAME for $EPOCHS epochs (batch=$BATCH) ..."
python3 train.py --data dataset.yaml --model "$MODEL" \
                 --epochs "$EPOCHS" --batch "$BATCH" --name "$RUN_NAME"

# 3. Export ONNX.
WEIGHTS="runs/detect/$RUN_NAME/weights/best.pt"
[[ -f "$WEIGHTS" ]] || { echo "ERROR: $WEIGHTS missing"; exit 1; }
echo "[train_all] exporting ONNX from $WEIGHTS ..."
python3 export_onnx.py --weights "$WEIGHTS"
ONNX="runs/detect/$RUN_NAME/weights/best.onnx"
[[ -f "$ONNX" ]] || { echo "ERROR: ONNX export failed"; exit 1; }

cat <<EOF

[train_all] done.
  best.pt   : $WEIGHTS
  best.onnx : $ONNX

Next: convert to RKNN on a machine with rknn-toolkit2 installed:

  python3 onnx_to_rknn.py \\
      --onnx  $ONNX \\
      --calib $DATASET/images/train \\
      --output mavic3_drone.rknn

Then deploy:

  bash deploy_rknn.sh mavic3_drone.rknn ubuntu@<rk3588>
EOF
