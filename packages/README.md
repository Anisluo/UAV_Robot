# UAV_Robot / packages

Site-deploy bundle picked up by `tools/install_autostart.sh`. Everything here
is **optional but recommended** — the install script is idempotent and just
skips anything that's missing, but you'll be missing functionality if you
skip the wrong file.

| File | Used by | Skip = what breaks |
| ---- | ------- | ------------------ |
| `librknnrt.so` | proc_npu | YOLOv8 RKNN models fail to load (only matters if board's `/usr/lib/librknnrt.so` predates the toolkit you used to export the model) |
| `gs_usb.ko` | proc_piper (gen-2 arm) | Piper backend won't bind to the candleLight USB-CAN — you'll see `can_piper` missing from `ip link` |
| `numpy-*.whl` + `opencv_python_headless-*.whl` | battery_tracker, face_tracker | CV sidecars exit on import error if you don't already have these |
| `piper_sdk*.whl`, `python_can*.whl`, `msgpack*.whl`, `packaging*.whl`, `typing_extensions*.whl`, `wrapt*.whl` | proc_piper | proc_piper won't start; install_autostart.sh will try online pip as fallback. python-can needs msgpack + wrapt + packaging + typing_extensions transitively — bundle all six. |
| `udev/*.rules` | RealSense + Piper CAN | RealSense camera intermittently fails to enumerate; CAN adapter shows up as random `canN` instead of `can_piper` |

## Kernel-matched bits — re-source per machine

**`gs_usb.ko` is kernel-specific.** A `.ko` built for `5.10.110-rk3588`
won't load on `5.10.160-rk3588`. To rebuild on the target:

```bash
# On the RK3588 with matching kernel headers installed
git clone --depth 1 https://github.com/candle-usb/candleLight_fw  # for reference
cd /usr/src/linux-headers-$(uname -r) || apt install linux-headers-$(uname -r)
# Build the in-tree gs_usb driver:
make -C /lib/modules/$(uname -r)/build M=$(pwd) drivers/net/can/usb modules
# Copy out the result:
cp /lib/modules/$(uname -r)/build/drivers/net/can/usb/gs_usb.ko \
   /path/to/UAV_Robot/packages/gs_usb.ko
```

Or if the kernel already ships gs_usb (Ubuntu 22.04+ usually does), just
delete this file and the install script will rely on the in-kernel driver.

## Network-isolated sites

For an air-gapped install, populate the wheel files before deploy:

```bash
# On a build machine with internet, targeting aarch64:
pip3 download \
    --platform manylinux_2_17_aarch64 \
    --python-version 38 --only-binary=:all: \
    --dest packages/ \
    opencv-python-headless numpy piper_sdk python-can msgpack
```

Then `./tools/install_autostart.sh` picks them up with `--no-index`.

## One-shot deploy on the target

```bash
sudo ./tools/install_autostart.sh                 # defaults: backend=piper
sudo ./tools/install_autostart.sh --backend legacy  # gen-1 ZDT arm
sudo ./tools/install_autostart.sh --skip-deps      # iterating during dev
```

Verify after:

```bash
ip link show can_piper                            # must exist and be UP
lsmod | grep gs_usb                               # must show gs_usb
python3 -c 'import piper_sdk, can'                # must not throw
systemctl status uav-proc-piper.service           # must be active (running)
```
