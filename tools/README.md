# UAV_Robot tools/

A small set of scripts for deploying and operating the UAV_Robot stack on
an RK3588 host.

## TL;DR — three commands you actually need

### 1. One-click deploy from your laptop to a fresh RK3588

Run from anywhere on the project (Windows MINGW, Linux, macOS — anywhere with
Python 3 and `paramiko`):

```bash
python tools/deploy.py --host 192.168.10.2
```

What it does:

1. Tars the project (excluding `build/`, `.git/`, `__pycache__/`)
2. SFTPs the tarball to `/home/ubuntu/UAV_Robot` on the target
3. `apt install` build deps (`build-essential make libjpeg-dev can-utils`)
4. Installs the Intel RealSense SDK (idempotent — safe to re-run)
5. Builds every service (`uav_robotd`, `proc_arm`, `proc_gateway`, ...)
6. Installs all systemd units, enables them, starts them
7. Prints a final status summary

The script is idempotent: re-running just resyncs the source and
restarts the services.

Common flags:

```bash
# different host / credentials
python tools/deploy.py --host 192.168.1.50 --user ubuntu --password mypass

# subsequent deploys (skip apt + RealSense for speed)
python tools/deploy.py --host 192.168.10.2 --skip-apt --skip-realsense

# only build/install a subset
python tools/deploy.py --host 192.168.10.2 --services uav_robotd,proc_arm,proc_gateway
```

Requires `paramiko` locally:

```bash
python -m pip install paramiko
```

### 2. One-click stop (run on the RK3588)

```bash
sudo ./tools/stop_all.sh
```

Stops every UAV_Robot systemd service, kills any leftover hand-launched
processes (including those at unknown install paths), wipes stale IPC
sockets in `/tmp/`, then **verifies** nothing is still running. Exits
non-zero if cleanup is incomplete.

### 3. One-click start (run on the RK3588)

```bash
sudo ./tools/start_all.sh                                # all installed
sudo ./tools/start_all.sh --services uav_robotd,proc_arm # subset
```

Starts services in dependency order, then waits up to 15s per unit for
systemd to report it active **and** verifies the underlying binary is
actually running. Exits non-zero on failure with a per-service breakdown
and journal log hints.

---

## File reference

| File | Purpose |
|------|---------|
| **deploy.py** | One-click deploy from your laptop to a target RK3588. |
| **start_all.sh** | One-click `systemctl start` of every UAV_Robot service. |
| **stop_all.sh** | One-click stop + leftover-process cleanup + socket cleanup. |
| **install_autostart.sh** | Lower-level: builds selected services and installs/enables systemd units. Called by `deploy.py`. |
| **service_manager.sh** | Lower-level: per-service `start`/`stop`/`restart`/`status`/`logs`. Use when you only want to touch one or two services. |
| **realsense_install.sh** | Installs the Intel RealSense SDK (one-time per host). Called by `deploy.py` unless `--skip-realsense`. |
| **arm_motor_cli.py** | Interactive arm CLI for low-level joint debugging. Talks to `proc_arm` over `/tmp/uav_proc_arm.sock`. |
| **arm_motor_test.c** | C source for `build/bin/arm_motor_test`, the binary `arm_motor_cli.py` falls back to when not using the proc_arm transport. |
| **chassis_test.py** | Standalone diagnostic for the 6-wheel chassis on CAN3. |
| **rpc_test.py** | Tiny JSON-RPC client for `proc_arm` — used for ad-hoc testing of single methods. |
| **requirements.txt** | Python deps for the diagnostic scripts (`paramiko`, `pyzmq`, etc). |

---

## Typical workflow

**First time on a brand-new RK3588:**

```bash
python tools/deploy.py --host 192.168.10.2
```

Done. Services are running.

**After you change source code on your laptop:**

```bash
python tools/deploy.py --host 192.168.10.2 --skip-apt --skip-realsense
```

This resyncs the source, rebuilds, and restarts the services.

**Touch only one service:**

```bash
ssh ubuntu@192.168.10.2
sudo ./tools/service_manager.sh restart --services proc_arm
sudo ./tools/service_manager.sh logs --services proc_arm --lines 200
```

**Stop everything (e.g. before a power cycle or tinkering):**

```bash
ssh ubuntu@192.168.10.2
sudo ./tools/stop_all.sh
```

**Bring everything back:**

```bash
sudo ./tools/start_all.sh
```

---

## Diagnostic scripts (run on the target as needed)

```bash
# interactive joint debugger
python3 ./tools/arm_motor_cli.py

# raw RPC test (e.g. force a stop)
python3 ./tools/rpc_test.py arm.stop

# chassis test
python3 ./tools/chassis_test.py
```
