#!/usr/bin/env python3
"""One-click deploy of UAV_Robot to a fresh RK3588 host.

Usage from the project root (or anywhere - the script resolves paths from
its own location):

    python tools/deploy.py --host 192.168.10.2
    python tools/deploy.py --host 192.168.10.2 --user ubuntu --password ubuntu
    python tools/deploy.py --host 192.168.10.2 --services proc_arm,proc_gateway
    python tools/deploy.py --host 192.168.10.2 --skip-apt --skip-realsense

Steps:
    1. Tar the project source (excluding build/, .git/, __pycache__/)
    2. SFTP it to the target
    3. Extract under /home/<user>/UAV_Robot
    4. Optionally install apt build deps (libjpeg-dev, build-essential, make)
    5. Optionally install Intel RealSense SDK (one-time, only on first deploy)
    6. Run tools/install_autostart.sh on the target -> builds + installs
       systemd units + enables and starts the selected services
    7. Print final systemctl status

Designed to be re-runnable: every step is idempotent. Existing services are
stopped, leftover processes are killed, and stale IPC sockets are removed
before the rebuild starts.

Requires the `paramiko` package on the local machine (already used by other
diagnostic scripts in this repo).
"""

import argparse
import os
import subprocess
import sys
import tempfile
import time
from pathlib import Path

try:
    import paramiko
except ImportError:
    print(
        "[deploy] ERROR: paramiko is not installed.\n"
        "         Install with:  python -m pip install paramiko",
        file=sys.stderr,
    )
    sys.exit(1)


# All known proc_* services + the main daemon. Use this default unless the
# user passes --services explicitly.
ALL_SERVICES = (
    "uav_robotd,proc_realsense,proc_npu,proc_gateway,"
    "proc_car,proc_gripper,proc_arm,proc_airport,proc_grasp"
)

# Apt packages needed to build everything on a fresh Ubuntu 22.04 RK3588.
APT_PACKAGES = [
    "build-essential",
    "make",
    "libjpeg-dev",
    "can-utils",
]


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="One-click deploy UAV_Robot to a fresh RK3588 host."
    )
    p.add_argument("--host", required=True, help="Target IP or hostname")
    p.add_argument("--user", default="ubuntu", help="Remote SSH user (default: ubuntu)")
    p.add_argument(
        "--password",
        default="ubuntu",
        help="Remote SSH password (default: ubuntu)",
    )
    p.add_argument(
        "--remote-path",
        default=None,
        help="Remote project path (default: /home/<user>/UAV_Robot)",
    )
    p.add_argument(
        "--services",
        default=ALL_SERVICES,
        help=f"Comma-separated services to install. Default: {ALL_SERVICES}",
    )
    p.add_argument(
        "--skip-apt",
        action="store_true",
        help="Skip the apt package install step (use on subsequent deploys)",
    )
    p.add_argument(
        "--skip-realsense",
        action="store_true",
        help="Skip the Intel RealSense SDK install step",
    )
    p.add_argument(
        "--port",
        type=int,
        default=22,
        help="Remote SSH port (default: 22)",
    )
    return p.parse_args()


def project_root() -> Path:
    return Path(__file__).resolve().parent.parent


def to_tar_path(p: Path) -> str:
    """Convert a local path to one tar will accept.

    On MINGW / MSYS / Git-Bash a Windows-style path like ``F:\\foo`` makes
    tar try to open ``F:`` as an rsync-style remote host. Use cygpath -u
    to translate to ``/f/foo`` so tar treats it as a local directory.
    """
    if os.name == "nt" or "MSYSTEM" in os.environ:
        try:
            r = subprocess.run(
                ["cygpath", "-u", str(p)],
                capture_output=True,
                text=True,
                check=False,
            )
            if r.returncode == 0 and r.stdout.strip():
                return r.stdout.strip()
        except FileNotFoundError:
            pass
    return str(p)


def make_tarball(root: Path) -> Path:
    """Create a portable tarball of the project under /tmp."""
    tmp_dir = Path(tempfile.gettempdir())
    tmp = tmp_dir / "uav_robot_deploy.tar"
    if tmp.exists():
        tmp.unlink()
    print(f"[deploy] Packing project from {root}")
    tar_root = to_tar_path(root)
    tar_out = to_tar_path(tmp)
    cmd = [
        "tar",
        "--exclude=.git",
        "--exclude=build",
        "--exclude=*/build",
        "--exclude=*.pyc",
        "--exclude=__pycache__",
        "-cf",
        tar_out,
        "-C",
        tar_root,
        ".",
    ]
    rc = subprocess.run(cmd, capture_output=True, text=True)
    if rc.returncode != 0:
        print(f"[deploy] tar failed:\n{rc.stderr}", file=sys.stderr)
        sys.exit(1)
    size_mb = tmp.stat().st_size / 1024 / 1024
    print(f"[deploy] Archive ready: {tmp} ({size_mb:.1f} MB)")
    return tmp


def cygpath_w(p: Path) -> str:
    """Convert MINGW/Cygwin path to a Windows path so SFTP can read it.

    On non-Windows just return the path as-is.
    """
    if os.name == "nt" or "MSYSTEM" in os.environ:
        try:
            r = subprocess.run(
                ["cygpath", "-w", str(p)],
                capture_output=True,
                text=True,
                check=False,
            )
            if r.returncode == 0 and r.stdout.strip():
                return r.stdout.strip()
        except FileNotFoundError:
            pass
    return str(p)


class Remote:
    """Thin wrapper around paramiko.SSHClient with sudo helper."""

    def __init__(self, host: str, port: int, user: str, password: str):
        self.user = user
        self.password = password
        print(f"[deploy] Connecting to {user}@{host}:{port} ...")
        self.ssh = paramiko.SSHClient()
        self.ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        self.ssh.connect(
            hostname=host,
            port=port,
            username=user,
            password=password,
            timeout=15,
            banner_timeout=15,
        )
        print("[deploy] Connected.")

    def run(self, cmd: str, *, timeout: int = 600, check: bool = True) -> int:
        print(f"[remote] $ {cmd}")
        sys.stdout.flush()
        stdin, stdout, stderr = self.ssh.exec_command(cmd, timeout=timeout, get_pty=False)
        # Stream output line-by-line so long-running steps stay interactive.
        for line in iter(stdout.readline, ""):
            line = line.rstrip()
            if line:
                print(f"  {line}")
        err = stderr.read().decode(errors="replace").strip()
        if err:
            for line in err.splitlines():
                if "[sudo]" in line or "password for" in line.lower():
                    continue
                print(f"  ! {line}")
        rc = stdout.channel.recv_exit_status()
        if check and rc != 0:
            raise RuntimeError(f"remote command failed (exit {rc}): {cmd}")
        return rc

    def sudo(self, cmd: str, *, timeout: int = 600, check: bool = True) -> int:
        wrapped = f"echo {self.password} | sudo -S bash -c {shell_quote(cmd)}"
        return self.run(wrapped, timeout=timeout, check=check)

    def upload(self, local: Path, remote_path: str) -> None:
        print(f"[deploy] Uploading {local.name} -> {remote_path}")
        sftp = self.ssh.open_sftp()
        try:
            sftp.put(cygpath_w(local), remote_path)
        finally:
            sftp.close()

    def close(self) -> None:
        self.ssh.close()


def shell_quote(s: str) -> str:
    """Single-quote a string for safe inclusion in a remote bash command."""
    return "'" + s.replace("'", "'\"'\"'") + "'"


def main() -> int:
    args = parse_args()
    remote_path = args.remote_path or f"/home/{args.user}/UAV_Robot"

    root = project_root()
    if not (root / "Makefile").exists():
        print(f"[deploy] ERROR: project root has no Makefile: {root}", file=sys.stderr)
        return 1

    tar_path = make_tarball(root)

    r = Remote(args.host, args.port, args.user, args.password)
    try:
        # ── 1. Sync source ────────────────────────────────────────────────
        r.run(f"mkdir -p {remote_path}")
        r.upload(tar_path, "/tmp/uav_robot_deploy.tar")
        r.run(f"tar -xf /tmp/uav_robot_deploy.tar -C {remote_path}")
        r.run("rm -f /tmp/uav_robot_deploy.tar")
        r.run(f"chmod +x {remote_path}/tools/*.sh")

        # ── 2. apt deps (optional) ────────────────────────────────────────
        if not args.skip_apt:
            print("[deploy] Installing apt build dependencies ...")
            pkgs = " ".join(APT_PACKAGES)
            r.sudo(f"DEBIAN_FRONTEND=noninteractive apt-get update -y")
            r.sudo(
                f"DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends {pkgs}"
            )

        # ── 3. RealSense SDK (optional, one-time) ─────────────────────────
        if not args.skip_realsense:
            print("[deploy] Installing Intel RealSense SDK (idempotent) ...")
            r.sudo(
                f"bash {remote_path}/tools/realsense_install.sh",
                check=False,  # don't fail entire deploy if optional camera setup fails
            )

        # ── 4. Stop everything (clean leftovers from any previous run) ────
        # The target may already have UAV_Robot processes running, possibly
        # from an older install path. stop_all.sh kills them by binary
        # name and verifies they're really gone before we install fresh.
        print("[deploy] Stopping any leftover UAV_Robot services ...")
        r.sudo(
            f"bash {remote_path}/tools/stop_all.sh",
            check=False,  # OK if nothing was running
            timeout=120,
        )

        # ── 5. Build + systemd install + start ────────────────────────────
        print("[deploy] Running install_autostart.sh on target ...")
        r.sudo(
            f"cd {remote_path} && ./tools/install_autostart.sh --services {args.services}",
            timeout=1800,
        )

        # ── 6. Verify everything came up ──────────────────────────────────
        # Pass --services so we only verify the subset that was actually
        # built/installed (avoids spurious failures from leftover units of
        # services that weren't part of this deploy).
        print("[deploy] Verifying services started ...")
        rc = r.sudo(
            f"bash {remote_path}/tools/start_all.sh --services {args.services}",
            check=False,
            timeout=180,
        )
        if rc != 0:
            print("[deploy] WARN: start_all.sh reported failures - see output above")
    finally:
        r.close()
        try:
            tar_path.unlink()
        except OSError:
            pass

    print()
    print("=" * 60)
    print(f"[deploy] Deploy completed for {args.user}@{args.host}")
    print("=" * 60)
    print("Next steps on the target:")
    print(f"  ssh {args.user}@{args.host}")
    print(f"  cd {remote_path}")
    print("  ./tools/stop_all.sh    # one-click stop")
    print("  ./tools/start_all.sh   # one-click start")
    return 0


if __name__ == "__main__":
    sys.exit(main())
