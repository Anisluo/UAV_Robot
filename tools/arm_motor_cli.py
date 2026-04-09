#!/usr/bin/env python3
import os
import shlex
import subprocess
import sys


DEFAULT_BIN = os.environ.get("UAV_ARM_TEST_BIN", "./build/bin/arm_motor_test")
DEFAULT_JOINT = int(os.environ.get("UAV_ARM_JOINT", "1"))
DEFAULT_HOME_MODE = int(os.environ.get("UAV_ARM_HOME_MODE", "2"))


def joint_ratio_env_name(joint):
    return f"UAV_ARM_J{joint}_RATIO"


def ensure_single_axis_defaults(joint):
    env_name = joint_ratio_env_name(joint)
    if joint == 1 and not os.environ.get(env_name):
        os.environ[env_name] = "1"


def get_joint_ratio(joint):
    return os.environ.get(joint_ratio_env_name(joint), "(default)")


def get_env_value(name, fallback):
    return os.environ.get(name, fallback)


def run_cmd(args):
    cmd = [DEFAULT_BIN] + args
    print("\n$", " ".join(shlex.quote(part) for part in cmd))
    result = subprocess.run(cmd)
    return result.returncode


def print_help():
    print(
        "Commands:\n"
        "  home              home current joint\n"
        "  zero              write zero params for current joint\n"
        "  estop             stop current joint and trigger emergency hook\n"
        "  stop              send stop\n"
        "  joint <n>         change active joint\n"
        "  ratio <x>         set active joint ratio in env\n"
        "  rpm <n>           set move rpm in env\n"
        "  acc <n>           set move acceleration in env\n"
        "  zero-rpm <n>      set homing zero speed rpm in env\n"
        "  home-detect-rpm <n> set collision detect speed rpm in env\n"
        "  mode <n>          change home mode\n"
        "  goto <deg>        move active joint to absolute angle\n"
        "  <deg>             same as goto <deg>\n"
        "  help              show this help\n"
        "  quit              exit\n"
    )


def main():
    joint = DEFAULT_JOINT
    home_mode = DEFAULT_HOME_MODE

    ensure_single_axis_defaults(joint)
    print("arm_motor_cli")
    print(f"  iface={os.environ.get('UAV_ARM_CAN_IFACE', '(default)')}")
    print(f"  binary={DEFAULT_BIN}")
    print(
        "  joint={} home_mode={} ratio={} rpm={} acc={} zero_rpm={} home_detect_rpm={}".format(
            joint,
            home_mode,
            get_joint_ratio(joint),
            get_env_value("UAV_ARM_RPM", "300"),
            get_env_value("UAV_ARM_ACC", "20"),
            get_env_value("UAV_ARM_ZERO_SPEED_RPM", "30"),
            get_env_value("UAV_ARM_COLLISION_ZERO_SPEED_RPM", "300"),
        )
    )
    print_help()

    while True:
        try:
            raw = input(f"\narm[j{joint}]> ").strip()
        except EOFError:
            print()
            return 0
        except KeyboardInterrupt:
            print("\nInterrupted")
            return 130

        if not raw:
            continue

        parts = raw.split()
        cmd = parts[0].lower()

        if cmd in ("quit", "exit", "q"):
            return 0
        if cmd in ("help", "h", "?"):
            print_help()
            continue
        if cmd == "joint":
            if len(parts) != 2:
                print("usage: joint <n>")
                continue
            try:
                joint = int(parts[1])
            except ValueError:
                print("invalid joint")
                continue
            ensure_single_axis_defaults(joint)
            print(f"active joint -> {joint} ratio={get_joint_ratio(joint)}")
            continue
        if cmd == "ratio":
            if len(parts) != 2:
                print("usage: ratio <x>")
                continue
            try:
                ratio = float(parts[1])
            except ValueError:
                print("invalid ratio")
                continue
            os.environ[joint_ratio_env_name(joint)] = f"{ratio:g}"
            print(f"joint {joint} ratio -> {get_joint_ratio(joint)}")
            continue
        if cmd == "mode":
            if len(parts) != 2:
                print("usage: mode <n>")
                continue
            try:
                home_mode = int(parts[1])
            except ValueError:
                print("invalid mode")
                continue
            print(f"home mode -> {home_mode}")
            continue
        if cmd == "rpm":
            if len(parts) != 2:
                print("usage: rpm <n>")
                continue
            os.environ["UAV_ARM_RPM"] = parts[1]
            print(f"rpm -> {get_env_value('UAV_ARM_RPM', '300')}")
            continue
        if cmd == "acc":
            if len(parts) != 2:
                print("usage: acc <n>")
                continue
            os.environ["UAV_ARM_ACC"] = parts[1]
            print(f"acc -> {get_env_value('UAV_ARM_ACC', '20')}")
            continue
        if cmd == "zero-rpm":
            if len(parts) != 2:
                print("usage: zero-rpm <n>")
                continue
            os.environ["UAV_ARM_ZERO_SPEED_RPM"] = parts[1]
            print(f"zero_rpm -> {get_env_value('UAV_ARM_ZERO_SPEED_RPM', '30')}")
            continue
        if cmd == "home-detect-rpm":
            if len(parts) != 2:
                print("usage: home-detect-rpm <n>")
                continue
            os.environ["UAV_ARM_COLLISION_ZERO_SPEED_RPM"] = parts[1]
            print(f"home_detect_rpm -> {get_env_value('UAV_ARM_COLLISION_ZERO_SPEED_RPM', '300')}")
            continue
        if cmd == "home":
            code = run_cmd(["home-joint", str(joint), str(home_mode)])
            print(f"exit={code}")
            continue
        if cmd == "zero":
            code = run_cmd(["set-zero-params", str(joint)])
            print(f"exit={code}")
            continue
        if cmd == "estop":
            code = run_cmd(["estop-joint", str(joint)])
            print(f"exit={code}")
            continue
        if cmd == "stop":
            code = run_cmd(["stop"])
            print(f"exit={code}")
            continue
        if cmd == "goto":
            if len(parts) != 2:
                print("usage: goto <deg>")
                continue
            angle_text = parts[1]
        else:
            angle_text = raw

        try:
            angle = float(angle_text)
        except ValueError:
            print("unknown command, type help")
            continue

        code = run_cmd(["move-joint", str(joint), str(angle)])
        print(f"exit={code}")


if __name__ == "__main__":
    sys.exit(main())
