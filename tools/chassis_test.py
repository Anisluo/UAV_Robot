#!/usr/bin/env python3
"""
六电机底盘 CAN 控制测试脚本
协议：RoboModule_DRV  接口：can3 @ 1MHz
用法：sudo python3 chassis_test.py [选项]

选项：
  --rpm N        目标转速 rpm (默认 100)
  --pwm P        PWM 限幅 (默认 800)
  --time T       匀速阶段时间 s (默认 0.3)
  --ramp-steps N 软启动步数 (默认 4)
  --ramp-dt T    软启动每步间隔 s (默认 0.05)

速度标定（实测 @200rpm 速度约 4m/s）：
  1m ≈ --rpm 100 --time 0.3 --ramp-steps 4 --ramp-dt 0.05
"""

import socket
import struct
import time
import sys
import argparse

# ─── 默认参数 ─────────────────────────────────────────────────────────────────
CAN_IFACE  = 'can3'
GROUP      = 0
DEFAULT_RPM        = 100    # 目标转速 rpm（~2m/s）
DEFAULT_PWM        = 800    # PWM 限幅 [0,5000]
DEFAULT_TIME       = 0.3    # 匀速阶段时间(s)
DEFAULT_RAMP_STEPS = 4      # 软启动步数
DEFAULT_RAMP_DT    = 0.05   # 每步间隔(s) → ramp 约 0.2s

REFRESH_DT     = 0.15       # 速度刷新周期(s)
INTER_FRAME_DT = 0.004      # 帧间隔(s)

# ─── 电机布局与方向 ───────────────────────────────────────────────────────────
#  左侧：number 1(LF) 5(LM) 2(LR)  direction +1
#  右侧：number 4(RF) 6(RM) 3(RR)  direction -1（物理反向）
MOTOR_DIR = {1: +1, 2: +1, 5: +1, 4: -1, 6: -1, 3: -1}
LEFT_MOTORS  = [1, 5, 2]   # LF LM LR
RIGHT_MOTORS = [4, 6, 3]   # RF RM RR

# ─── SocketCAN 帧封装 ─────────────────────────────────────────────────────────
CAN_FRAME_FMT = '=IB3x8s'   # can_id(u32), dlc(u8), pad(3), data(8)

def open_can(iface):
    s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    s.bind((iface,))
    return s

def send_frame(sock, cid, data8):
    d = bytes(data8)
    assert len(d) == 8
    sock.send(struct.pack(CAN_FRAME_FMT, cid & 0x7FF, 8, d))

def make_can_id(number, cmd):
    return (GROUP << 8) | (number << 4) | (cmd & 0xF)

# ─── 指令构造 ─────────────────────────────────────────────────────────────────
def cmd_reset(sock, motor_num):
    send_frame(sock, make_can_id(motor_num, 0x0), [0x55]*8)

def cmd_mode(sock, motor_num, mode=0x03):
    send_frame(sock, make_can_id(motor_num, 0x1), [mode] + [0x55]*7)

def cmd_velocity(sock, motor_num, rpm, limit_pwm=DEFAULT_PWM):
    actual = int(rpm * MOTOR_DIR[motor_num])
    data = (list(struct.pack('>H', limit_pwm)) +
            list(struct.pack('>h', actual)) +
            [0x55]*4)
    send_frame(sock, make_can_id(motor_num, 0x4), data)

# ─── 底盘级操作 ───────────────────────────────────────────────────────────────
def chassis_reset(sock):
    print("[INIT] Reset all motors...")
    for n in range(1, 7):
        cmd_reset(sock, n)
        time.sleep(INTER_FRAME_DT)
    time.sleep(0.15)

def chassis_set_mode(sock, mode=0x03):
    print("[INIT] Set velocity-closed-loop mode (0x03)...")
    for n in range(1, 7):
        cmd_mode(sock, n, mode)
        time.sleep(INTER_FRAME_DT)
    time.sleep(0.15)

def chassis_set_lr(sock, left_rpm, right_rpm, limit_pwm=DEFAULT_PWM):
    for n in LEFT_MOTORS:
        cmd_velocity(sock, n, left_rpm, limit_pwm)
        time.sleep(INTER_FRAME_DT)
    for n in RIGHT_MOTORS:
        cmd_velocity(sock, n, right_rpm, limit_pwm)
        time.sleep(INTER_FRAME_DT)

def chassis_stop(sock):
    chassis_set_lr(sock, 0, 0, limit_pwm=0)

def ramp(sock, from_rpm, to_rpm, limit_pwm=DEFAULT_PWM,
         steps=DEFAULT_RAMP_STEPS, dt=DEFAULT_RAMP_DT):
    for i in range(1, steps + 1):
        rpm = from_rpm + (to_rpm - from_rpm) * i / steps
        chassis_set_lr(sock, rpm, rpm, limit_pwm)
        time.sleep(dt)

def run_straight(sock, rpm, duration, label, limit_pwm=DEFAULT_PWM,
                 ramp_steps=DEFAULT_RAMP_STEPS, ramp_dt=DEFAULT_RAMP_DT):
    t_ramp = ramp_steps * ramp_dt
    print(f"[RUN ] {label}  rpm={rpm}  匀速={duration:.2f}s  ramp={t_ramp:.2f}s  pwm={limit_pwm}")
    ramp(sock, 0, rpm, limit_pwm, ramp_steps, ramp_dt)
    elapsed = 0.0
    while elapsed < duration:
        chassis_set_lr(sock, rpm, rpm, limit_pwm)
        time.sleep(REFRESH_DT)
        elapsed += REFRESH_DT
    ramp(sock, rpm, 0, limit_pwm, ramp_steps, ramp_dt)
    chassis_stop(sock)
    print(f"[STOP] {label} done")
    time.sleep(0.5)

# ─── 主流程 ───────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description='底盘前进/后退测试')
    parser.add_argument('--rpm',        type=int,   default=DEFAULT_RPM,        help=f'目标转速 rpm (默认 {DEFAULT_RPM})')
    parser.add_argument('--pwm',        type=int,   default=DEFAULT_PWM,        help=f'PWM 限幅 (默认 {DEFAULT_PWM})')
    parser.add_argument('--time',       type=float, default=DEFAULT_TIME,       help=f'匀速阶段时间 s (默认 {DEFAULT_TIME})')
    parser.add_argument('--ramp-steps', type=int,   default=DEFAULT_RAMP_STEPS, help=f'软启动步数 (默认 {DEFAULT_RAMP_STEPS})')
    parser.add_argument('--ramp-dt',    type=float, default=DEFAULT_RAMP_DT,    help=f'软启动步间隔 s (默认 {DEFAULT_RAMP_DT})')
    args = parser.parse_args()

    TARGET_RPM  = args.rpm
    LIMIT_PWM   = args.pwm
    MOVE_TIME   = args.time
    RAMP_STEPS  = args.ramp_steps
    RAMP_DT_ARG = args.ramp_dt
    t_ramp = RAMP_STEPS * RAMP_DT_ARG

    est_dist = (TARGET_RPM / 200.0 * 4.0) * (t_ramp + MOVE_TIME)
    print(f"=== 底盘测试  接口={CAN_IFACE}  rpm={TARGET_RPM}  pwm={LIMIT_PWM}  "
          f"匀速={MOVE_TIME:.2f}s  ramp={t_ramp:.2f}s  预估距离≈{est_dist:.1f}m ===\n")
    print("提示：如果电机蜂鸣/不响应，降低 --rpm 和 --pwm\n")

    try:
        sock = open_can(CAN_IFACE)
    except Exception as e:
        print(f"[ERR ] 打开 {CAN_IFACE} 失败: {e}")
        sys.exit(1)

    try:
        chassis_reset(sock)
        chassis_set_mode(sock)

        run_straight(sock, TARGET_RPM,  MOVE_TIME, "前进", LIMIT_PWM, RAMP_STEPS, RAMP_DT_ARG)

        print("[WAIT] 1s 间隔...")
        time.sleep(1.0)

        run_straight(sock, -TARGET_RPM, MOVE_TIME, "后退", LIMIT_PWM, RAMP_STEPS, RAMP_DT_ARG)

        print("[DONE] 测试完成，最终停止")
        chassis_stop(sock)

    except KeyboardInterrupt:
        print("\n[ABORT] 用户中断，紧急停止")
        chassis_stop(sock)
    finally:
        sock.close()

if __name__ == '__main__':
    main()
