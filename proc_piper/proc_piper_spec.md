# proc_piper 技术规格 (UAV_Robot gen-2)

## 用途

替代 `proc_arm` + `proc_gripper`, 通过 [piper_sdk](https://github.com/agilexrobotics/piper_sdk) 驱动 **AgileX Piper 6-DOF 机械臂**(含一体化夹爪)。

## 架构

```
HostGUI (PiperWidget) / arm CLI
        │ JSON-RPC (Unix socket)
        ▼
    proc_piper                ← proc_piper.py
        │  • RPC server (asyncio-free, threading)
        │  • 200Hz CAN 心跳后台线程
        │  • V1.8-2 固件握手 (MasterSlave → STANDBY → CAN_CTRL → Enable)
        ▼
    piper_sdk (Python)
        │ SocketCAN, 1 Mbps
        ▼
    can_piper (candleLight USB-CAN) → Piper 控制器
```

## 通信接口

| 接口 | 路径 | 协议 |
|---|---|---|
| 控制 socket | `/tmp/uav_proc_piper.sock` | JSON-RPC 2.0, 换行分隔 |
| CAN 总线 | `can_piper` (env `UAV_PIPER_CAN_IFACE`) | SocketCAN, 1 Mbps |

## 关节物理限制

| 关节 | 范围 (deg) | 备注 |
|---|---|---|
| J1 | ±150° | 底座 |
| J2 | [0°, +180°] | **下限 0° 是软限位**, 上电后默认在 0° |
| J3 | [-170°, 0°] | **上限 0° 是软限位** |
| J4 | ±100° | |
| J5 | ±70° | |
| J6 | ±180° | |
| 夹爪 | 0-80 mm | 连续 |

## JSON-RPC 方法

### 镜像 proc_arm (供 gateway / HostGUI 直接复用)

| 方法 | 参数 | 返回 | 备注 |
|---|---|---|---|
| `system.ping` | - | `{ok, backend:"piper"}` | |
| `arm.home` | - | `{ok}` | MoveJ 到 `[0]*6` 安全休眠位 |
| `arm.home_joint` | `{joint_index}` | `{ok}` | 单关节归零, 其他不动 |
| `arm.stop` | - | `{ok}` | 把 target 设回当前读数 |
| `arm.emergency_stop` | `{enable}` | `{ok}` | enable=true 急停, false 恢复并重做握手 |
| `arm.set_free_mode` | `{enable}` | `{ok:false, error}` | **Piper 拖动示教只能用机身物理按钮**, 不支持 CAN |
| `arm.set_speeds` | `{joint_dps?, move_rpm?}` | `{ok, joint_dps, ...}` | dps→speed_pct 启发式映射 |
| `arm.get_speeds` | - | `{ok, joint_dps, ...}` | |
| `arm.move_joint` | `{joint_index, target_deg}` | `{ok}` | |
| `arm.move_joints` | `{joint_1..joint_6 \| angles[6], speed_ratio?}` | `{ok, eta_s}` | |
| `arm.move_xyz` | `{x_mm, y_mm, z_mm}` | `{ok, eta_s}` | MOVE_P, 保持当前 RX/RY/RZ |
| `arm.move_pose6d` | `{x_mm, y_mm, z_mm, roll_deg, pitch_deg, yaw_deg, speed_ratio?, rotation_order?}` | `{ok, eta_s}` | MOVE_P |
| `arm.move_linear_xyz_rotation` | 同上 | `{ok, eta_s}` | MOVE_L |
| `arm.get_motor_angles` | - | `[j1..j6]` (deg) | |
| `arm.get_pose` | `{rotation_order?}` | `[x,y,z,r,p,y]` (mm / deg) | |
| `arm.get_transform` | - | 16-elem 单位矩阵 | **未实现**, 返回 identity |
| `arm.set_current_zero` | `{joint_index}` | `{ok:false, error}` | Piper 不支持软件零偏 |
| `arm.reset_current_zero` | `{joint_index}` | `{ok:false, error}` | 同上 |
| `gripper.set` | `{open}` | `{ok}` | open=true → 80mm, false → 0mm |
| `arm.servo_gripper` | `{angle_deg}` | `{ok, eta_s}` | 0-90° 线性映射到 0-80mm |

### Piper 专用扩展 (`piper.*`)

| 方法 | 参数 | 返回 | 备注 |
|---|---|---|---|
| `piper.handshake` | - | `{ok}` | 重做 STANDBY→CAN_CTRL 握手 |
| `piper.park_zero` | - | `{ok}` | 同 `arm.home`, 显式别名 |
| `piper.get_status` | - | `{ctrl_mode, arm_status, mode_feed, teach_status, motion_status, error_code, heartbeat_alive, speed_pct}` | |
| `piper.set_gripper_angle` | `{angle_mm, effort_mNm?}` | `{ok, angle_mm}` | 连续夹爪 0-80mm |
| `piper.move_cartesian` | `{X_mm, Y_mm, Z_mm, RX_deg, RY_deg, RZ_deg, mode?}` | `{ok}` | mode='P'\|'L' (默认 P) |
| `system.get_backend` | - | `{backend:"piper", model}` | gateway / HostGUI 探测用 |

## V1.8-2 固件握手 (关键, 不可省)

```
MasterSlaveConfig(0xFC)           # 从臂模式
MotionCtrl_2(0x00, ...)            # ★ 显式 STANDBY
MotionCtrl_2(0x01, 0x01, spd, 0)  # CAN_CTRL + MOVE_J
EnableArm(7, 0x02)                 # 全部使能
```

不做这个序列, `ctrl_mode` 会卡在 TEACHING(2), JointCtrl 完全无效。
[详见 memory/piper_can_ctrl_handshake.md](https://github.com/Anisluo/UAV_Robot)

## 安全关机

`SIGTERM` / `SIGINT` 收到后:
1. 把 target 设为 `[0]*6` (Piper 自然休眠位)
2. 等待 ≤ 8s 收敛
3. 停心跳线程
4. **不调 DisableArm** —— 电机继续保持零位 (用户偏好)

## 环境变量

```bash
UAV_PIPER_CAN_IFACE="can_piper"          # SocketCAN 接口名
UAV_PIPER_SPEED="30"                       # 默认运动速度百分比 (1-100)
UAV_PROC_PIPER_SOCK="/tmp/uav_proc_piper.sock"  # RPC socket 路径
UAV_PROC_PIPER_ARGS=""                     # 额外命令行参数
```

## CLI 调试

```bash
# 启动 (开发时手动跑)
python3 proc_piper/proc_piper.py --iface can_piper --speed 30

# JSON-RPC 客户端测试
echo '{"jsonrpc":"2.0","id":1,"method":"system.ping"}' | nc -U /tmp/uav_proc_piper.sock
echo '{"jsonrpc":"2.0","id":2,"method":"arm.get_motor_angles"}' | nc -U /tmp/uav_proc_piper.sock
echo '{"jsonrpc":"2.0","id":3,"method":"arm.move_joint","params":{"joint_index":1,"target_deg":15}}' | nc -U /tmp/uav_proc_piper.sock
echo '{"jsonrpc":"2.0","id":4,"method":"piper.get_status"}' | nc -U /tmp/uav_proc_piper.sock
```

## 服务管理

```bash
# 启用 gen-2 (默认)
sudo ./tools/install_autostart.sh --backend piper

# 切回 gen-1 (ZDT)
sudo ./tools/install_autostart.sh --backend legacy

# 只装 proc_piper
sudo ./tools/install_autostart.sh --services proc_piper --skip-deps

# 看日志
sudo journalctl -u uav-proc-piper.service -f
```

## 已知限制

| 限制 | 原因 | 影响 |
|---|---|---|
| `arm.get_transform` 返回 identity | piper_sdk 没暴露 T 矩阵 | HostGUI 若依赖 T 矩阵需用 `get_pose` 自己算 |
| `arm.set_free_mode` 返回 error | Piper 拖动示教仅物理按钮触发 | HostGUI 应捕获并提示用户按按钮 |
| `arm.set_current_zero` 返回 error | Piper 没有软件零偏 | 不适用于 Piper |
| `arm.servo_gripper` 角度→mm 线性映射 | 协议差异 | 实际抓握建议用 `piper.set_gripper_angle` |
| 速度单位是百分比 1-100, 不是 RPM | piper_sdk 限制 | 设速时启发式映射, 精度有损 |
