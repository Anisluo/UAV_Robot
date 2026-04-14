# proc_arm 技术规格

## 架构

```
HostGUI / arm_motor_cli.py
        │ JSON-RPC (Unix socket)
        ▼
    proc_arm          ← arm_controller.cpp (IK/FK, 限位, 缓存)
        │                 arm_runtime.cpp   (JSON-RPC server)
        │ dev.h API
        ▼
    arm.c             ← ZDT CAN 协议驱动 (proto_zdt_arm)
        │ SocketCAN
        ▼
    can1 (PCAN-USB)   → ZDT Emm V5.0 步进电机
```

## 通信接口

| 接口 | 路径 | 协议 |
|------|------|------|
| 控制 socket | `/tmp/uav_proc_arm.sock` | JSON-RPC 2.0, 换行分隔 |
| CAN 总线 | `can1` (env `UAV_ARM_CAN_IFACE`) | SocketCAN, 500kbps |
| CAN 帧格式 | 扩展帧 (29-bit), ID = `addr << 8` | ZDT Emm V5.0 协议 |

## ZDT CAN 协议

校验字节固定 `0x6B`。CAN ID = `motor_addr << 8 | frame_index` (扩展帧)。

| 命令 | 字节 | 载荷 | 说明 |
|------|------|------|------|
| 使能 | `0xF3` | `F3 AB 01/00 sync 6B` | 01=使能, 00=去使能 |
| 位置 | `0xFD` | `FD dir rpm_H rpm_L acc pulse3..0 abs sync 6B` | 12字节, 分2帧发 |
| 停止 | `0xFE` | `FE 98 sync 6B` | |
| 同步启动 | `0xFF` | `FF 66 6B` | 广播, addr=0 |
| 触发回零 | `0x9A` | `9A mode sync 6B` | mode=2 堵转检测 |
| 写零点参数 | `0x4C` | 19字节, 分3帧 | 含方向/速度/电流/超时 |
| 速度模式 | `0xF6` | `F6 dir rpm_H rpm_L acc sync 6B` | |
| **读编码器** | `0x30` | TX: `30 6B` / RX: `30 sign carry_H carry_L enc_H enc_L 6B` | 脉冲=carry*65536+enc |

## 堵转回零流程

```
1. 使能电机 (0xF3)        → 等 ACK
2. 写零点参数 (0x4C)      → 等 ACK (mode=2 堵转, dir=1 反转)
3. 触发回零 (0x9A mode=2) → 等 ACK
4. 等待 settle (默认 2.5s)  → 电机堵转到物理极限, 该位置设为 0°
5. 移动到安全位置           → post_home_deg
```

堵转回零后运动范围为 **0° ~ 360°** (单方向)。

## 关节参数

| 参数 | J1 | J2 | J3 | J4 | J5 | J6 |
|------|-----|-----|-----|-----|-----|-----|
| 减速比 | 25 | 20 | 25 | 10 | 4 | 1 |
| 最小角度 | 0° | 0° | 0° | 0° | 0° | 0° |
| 最大角度 | 360° | 180° | 163° | 335° | 220° | 335° |
| 安全位置 | 180° | 90° | 83° | 30° | 110° | 30° |
| CAN 地址 | 1 | 2 | 3 | 4 | 5 | 6 |

脉冲计算: `pulses = deg / 360 × 3200 × ratio × dir_sign`

## JSON-RPC 方法

### 运动控制
| 方法 | 参数 | 返回 |
|------|------|------|
| `arm.move_joint` | `joint_index`, `target_deg` | `{ok, eta_s}` |
| `arm.move_pose6d` | `x/y/z_mm`, `roll/pitch/yaw_deg`, `speed_ratio` | `{ok, eta_s}` |
| `arm.move_xyz` | `x/y/z_mm` | `{ok, eta_s}` |
| `arm.home` | - | bool |
| `arm.home_joint` | `joint_index` | bool |
| `arm.stop` | - | bool |
| `arm.emergency_stop` | `enable` | bool |

### 状态查询
| 方法 | 返回 |
|------|------|
| `arm.get_motor_angles` | `[j1..j6]` (真实编码器角度, 0x30 CAN 读取) |
| `arm.get_pose` | `[x,y,z,roll,pitch,yaw]` |
| `arm.get_speeds` | `{move_rpm, zero_rpm}` |
| `system.ping` | `{ok:true}` |

### 设置
| 方法 | 参数 |
|------|------|
| `arm.set_speeds` | `move_rpm`, `zero_rpm` |
| `arm.set_current_zero` | `joint_index` |
| `arm.reset_current_zero` | `joint_index` |
| `arm.servo_gripper` | `angle_deg` |

## 环境变量 (/etc/default/uav_robot)

```bash
UAV_ARM_CAN_IFACE=can1              # CAN 接口
UAV_ARM_RPM=300                      # 运动速度 (1-3000)
UAV_ARM_ACC=20                       # 加速度 (0-255)
UAV_ARM_ZERO_SPEED_RPM=30            # 回零速度
UAV_ARM_HOME_MODE=2                  # 0=限位开关 1=双限位 2=堵转 3=闭环
UAV_ARM_HOME_SETTLE_MS=2500          # 堵转后等待时间
UAV_ARM_COLLISION_ZERO_CURRENT_MA=800 # 堵转检测电流阈值
UAV_ARM_JOINT_COUNT_CONNECTED=1      # 已连接关节数 (避免未接关节超时)
UAV_ARM_J<n>_RATIO=25                # 各轴减速比
UAV_ARM_J<n>_MIN_DEG=0               # 各轴最小角度
UAV_ARM_J<n>_MAX_DEG=360             # 各轴最大角度
UAV_ARM_J<n>_POST_HOME_DEG=180       # 各轴回零后安全位置
```

## arm_motor_cli.py 使用

```bash
sudo ./tools/arm_motor_cli.py
```

交互命令:

| 命令 | 说明 |
|------|------|
| `home` | 当前关节堵转回零 |
| `<deg>` 或 `goto <deg>` | 移动到绝对角度 |
| `angles` | 读取真实编码器角度 |
| `stop` | 停止所有关节 |
| `estop` | 急停 |
| `joint <n>` | 切换当前操作关节 (1-6) |
| `rpm <n>` | 设置运动速度 |
| `zero-here` | 将当前位置设为软件零点 |
| `zero-reset` | 清除软件零点 |
| `home-watch` | 回零并实时显示角度 |
| `goto-watch <deg>` | 移动并实时显示角度 |

典型测试流程:
```
arm[j1]> home                    # 堵转回零, 0° = 物理极限
arm[j1]> angles                  # 确认 0°
arm[j1]> 90                      # 转到 90°
arm[j1]> angles                  # 确认真实角度 ≈ 90°
arm[j1]> 360                     # 转到 360° (满圈)
arm[j1]> 0                       # 回到零点
```

## CAN 调试

```bash
# 监听 CAN 总线
candump can1 -t A

# 手动发使能
cansend can1 00000100#F3.AB.01.00.6B

# 手动发停止
cansend can1 00000100#FE.98.00.6B

# 读编码器位置
cansend can1 00000100#30.6B

# 检查接口状态
ip -d -s link show can1
```

CAN 硬件注意事项:
- PCAN-USB 必须拨到 **120Ω 终端电阻** 挡位
- CAN 波特率 500kbps
- 电机断电后 CAN 收发器不工作, TX 无 error 但 RX=0
- `ip link set can1 up` 必须在 proc_arm 启动之前执行
