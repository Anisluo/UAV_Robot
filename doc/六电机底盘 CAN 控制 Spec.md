六电机底盘 CAN 控制 Spec（RoboModule_DRV）
1. Scope

在 RK3588 Linux 上通过 SocketCAN（can1，1 Mbps）向 6 个驱动器发送标准帧指令，实现差速底盘的速度控制、启停、转向。协议为 RoboModule_DRV（来自厂商 STM32 参考实现/协议说明）。

2. Physical Layer

Bus: Classical CAN, Standard 11-bit ID（is_extended_id = false）

Bitrate: 1,000,000 bps

Recommended wiring:

CANH / CANL 必须为双绞线

建议共地：驱动器 24V 0V 与 CAN 侧 GND/Shield 同参考（显著提升稳定性）

终端：总线两端各 120Ω（断电测 CANH–CANL ≈ 60Ω为理想）

3. Node Addressing
3.1 Arbitration ID 编码（11-bit）
CAN_ID = (group << 8) | (number << 4) | cmd
group  : 0..7
number : 0..15
cmd    : 0..15（低 4-bit 命令码）
3.2 实际电机编号映射（已验证）

编号顺序（number）：

1: 左前 LF

2: 左后 LR

3: 右后 RR

4: 右前 RF

5: 左中 LM

6: 右中 RM

底盘分组：

Left set: [1(LF), 5(LM), 2(LR)]

Right set: [4(RF), 6(RM), 3(RR)]

备注：右侧物理安装导致方向相反，需要方向修正（见 6.2）。

4. Frame Format

DLC: 固定 8 bytes

Data: 按命令定义，未用字节填充 0x55

5. Command Set（关键控制指令）
5.1 Reset（复位/清状态）

cmd = 0x0

Data = 55 55 55 55 55 55 55 55

Purpose:

清理驱动器状态（部分故障/状态可能需要断电才可清）

初始化阶段建议先 reset

5.2 Mode Select（模式选择）

cmd = 0x1

Data:

data[0] = mode

data[1..7] = 0x55

Mode 值（已用并验证）：

Velocity closed-loop: 0x03

5.3 Velocity Run（速度控制）

cmd = 0x4

Data:

data[0..1] = limit_pwm（uint16 big-endian）

data[2..3] = target_rpm（int16 big-endian，补码）

data[4..7] = 0x55

Range（工程约束）：

limit_pwm ∈ [0, 5000]（建议逐步调大）

target_rpm ∈ [-4000, 4000]（按厂商约束）

6. Chassis Control Semantics
6.1 差速控制接口（逻辑层 API）

定义控制接口：

set_lr(left_rpm, right_rpm, limit_pwm)

左侧 3 电机发送 target_rpm = left_rpm

右侧 3 电机发送 target_rpm = right_rpm

6.2 方向修正（右侧反向）

由于右侧轮组方向相反，必须进行符号修正：

left_cmd = +left_rpm

right_cmd = -right_rpm（RIGHT_SIGN = -1）

若未来出现单个轮方向异常，升级为 per-motor dir_map：
dir_map = {1:+1,2:+1,5:+1,4:-1,6:-1,3:-1}

6.3 推荐初始化序列（必须）

串行初始化（避免同时突发导致驱动器异常/蜂鸣）：

对每个电机依次发送 Reset

延时 100~200 ms

对每个电机依次发送 ModeSelect(Velocity=0x03)

延时 100~200 ms

6.4 推荐运行策略（必须）

软启动/软停止（ramp）：禁止 0→大 rpm 的阶跃输入（闭环速度模式会瞬时打满输出）

刷新频率建议：

5~10 Hz（100~200 ms 一次）即可稳定

帧间隔（inter-frame gap）：

建议 3~5 ms（避免 burst 压垮驱动器处理/SocketCAN TX 队列）

7. Linux / SocketCAN Requirements
7.1 Interface config

up + bitrate：

ip link set can1 down

ip link set can1 type can bitrate 1000000 restart-ms 100

ip link set can1 up

7.2 TX queue

为避免 ENOBUFS (No buffer space)，建议：

ip link set can1 txqueuelen 1000

7.3 典型错误与含义

OSError: [Errno 100] Network is down

can1 未 up / 设备拔插后未重新配置

OSError: [Errno 105] No buffer space available

发送过快、txqueuelen 太小、或 socket 未及时消费

解决：降低刷新频率、加帧间隔、提高 txqueuelen

8. Debug Checklist（调试注意事项）
8.1 蜂鸣器报警（最关键）

现象：多电机联动时蜂鸣、停止响应，重上电恢复
结论：驱动器进入保护/锁存（常见：欠压/过流/堵转/通信异常）。
对策：

先降参数：低 rpm、低 limit_pwm

必须 ramp

检查 24V 母线压降（驱动器端测 Vmin）

必要时加母线电容/提升电源瞬态能力

8.2 物理层稳定性

必做：断电测 CANH–CANL 电阻（≈60Ω 理想）

建议：共地（24V 0V ↔ CAN GND/Shield）

建议：避免长分叉（stub）与星形拓扑

8.3 抓包验证

candump -tz can1 可直接看到发出的帧（RX=0 不代表无 ACK，ACK 不会上报为数据帧）

ip -details -statistics link show can1 观察 bus-errors / error-pass / bus-off 是否增长

9. Minimal Acceptance Test（验收标准）

单电机：Reset→VelocityMode→Run→Stop 正常

左 2、左 3：可重复稳定运行，不触发蜂鸣

六电机：

低速低 PWM 稳定

ramp 到目标速度稳定

直行/原地旋转方向正确（右侧取反后）