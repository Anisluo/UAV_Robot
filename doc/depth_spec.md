# UAV_Robot Standard Host Config Spec

本文档以“原本能跑通的主机”作为标准配置基线，用于后续新主机恢复、排障和验收。

## 1. 目标主机

- OS: Ubuntu 20.04.6 LTS
- Kernel: `5.10.160`
- Arch: `aarch64`
- User: `ubuntu`
- Project path: `/home/ubuntu/UAV_Robot`

说明：
- 本文档记录的是“标准软件配置”。
- 外设是否在线，取决于运行时是否实际插入 RealSense、PEAK USB-CAN、串口设备等。

## 2. 标准服务

以下 systemd 服务应安装并启用：

- `uav-robotd.service`
- `uav-proc-gateway.service`
- `uav-proc-realsense.service`
- `uav-proc-npu.service`

默认职责：

- `uav_robotd`: 主控任务与 UDP 指令入口
- `proc_gateway`: HostGUI JSON-RPC 和视频转发入口
- `proc_realsense`: RealSense 采集进程
- `proc_npu`: NPU 推理进程

## 3. 标准端口

- HostGUI JSON-RPC: `7001/TCP`
- HostGUI 视频流: `7002/TCP`
- 主控 UDP 指令监听: `9001/UDP`

## 4. 标准环境文件

标准环境文件位置：

- `/etc/default/uav_robot`

以原主机为标准，关键配置如下：

```bash
UAV_ROBOTD_ARGS="--listen-port 9001"
UAV_PROC_REALSENSE_ARGS=""
UAV_PROC_NPU_ARGS=""
UAV_PROC_GATEWAY_ARGS=""

UAV_CAR_CAN_IFACE="can3"
UAV_CAR_TRACK_WIDTH_MM="600"
UAV_CAR_MM_PER_SEC_PER_RPM="5.0"
UAV_CAR_MIN_RPM="60"

UAV_AIRPORT_CAN_IFACE="can1"
UAV_AIRPORT_RAIL1_ADDR="1"
UAV_AIRPORT_RAIL2_ADDR="2"
UAV_AIRPORT_RAIL3_ADDR="3"
UAV_AIRPORT_RAIL_RPM="1500"
UAV_AIRPORT_RAIL_ACC="0"
UAV_AIRPORT_RAIL_PULSES_PER_MM="100"

UAV_AIRPORT_RELAY1_PATH="GPIO_1"
UAV_AIRPORT_RELAY2_PATH="GPIO_2"
UAV_AIRPORT_RELAY3_PATH="GPIO_3"
UAV_AIRPORT_GRIPPER_RELAY_PATH="GPIO_3"
UAV_AIRPORT_GRIPPER_ACTIVE_HIGH="1"

UAV_GRIPPER_UART_PATH="/dev/ttyUSB0"
```

补充约定：

- `ARM_CAN` 按项目当前恢复基线使用 `can4`
- `CAR_CAN` 使用 `can3`
- `AIRPORT_CAN` 使用 `can1`

## 5. 标准 CAN 命名约定

以历史可运行环境为标准，USB-CAN 逻辑命名应满足：

- 物理 `can1` 对应系统 `can1`
- 原系统枚举第二路对应逻辑 `can3`
- 原系统枚举第三路对应逻辑 `can4`

也就是代码运行时，最终应保证：

- `can1` -> platform / airport
- `can3` -> 小车底盘
- `can4` -> 机械臂

推荐波特率：

- `can1`: `500000`
- `can3`: `1000000`
- `can4`: `500000`

推荐开机初始化：

- `peak_usb.ko` 自动加载
- `uav-can-init.service` 自动执行 `ip link set ... up`

标准机当前在线快照补充：

- 当前在线标准机内核：`5.10.160 #12`
- 当前在线标准机实际映射：
  - `can1` -> `/sys/devices/platform/fc880000.usb/usb2/2-1/2-1.1/2-1.1.1/2-1.1.1:1.0`
  - `can3` -> `/sys/devices/platform/fc880000.usb/usb2/2-1/2-1.1/2-1.1.3/2-1.1.3:1.0`
  - `can4` -> `/sys/devices/platform/fc880000.usb/usb2/2-1/2-1.1/2-1.1.4/2-1.1.4:1.0`
- 当前在线标准机未发现以下持久化配置文件：
  - `/etc/udev/rules.d/80-uav-can-names.rules`
  - `uav-can-init.service`
- 当前在线标准机已加载 `peak_usb` 模块
- 当前在线标准机实测在线状态：
  - `can1` 为 `UP`，`500000`
  - `can3` 为 `UP`，`1000000`
  - `can4` 为 `UP`，`500000`
- 当前在线标准机抓到的一个实测快照：
  - `can1` 有 `RX/TX`
  - `can3` 有 `TX`，当次快照未见 `RX`
  - `can4` 当次快照无有效收发计数

说明：

- 上述 `RX/TX` 计数只是一时刻快照，不应单独用于判断设备是否“能正常运动”。
- 如果现场确认标准机此刻 `can1` 平台、`can3` 小车都能运动，则应以“标准机当前接线与设备状态正常”为准。
- 新主机恢复时，接口命名、物理 USB 路径、波特率应优先对齐这一在线标准机快照。

## 6. 标准 USB / 外设前提

只有在设备实际插入时，以下能力才会正常：

### 6.1 RealSense

应满足：

- `lsusb` 可见 Intel / RealSense 设备
- `rs-enumerate-devices` 能枚举到设备
- `/dev/video*` 中存在 RealSense 节点

以原主机实测正常状态为参考：

- `lsusb` 可见 `8086:0b07 Intel RealSense D435`
- `lsusb -t` 中设备已绑定 `uvcvideo`
- `v4l2-ctl --list-devices` 可列出 `Intel(R) RealSense(TM) Depth Ca`
- `/dev/video0 ~ /dev/video5` 存在
- `rs-enumerate-devices` 能返回以下关键信息：
  - Name: `Intel RealSense D435`
  - Serial Number: `317222073887`
  - Firmware Version: `5.17.0.10`
  - Recommended Firmware Version: `5.17.0.10`
  - Product Id: `0B07`
  - Product Line: `D400`
- `uav-proc-realsense.service` 可正常进入 `active (running)`

补充说明：

- 原主机实测时，设备当前链路速度显示为 `480M`
- 即使不是 `5000M`，该主机仍可正常枚举设备并启动 `proc_realsense`
- 因此新主机若失败，不能仅凭“不是 USB 3.0”就判断为唯一根因

如果未插设备，典型现象是：

- `proc_realsense: rs2 start failed: No device connected`
- `rs-enumerate-devices` 输出 `No device detected`

### 6.2 PEAK USB-CAN

应满足：

- `lsusb` 可见 `0c72:*` 设备
- `peak_usb` 模块已加载
- 系统中出现目标 `can1/can3/can4`

如果未插设备，典型现象是：

- `ls /sys/class/net` 中只有板载 `can0`
- 没有 `can1/can3/can4`

## 7. 新主机恢复时必须对齐的项

新主机应以原主机为标准，至少对齐以下内容：

1. Ubuntu 20.04 / 5.10.160 / aarch64 运行环境
2. `/home/ubuntu/UAV_Robot` 项目路径
3. 四个 systemd 服务已安装并启用
4. `/etc/default/uav_robot` 与标准值一致
5. `peak_usb.ko` 已部署并配置开机加载
6. CAN 逻辑命名恢复到 `can1/can3/can4`
7. `can1/can3/can4` 开机自动 `UP`
8. RealSense 运行库与头文件已安装
9. RKNN 运行库与头文件已安装

## 8. 当前排障结论

以本次探测结果看：

- 原主机的软件配置可作为标准基线
- 原主机在插上 RealSense 后，可正常枚举并启动 `proc_realsense`
- 标准机当前在线快照显示：
  - 四个主服务均为 `active`
  - `peak_usb` 已加载
  - `can1/can3/can4` 存在且可配置
  - `rs-enumerate-devices` 可识别 `Intel RealSense D435`
- 但若原主机当前未插外设，则无法用它验证“设备在线状态”
- 新主机恢复时，软件配置应严格对齐本文档
- 若 HostGUI 无响应，优先检查：
  - 端口 `7001/7002`
  - CAN 逻辑命名
  - `peak_usb` 是否加载
  - RealSense 是否被系统枚举
  - 外设是否实际插入

## 9. 建议验收顺序

1. `systemctl status` 检查四个服务
2. `ip link show` 检查 `can1/can3/can4`
3. `lsusb` 检查 PEAK 和 RealSense
4. `rs-enumerate-devices` 检查相机
5. HostGUI 调 `system.ping`
6. HostGUI 调 `ugv.set_velocity`
7. HostGUI 检查 `7002` 视频
