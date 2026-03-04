
UAV_ROBOT Architecture Specification v1.0
1. System Overview
1.1 System Role

uav_robot 为运行于 RK3588 的机器人控制节点，负责：

执行外部控制指令

管理机器人任务流程

控制多总线执行设备

提供状态反馈与安全控制

系统采用 事件驱动（Reactor + Router）架构。

1.2 Controlled Devices
Device	Interface	Binding
六轴机械臂	CAN	can1
平台固定装置	CAN	can2
六轮小车	RS485	tty*
平行夹爪	GPIO	gpiochipX
D435 相机	USB	librealsense
Mesh通信	Ethernet	UDP Port
2. Architecture Layers

系统采用三层结构：

APP
CORE
DRV

调用方向必须单向：

APP → CORE → DRV

禁止反向依赖。

3. DRV Layer (Device Adaptation)
3.1 Purpose

屏蔽 Linux I/O 与设备协议差异。

DRV 不允许包含业务逻辑。

3.2 Internal Structure
drv/
 ├── io/
 ├── proto/
 └── dev/
io（I/O适配）

负责系统调用：

socketcan

termios(485)

libgpiod

UDP socket

librealsense

仅处理：

open/read/write/poll
proto（协议层）

负责：

帧解析

分包/重组

CRC

ACK/SEQ

message 编解码

输出统一 Message。

禁止访问 CORE。

dev（设备语义层）

提供设备能力接口：

arm_move()
platform_lock()
car_set_velocity()
gripper_open()

dev 不包含任务逻辑。

4. CORE Layer (System Mechanism)

CORE 为系统控制中心。

4.1 Reactor

统一 I/O 入口。

监听：

can1

can2

rs485

udp control port

timerfd

基于 epoll 实现。

产生底层事件：

EVT_IO_xxx
EVT_TIMER
4.2 Router

职责：

Message → Event 转换

事件分发

定向投递(task/device)

Router 不执行逻辑。

4.3 CommandDispatcher ⭐

位置：

core/command/

系统唯一外部控制入口。

职责：

接收 EVT_CMD_REQUEST

权限检查

状态检查

冲突检测

转换为 TaskRequest

回复 ACK/NACK

禁止：

直接控制设备

推进任务FSM

4.4 Task Manager

负责：

Task 生命周期

FSM 推进

超时管理

取消/重试

资源互锁

Task 仅通过 Event 推进。

4.5 Device Registry

维护：

ARM → can1
PLATFORM → can2
GRIPPER → gpio
CAR → rs485

系统启动必须完成设备检查。

4.6 Event Bus

系统唯一状态传播机制。

禁止模块间直接调用修改状态。

5. APP Layer (Behavior Logic)

APP 仅描述行为流程。

示例：

TASK_BATTERY_PICK
TASK_ARM_HOME
TASK_PLATFORM_LOCK
TASK_CAR_MOVE

APP 不允许：

操作 fd

解析协议

访问 GPIO/CAN

5.1 Battery Pick FSM
PRECHECK
→ FIXTURE_LOCK
→ DETECT
→ APPROACH
→ GRIPPER_OPEN
→ ALIGN
→ GRASP
→ LIFT
→ DONE

异常统一进入 FAIL/SAFE_STOP。

6. External Command Flow

外部控制路径必须为：

Network Port
 → drv/io
 → drv/proto
 → Router
 → CommandDispatcher
 → TaskManager
 → APP Task
 → Device

禁止绕过 CommandDispatcher。

7. Safety Rules
Emergency Stop

最高优先级。

触发后：

停止机械臂

停止小车

GPIO进入安全态

所有Task取消

Mutual Exclusion

抓取任务期间：

小车必须 STOP

平台必须锁止

Device Offline

任一关键设备离线：

System → FAULT
8. Thread Model

推荐线程：

Thread	Role
reactor	所有I/O
task	FSM推进
vision	D435处理
optional mesh	网络高负载

规则：

Task线程禁止阻塞I/O

状态更新仅通过 Event

9. Observability

系统必须支持：

统一日志

Task Trace

Command记录

Device回包记录

支持任务回放。

10. Build System

使用 Makefile。

必须支持：

make all
make clean
make run
make install
make test

输出：

build/bin/uav_robotd
build/bin/uav_cli
11. Directory Layout
uav_robot/
 ├── app/
 ├── core/
 │    ├── reactor/
 │    ├── router/
 │    ├── command/
 │    ├── task/
 │    └── bus/
 ├── drv/
 │    ├── io/
 │    ├── proto/
 │    └── dev/
 ├── include/
 ├── tools/
 └── Makefile
12. Architectural Constraints (Mandatory)

外部控制必须经过 CommandDispatcher

APP 不访问 DRV

DRV 不依赖 CORE

所有状态变化通过 Event

Task 为唯一行为执行单位