# UAV_Robot 系统架构规格说明书（Spec）

## 1. 文档目的

本文档用于定义 `UAV_Robot` 系统的软件总体架构、进程职责划分、进程内分层规范、IPC 通信机制、网关转发机制、消息优先级策略及工程实现约束，作为后续开发、联调、扩展和维护的统一设计依据。

---

## 2. 设计目标

本系统面向多设备协同控制场景，需支持以下能力：

1. 对多个异构模块进行统一调度与协同控制，包括但不限于：
   - 六轴机械臂
   - 六轮底盘/小车
   - 三轴平台/云台
   - RealSense 相机
   - NPU 推理模块
   - 远程 HostGUI 接入

2. 保证系统具备以下工程特性：
   - 模块职责清晰
   - 故障隔离
   - 易于扩展
   - 易于调试
   - 支持高低优先级消息调度
   - 支持高带宽数据链路与低延迟控制链路分离
   - 支持统一对外协议出口

---

## 3. 总体架构原则

系统采用以下总体原则：

### 3.1 系统级采用多进程架构
各核心功能模块以独立进程形式运行，形成明确的职责边界和故障隔离边界。

### 3.2 进程内部采用三层分层
每个 `proc_*` 内部采用：

- `app`：业务入口/命令处理/状态机
- `core`：模块语义逻辑/能力抽象
- `drv`：底层硬件、SDK、OS 接口适配

### 3.3 控制流与数据流分离
- 控制决策流必须经过 `UAV_robot`
- 高带宽原始数据流允许点对点高性能通道
- 对外通信统一收口至 `proc_gateway`

### 3.4 对外统一出口
远端 `HostGUI` 仅连接 `proc_gateway`，不直接连接内部 `proc_*`。

### 3.5 事件驱动 + 优先级调度
内部状态上报、事件上报采用事件驱动机制，`proc_gateway` 按来源和消息类型进行优先级调度发送。

---

## 4. 系统进程划分

系统建议至少包含以下进程：

### 4.1 `UAV_robot`
系统主控进程，负责：

- 全局状态机
- 任务流程推进
- 模块协同控制
- 安全联锁
- 超时管理
- 重试与回退策略
- 模块状态汇总
- 对内部模块的命令仲裁

### 4.2 `proc_arm`
机械臂控制进程，负责：

- 机械臂连接与初始化
- 回零
- 关节运动
- 笛卡尔位姿运动
- 抓取/夹爪动作
- 状态反馈
- 故障与告警上报

### 4.3 `proc_car`
底盘/小车控制进程，负责：

- 速度控制
- 运动停止
- 模式切换
- 里程计/姿态反馈
- 故障上报

### 4.4 `proc_gimbal`
三轴平台/云台控制进程，负责：

- 姿态调整
- 角度反馈
- 锁定/解锁
- 故障与状态上报

### 4.5 `proc_realsense`
相机采集进程，负责：

- 相机设备管理
- RGB/Depth 数据采集
- 帧时间戳管理
- 相机在线状态管理
- 图像元数据上报

### 4.6 `proc_npu`
推理进程，负责：

- 模型加载
- 图像推理
- 后处理
- 输出检测框、类别、置信度、位姿、目标状态等摘要结果

### 4.7 `proc_gateway`
系统网关进程，负责：

- 对外协议封装
- 与 HostGUI 的会话管理
- 本地各 `proc_*` 上报消息接收
- 按优先级调度转发
- 对外状态汇聚与事件推送
- 外部命令入口接收与分发

### 4.8 可选扩展进程
后续可按需增加：

- `proc_logger`
- `proc_storage`
- `proc_battery`
- `proc_mesh`
- `proc_io`

---

## 5. 进程协同关系

系统中进程关系应遵循以下规则：

### 5.1 `UAV_robot` 是唯一系统级协同控制中心
所有涉及“是否执行动作、何时切换状态、是否允许下一步”的决策，统一由 `UAV_robot` 负责。

### 5.2 `proc_*` 是能力执行单元
各模块只负责本模块范围内的执行与状态反馈，不负责全局流程推进。

### 5.3 `proc_gateway` 不参与业务决策
`proc_gateway` 只做：

- 收消息
- 管连接
- 编解码
- 排队
- 转发

不得在 `proc_gateway` 中实现任务状态机、动作仲裁、安全策略主逻辑。

### 5.4 高带宽数据可点对点
例如：

- `proc_realsense -> proc_npu`

允许采用共享内存等高性能链路，不要求经由 `UAV_robot` 中转原始图像数据。

### 5.5 控制流必须收口
例如：

- `proc_npu` 不得直接控制 `proc_arm`
- `proc_realsense` 不得直接控制 `proc_car`

所有执行类动作必须经过 `UAV_robot`。

---

## 6. 每个进程内部三层规范

每个 `proc_*` 必须采用 `app / core / drv` 三层组织，不允许职责混乱。

### 6.1 app 层定义

`app` 层职责：

- 接收外部命令
- 处理进程级状态机
- 组织 `core` 完成业务动作
- 对外上报响应、状态、事件
- 管理本进程生命周期

`app` 层禁止：

- 直接操作硬件寄存器
- 直接处理复杂底层报文细节
- 承担设备驱动职责

### 6.2 core 层定义

`core` 层职责：

- 实现模块语义逻辑
- 建立本模块统一能力抽象
- 组织动作执行逻辑
- 实现状态转换逻辑
- 统一错误语义

例如机械臂 `core` 应负责：

- 回零
- move_joint
- move_pose
- stop
- grasp
- motion done 判断

`core` 层禁止：

- 直接承担 socket 会话管理
- 直接做 UI 协议解析
- 混入 OS 层 fd/poll 细节

### 6.3 drv 层定义

`drv` 层职责：

- 硬件接口访问
- SDK 适配
- fd 打开关闭
- CAN/UART/USB/V4L2/librealsense/RKNN 等底层接入
- OS 资源封装

`drv` 层禁止：

- 编写业务流程状态机
- 决定系统动作策略
- 承担跨模块协同逻辑

---

## 7. 推荐目录结构

```text
UAV_Robot/
├── common/
│   ├── include/
│   ├── log/
│   ├── ipc/
│   ├── msg/
│   ├── time/
│   ├── config/
│   └── utils/
│
├── uav_robotd/
│   ├── app/
│   ├── core/
│   ├── drv/
│   └── main.c
│
├── proc_arm/
│   ├── app/
│   ├── core/
│   ├── include/
│   └── main.c
│
├── proc_car/
│   ├── app/
│   ├── core/
│   ├── include/
│   └── main.c
│
├── proc_gimbal/
│   ├── app/
│   ├── core/
│   ├── include/
│   └── main.c
│
├── proc_realsense/
│   ├── app/
│   ├── core/
│   ├── include/
│   └── main.cpp
│
├── proc_npu/
│   ├── app/
│   ├── core/
│   ├── include/
│   └── main.cpp
│
├── proc_gateway/
│   ├── app/
│   ├── core/
│   ├── include/
│   └── main.c
│
└── tools/
```

---

## 8. IPC 通信规范

### 8.1 IPC 分类原则

系统内部 IPC 分为两类：

#### A. 小消息 IPC
用于：

- 命令
- ACK
- 状态上报
- 故障事件
- 心跳
- 推理摘要结果

推荐使用：

**Unix Domain Socket**

#### B. 大数据 IPC
用于：

- RGB 图像
- 深度图
- 大块二进制缓冲
- 点云/张量等

推荐使用：

**共享内存 + Ring Buffer + 事件通知**

### 8.2 Unix Domain Socket 使用原则

建议采用：

- `AF_UNIX + SOCK_SEQPACKET` 优先
- 若平台兼容性有限，可退化为 `SOCK_STREAM + 自定义消息头分帧`

适用链路包括：

- `proc_* <-> UAV_robot`
- `proc_* -> proc_gateway`
- `proc_gateway <-> UAV_robot`

### 8.3 共享内存使用原则

适用链路：

- `proc_realsense -> proc_npu`

共享内存中存储：

- frame_id
- timestamp
- width
- height
- pixel_format
- data_size
- buffer_offset / slot_id

通知方式可选：

- eventfd
- Unix socket 小消息通知
- semaphore

---

## 9. 消息模型规范

所有小消息应采用统一消息头。

### 9.1 消息头字段

建议字段如下：

```c
typedef struct {
    uint16_t magic;
    uint16_t version;
    uint16_t src;
    uint16_t dst;
    uint16_t msg_type;
    uint16_t priority;
    uint32_t seq;
    uint64_t timestamp_ns;
    uint32_t payload_len;
} uav_msg_hdr_t;
```

### 9.2 消息类型分类

建议至少定义以下类型：

- `CMD`：命令
- `ACK`：命令响应
- `STATUS`：状态上报
- `EVENT`：离散事件
- `ERROR`：错误/故障
- `HEARTBEAT`：心跳
- `TELEMETRY`：遥测
- `DEBUG`：调试信息

---

## 10. `proc_gateway` 设计规范

### 10.1 职责定义

`proc_gateway` 是唯一对外网络出口，负责：

- HostGUI 接入
- 外部协议解析
- 内部消息转发
- 队列调度
- 优先级发送
- 状态快照组织
- 告警推送

不得承担：

- 任务状态机
- 跨模块动作决策
- 主业务逻辑推进

### 10.2 队列模型

不允许全系统共用单一全局 FIFO。  
应采用：

**每个 `proc_*` 独立上报队列**。

例如：

- `Q_arm`
- `Q_car`
- `Q_gimbal`
- `Q_npu`
- `Q_cam_meta`
- `Q_io`
- `Q_log`

更推荐升级为：

- `Q_arm_event`
- `Q_arm_status`
- `Q_car_event`
- `Q_car_status`
- `Q_npu_result`
- `Q_log`

### 10.3 事件驱动原则

各 `proc_*` 在以下场景应立即触发上报：

- 到位
- 故障
- 急停
- 目标出现/丢失
- 模块上线/掉线
- ACK/拒绝执行
- 超时

### 10.4 状态类数据策略

对于连续更新的状态类数据，不要求严格保留所有历史值，允许使用：

**latest-value 覆盖策略**

适用示例：

- 当前车速
- 当前姿态角
- 机械臂当前位姿
- 当前识别摘要

不适用示例：

- 急停事件
- 故障事件
- 到位事件
- 模式切换事件

---

## 11. 消息优先级规范

消息优先级按“消息类型”划分，而非仅按“进程来源”划分。

### P0：最高优先级
安全与控制关键消息：

- 急停
- 碰撞
- 故障
- stop ack
- 到位
- 关键拒绝执行
- 模块离线

### P1：高优先级
运动执行反馈：

- 机械臂状态
- 小车状态
- 云台状态
- busy/idle
- command accepted

### P2：中优先级
普通状态：

- 温度
- 电量
- 当前模式
- 遥测摘要
- NPU 结果摘要

### P3：低优先级
调试与日志：

- debug trace
- 性能统计
- verbose 输出

---

## 12. HostGUI 接入规范

### 12.1 外部连接规则
远端 `HostGUI` 只允许连接 `proc_gateway`。

不得直接连接：

- `proc_arm`
- `proc_car`
- `proc_npu`
- `proc_realsense`
- 其他内部进程

### 12.2 HostGUI 获取的数据
HostGUI 可从 `proc_gateway` 获取：

- 系统状态
- 模块状态
- 当前任务阶段
- 告警事件
- 控制命令结果
- 遥测摘要
- 可选的视频/媒体通道

### 12.3 HostGUI 下发的命令
HostGUI 只将外部命令发给 `proc_gateway`，由 `proc_gateway` 转交 `UAV_robot` 或内部目标进程。

不得绕过系统协同层直接下发到底层设备进程。

---

## 13. 推荐通道划分

对外通信建议逻辑分为三类通道：

### 13.1 fast_control/status_fast
用于：

- 急停
- 关键事件
- 控制 ACK
- 到位反馈
- 高优先级遥测

### 13.2 status_snapshot
用于：

- 系统汇总状态
- 周期状态
- 模块健康状态
- 任务阶段信息

### 13.3 media/debug
用于：

- 图像流
- 深度图
- 日志
- 调试数据

说明：  
逻辑上统一由 `proc_gateway` 提供，物理上允许使用不同 socket/端口/通道实现。

---

## 14. 状态机与协同控制规范

### 14.1 全局状态机归属
系统全局状态机只能位于 `UAV_robot`。

### 14.2 本地状态机归属
各 `proc_*` 可以有本地状态机，但仅限本模块范围内。

例如 `proc_arm` 本地状态：

- INIT
- IDLE
- BUSY
- ERROR

不得用本地状态机替代全局任务流程状态机。

---

## 15. 心跳与故障恢复

### 15.1 心跳机制
每个 `proc_*` 必须定期向 `UAV_robot` 上报心跳。  
必要时也可向 `proc_gateway` 镜像上报在线状态。

### 15.2 故障处理
当模块发生故障时：

1. 模块必须立即上报 `ERROR/EVENT`
2. `UAV_robot` 必须更新全局状态
3. 必要时中止当前任务
4. 必要时触发回退、停机、重试或降级

### 15.3 崩溃恢复
后续可增加 supervisor/watchdog 机制，实现子进程异常退出后的自动重启。

---

## 16. 开发约束

### 16.1 禁止事项
- 禁止 `proc_npu` 直接控制 `proc_arm`
- 禁止 `proc_realsense` 直接控制执行模块
- 禁止 `proc_gateway` 承担业务主状态机
- 禁止所有进程共享单个无区分大 FIFO
- 禁止将图像大流量和控制关键流走同一拥塞通道
- 禁止 app/core/drv 职责串层

### 16.2 必须事项
- 必须统一消息头
- 必须统一错误码和日志接口
- 必须为关键消息提供 seq 和 timestamp
- 必须定义超时机制
- 必须定义 ACK 语义
- 必须区分事件类与状态类消息

---

## 17. 架构结论

本系统最终采用如下架构原则：

1. **系统级采用多进程架构**
2. **每个进程内部采用 app/core/drv 三层**
3. **UAV_robot 为唯一系统协同与状态机中心**
4. **proc_gateway 为唯一对外网关与协议转发中心**
5. **HostGUI 仅连接 proc_gateway**
6. **控制决策流统一收口，原始高带宽数据允许点对点优化**
7. **内部上报采用事件驱动、多队列、优先级调度**
8. **高优先级控制反馈优先于普通状态和调试日志**

---

## 18. 一句话总纲

**本架构采用“多进程隔离职责、三层分层保证内部可维护性、UAV_robot 统一协同控制、proc_gateway 统一对外出口、事件驱动与优先级队列保证实时性”的总体设计路线。**
