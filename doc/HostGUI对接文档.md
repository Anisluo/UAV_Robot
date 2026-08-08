# HostGUI 对接文档

## 1. 目标

HostGUI 与 `proc_gateway` 通过 JSON-RPC 通信。

当前设计目标：

- HostGUI 只负责任务配置、点击执行、查看日志和状态
- 任务执行过程中不需要 HostGUI 持续介入控制
- `proc_realsense -> proc_npu -> uav_robotd/task` 在机器人主机本地闭环完成
- 视频流是否转发，由 HostGUI 主控界面的开关控制

## 2. 通信端口

- JSON-RPC: `7001`
- MJPEG 视频流: `7002`

JSON-RPC 为按行分隔的文本协议，每条请求和响应都以 `\n` 结尾。

## 3. 推荐交互流程

### 3.1 执行任务

1. HostGUI 在任务配置窗口选择任务
2. HostGUI 调用 `task.start`
3. 机器人主机开始自行执行任务
4. HostGUI 通过 `task.get_status` 轮询任务状态
5. HostGUI 通过 `system.get_logs` 拉取执行日志

说明：

- 执行过程中不需要 HostGUI 再逐步控制机械臂、底盘或夹爪
- HostGUI 只做状态展示，不做中途打断

### 3.2 视频流开关

1. 主控界面提供一个 `RadioButton` / 开关控件
2. 勾选启用时，调用 `video.set_enabled`
3. 未勾选时，调用 `video.set_enabled` 并设置为 `false`
4. 只有启用时，`proc_gateway` 才允许视频连接并转发视频流

## 4. JSON-RPC 接口

## 4.1 系统基础

### `system.ping`

请求：

```json
{"id":1,"method":"system.ping"}
```

响应：

```json
{"id":1,"result":{"pong":true,"uptime_ms":12345}}
```

## 4.2 任务控制

### `task.start`

用途：

- 启动任务

当前推荐任务名：

- `battery_pick`

请求：

```json
{"id":2,"method":"task.start","task":"battery_pick"}
```

响应：

```json
{"id":2,"result":{"ok":true,"task":"battery_pick"}}
```

### `task.get_status`

用途：

- 获取当前任务状态

请求：

```json
{"id":3,"method":"task.get_status"}
```

响应示例：

```json
{"id":3,"result":{"active":true,"task":"BATTERY_PICK","status":"running","reason":""}}
```

可能状态：

- `idle`
- `running`
- `done`
- `failed`
- `stopped`

建议：

- HostGUI 每 `500ms ~ 1000ms` 轮询一次

### `system.get_logs`

用途：

- 拉取最近日志，显示在 HostGUI 日志窗口

请求：

```json
{"id":4,"method":"system.get_logs","max_lines":100}
```

响应：

```json
{"id":4,"result":{"path":"/tmp/uav_robotd.log","logs":"..."}}
```

建议：

- HostGUI 每 `1000ms` 拉取一次
- `max_lines` 推荐 `50 ~ 200`

## 4.3 视频流控制

### `video.set_enabled`

用途：

- 控制 `proc_gateway` 是否允许视频流输出

请求：

```json
{"id":5,"method":"video.set_enabled","enabled":true}
```

或：

```json
{"id":6,"method":"video.set_enabled","enabled":false}
```

响应：

```json
{"id":5,"result":{"ok":true,"enabled":true}}
```

行为说明：

- `enabled=true`：允许 HostGUI 连接 `7002` 并接收 MJPEG
- `enabled=false`：`proc_gateway` 不再转发视频，并关闭已有视频客户端连接

### `video.get_status`

请求：

```json
{"id":7,"method":"video.get_status"}
```

响应：

```json
{"id":7,"result":{"enabled":true,"clients":1}}
```

## 4.4 NPU / 相机控制

### `camera.set_profile`

请求：

```json
{"id":8,"method":"camera.set_profile","width":640,"height":480,"fps":30}
```

### `camera.set_exposure`

请求：

```json
{"id":9,"method":"camera.set_exposure","exposure_us":8000}
```

### `npu.start`

```json
{"id":10,"method":"npu.start"}
```

### `npu.stop`

```json
{"id":11,"method":"npu.stop"}
```

### `npu.set_strategy`

```json
{"id":12,"method":"npu.set_strategy","strategy_id":0}
```

支持的 `strategy_id`：

| id | 名称 | 模型 / 实现 | 说明 |
|----|------|-------------|------|
| 0  | default          | `default.rknn`       | 方形电池检测（RKNN） |
| 1  | battery_v2       | `battery_v2.rknn`    | 电池 V2（RKNN） |
| 2  | custom           | `custom.rknn`        | 自定义占位（RKNN） |
| 3  | face             | `face_tracker.py`    | 人脸追踪（Python 旁路） |
| 4  | battery_cv       | `battery_tracker.py` | 御三电池识别（Python 旁路） |
| 5  | mavic3_drone     | `mavic3_drone.rknn`  | 御三无人机机身识别（RKNN，单类） |

### `npu.set_threshold`

```json
{"id":13,"method":"npu.set_threshold","threshold":0.5}
```

### `npu.get_detections`

用途：

- 获取 `proc_npu` 最近一次检测结果

```json
{"id":14,"method":"npu.get_detections"}
```

## 4.5 机械臂控制

说明：

- 机械臂走 `can4`
- 机械臂为 ZDT 闭环 CAN 电机

### `arm.home`

```json
{"id":20,"method":"arm.home"}
```

### `arm.stop`

```json
{"id":21,"method":"arm.stop"}
```

### `arm.move_pose`

支持的 pose：

- `home`
- `approach`
- `lift`
- `zero`

```json
{"id":22,"method":"arm.move_pose","pose":"approach"}
```

### `arm.move_xyz`

```json
{"id":23,"method":"arm.move_xyz","x_mm":260,"y_mm":0,"z_mm":120}
```

### `arm.move_joint`

```json
{"id":24,"method":"arm.move_joint","joint":3,"target_deg":-25}
```

### `arm.move_joints`

```json
{"id":25,"method":"arm.move_joints","j1_deg":0,"j2_deg":50,"j3_deg":-30,"j4_deg":0,"j5_deg":0,"j6_deg":0}
```

## 4.6 夹爪控制

夹爪当前为串口控制，`115200 8N1`。
默认串口配置为：

```text
UAV_GRIPPER_UART_PATH="/dev/ttyUSB0"
```

当前固化协议字节为：

```text
open  = 55 55 01 07 01 90 01 E8 03 7A
close = 55 55 01 07 01 84 03 E8 03 84
```

`airport.gripper` 只通过机场继电器控制机场夹爪，不与机械臂夹爪共用控制链路。

### `airport.gripper`

```json
{"id":30,"method":"airport.gripper","open":true}
```

```json
{"id":31,"method":"airport.gripper","open":false}
```

## 4.7 底盘控制

### `ugv.set_velocity`

```json
{"id":40,"method":"ugv.set_velocity","vx":0.2,"vy":0.0,"omega":0.0}
```

### `ugv.stop`

```json
{"id":41,"method":"ugv.stop"}
```

## 4.8 舱门 / 停机坪控制（proc_door）

底层是 RS485 / Modbus-RTU 数字量输入输出模块（中盛 DIO 系列），
由 `proc_door` 独占串口，网关把 `door.*` 和 `helipad.*` 原样转发。

现场接线：

```text
Y1+Y2  停机坪升降   Y1=1,Y2=0 上升 / Y1=0,Y2=1 下降 / 全 0 停
Y3     舱门电机使能 (1=通电运动)
Y4     舱门方向     (1=正转=开舱门 / 0=反转=关舱门)
X1 坪-上限位   X2 坪-下限位   X3 舱门-开到位   X4 舱门-关到位
```

**运动是异步的**：命令上电后立即返回，后端按 150ms 周期监督限位与超时并自动断电。
所以 HostGUI 必须轮询 `door.get_status` 来跟踪进度，好处是"停止"随时可达。
完整说明见 `proc_door/README.md`。

### 舱门

```json
{"id":60,"method":"door.open"}
```

```json
{"id":61,"method":"door.close"}
```

```json
{"id":62,"method":"door.stop"}
```

应答：

```json
{"id":60,"result":{"ok":true,"reason":"started",
  "hatch":{"state":"opening","moving":true,"opened":false,"closed":false,
           "reason":"started","elapsed_ms":0,"timeout_ms":20000}}}
```

`reason`：`started`(已上电) / `already`(已在位，未上电) / `reached`(到限位已断电) /
`timeout`(超时已断电) / `stopped`(被停止) / `manual_override` / `link down`。

### 停机坪

```json
{"id":63,"method":"helipad.up"}
```

```json
{"id":64,"method":"helipad.down"}
```

```json
{"id":65,"method":"helipad.stop"}
```

### 急停

```json
{"id":66,"method":"door.stop_all"}
```

舱门 + 停机坪全部断电，回到"输出全关"的默认安全状态。

### `door.get_status`

HostGUI 以 300ms 周期轮询，驱动传感器指示灯和两个轴的状态文字。

```json
{"id":67,"method":"door.get_status"}
```

```json
{"id":67,"result":{
  "ok":true,"connected":true,"baud":38400,"addr":1,
  "hatch":  {"state":"closed","moving":false,"opened":false,"closed":true,
             "reason":"reached","elapsed_ms":4200,"timeout_ms":20000},
  "helipad":{"state":"top","moving":false,"top":true,"bottom":false,
             "reason":"reached","elapsed_ms":8100,"timeout_ms":30000},
  "in_count":8,"out_count":8,
  "inputs":[1,0,0,1,0,0,0,0],"outputs":[0,0,0,0,0,0,0,0],
  "input_age_ms":25,"last_change_ms":8300,"change_seq":4}}
```

- `connected:false` 表示串口没打开或模块无应答，此时 `inputs`/`outputs` 为空数组
- 舱门 `state`：`opening`/`closing`/`opened`/`closed`/`between`/`fault`/`unknown`
- 停机坪 `state`：`rising`/`lowering`/`top`/`bottom`/`between`/`fault`/`unknown`
- `fault` = 同轴两个限位同时有效 → 传感器/接线问题
- `change_seq` 每次输入变化 +1，可用于判断"有没有新事件"而不用比数组

### 单路继电器直控 / 调试

```json
{"id":68,"method":"door.set_relay","channel":5,"on":true}
```

```json
{"id":69,"method":"door.set_all","on":false}
```

```json
{"id":70,"method":"door.raw","hex":"01 05 00 00 FF 00"}
```

注意：写到 Y1~Y4 会让对应轴放弃监督（操作员优先），状态记 `manual_override`。

### 日志

```json
{"id":71,"method":"system.get_logs","source":"proc_door","max_lines":100}
```

## 5. HostGUI 界面建议

### 5.1 任务配置窗口

- 提供任务选择框
- 提供“执行任务”按钮
- 点击后只调用一次 `task.start`
- 执行后将按钮置灰，直到 `task.get_status` 返回非 `running`

### 5.2 主控界面

- 提供视频使能 `RadioButton` 或开关
- 选中时调用 `video.set_enabled(enabled=true)`
- 取消选中时调用 `video.set_enabled(enabled=false)`
- 只有启用时才连接 `7002`

### 5.3 日志窗口

- 周期调用 `system.get_logs`
- 显示最近日志文本
- 同时结合 `task.get_status` 展示任务状态

## 6. 环境变量

关键配置位于：

- `/etc/default/uav_robot`

常用项：

```bash
UAV_LOG_FILE="/tmp/uav_robotd.log"
UAV_TASK_STATUS_FILE="/tmp/uav_task_status.json"
UAV_VIDEO_STREAM_ENABLED="1"
UAV_ARM_CAN_IFACE="can4"
UAV_GRIPPER_UART_PATH="/dev/ttyUSB0"
UAV_DOOR_UART_PATH="/dev/ttyUSB1"
UAV_DOOR_ADDR="1"
UAV_DOOR_MODE="pulse"
UAV_DOOR_OPEN_CH="1"
UAV_DOOR_CLOSE_CH="2"
UAV_DOOR_OPENED_IN="1"
UAV_DOOR_CLOSED_IN="2"
```

## 7. 注意事项

- HostGUI 不应在任务执行过程中频繁下发运动控制命令
- 视频关闭时，不应再尝试维持 `7002` 视频连接
- 若任务执行失败，优先查看 `task.get_status` 的 `reason` 和 `system.get_logs`
- `task.start` 成功只表示任务已成功触发，不代表任务已执行完成
