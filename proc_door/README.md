# proc_door — 舱门 / 停机坪 · 继电器与输入监测进程

对接 **中盛 数字量输入输出模块（RS485 / Modbus RTU）**，即 `doorMonitor/DioPanel.ps1`
所控制的那块板子。把该板子的串口独占在一个进程里，对外用 JSON-RPC 暴露：

```text
/tmp/uav_proc_door.sock      # Unix SOCK_STREAM，一行一个 JSON
```

架构与 `proc_airport` / `proc_gripper` 一致：`src/main.cpp` → `app/` → `core/`，
`proc_gateway` 把 `door.*` 和 `helipad.*` 原样转发过来（HostGUI 只连 7001 即可）。

## 1. 现场接线

```text
Y1 + Y2   停机坪升降电机    Y2=1 上升   Y1=1 下降   全 0 停
Y3        舱门电机使能      1 = 通电运动
Y4        舱门方向          0 = 开舱门   1 = 关舱门
Y5 ~ Y8   未使用

X1        停机坪 上限位 (Top)
X2        停机坪 下限位 (Bottom)
X3        舱门 开到最大位置
X4        舱门 关到最小位置
```

**输出全部关闭 = 默认安全状态。** 进程退出、急停、超时后都会回到这个状态。

两种运动形式，代码里对应 `AxisMode`：

| 轴 | 模式 | 说明 |
| --- | --- | --- |
| 停机坪 `helipad` | `kDualCoil` | 一个方向一路线圈，**绝不同时吸合** |
| 舱门 `hatch` | `kRunDir` | 使能线圈 + 方向线圈，**方向只在断电时切换** |

> 两轴的实际转向都与初版接线说明相反，2026-08-08 现场首测后改正。
> 修正只落在 `UAV_DOOR_HATCH_OPEN_LEVEL` 和 `UAV_DOOR_PAD_UP_CH` /
> `UAV_DOOR_PAD_DOWN_CH` 上，代码没动。

## 2. 运动是异步的

命令只负责上电然后立刻返回，**由后台轮询监督限位和超时并断电**。

这样设计的原因：进程是单线程 poll 循环，如果命令阻塞等限位，运动期间
`door.stop` 根本进不来——急停按钮会失效。异步之后，停止随时可达，
调用方（HostGUI 本来就在轮询传感器）通过 `door.get_status` 看进度。

监督周期 `UAV_DOOR_POLL_MS`（默认 150ms），限位超程上限就是这个周期。

## 3. 安全逻辑

- **换向保护**：反向前先断电，再等 `UAV_DOOR_DIR_SETTLE_MS`（默认 150ms）
  才切方向线圈——带电换向是接触器粘连的典型原因
- **互锁**：停机坪两路线圈绝不同时吸合
- **已在位不动作**：目标限位已触发时直接返回 `reason:"already"`，不上电
- **超时断电**：舱门 20s / 停机坪 30s（可配），到点强制断电并记 `timeout`
- **断链保护**：连续 5 次通讯失败判定链路断开，正在运行的轴标记为放弃
  （因为已经看不到限位、也保证不了能断电），并每 2s 尝试重开串口
- **手动改写**：`door.set_relay` 写到某轴正在用的通道时，该轴放弃监督权
  （操作员优先），记 `manual_override`
- **退出释放**：SIGTERM 时释放 Y1~Y4
- **不调 `tcdrain`**：USB 转换器被拔掉后 `tcdrain` 会无限期阻塞，单线程
  服务里等于全局死锁。响应读本来就有自己的截止时间，不需要它
- **设备消失立即关 fd**：`write`/`read` 返回 EBADF/EIO/ENODEV/ENXIO 时直接
  关闭串口（这些错误码只可能是设备没了），不再傻等 5 次失败
- **断链降频**：模块不应答时，把探测退到 2s 一次而不是 150ms 一次。
  否则每个 tick 都在串口上白等超时，单线程服务循环被饿死，请求在 socket
  上堆积，网关又阻塞在转发上 —— 一块掉电的继电器板能把整个控制台拖死
  （2026-08-08 实际发生过，见排错表）

## 4. JSON-RPC 方法

请求 / 应答都是单行 JSON；参数既可平铺也可放在 `params` 里。

### 舱门

| 方法 | 说明 |
| --- | --- |
| `door.open` | Y4=0 → 等待 → Y3=1，跑到 X3 或超时后断电 |
| `door.close` | Y4=1 → 等待 → Y3=1，跑到 X4 或超时后断电 |
| `door.stop` | 立即断 Y3，并把 Y4 归位到 0 |

### 停机坪

| 方法 | 说明 |
| --- | --- |
| `helipad.up` | Y1=0 → Y2=1，跑到 X1 或超时后断电 |
| `helipad.down` | Y2=0 → Y1=1，跑到 X2 或超时后断电 |
| `helipad.stop` | 立即断 Y1/Y2 |

### 通用

| 方法 | 参数 | 说明 |
| --- | --- | --- |
| `door.stop_all` | – | 舱门 + 停机坪 全部断电（急停） |
| `door.get_status` | – | 全量状态，见下 |
| `door.get_inputs` | `refresh`(默认 true) | 读输入 `0x02` |
| `door.get_outputs` | – | 读继电器 `0x01` |
| `door.set_relay` | `channel`(1..N), `on` | 单路直控 `0x05` |
| `door.set_all` | `on` | 全开 / 全关 `0x0F` |
| `door.pulse` | `channel`, `ms` | 点动（仅用于辅助输出，勿用于电机轴） |
| `door.raw` | `hex`, `expect` | 手动帧，自动补 CRC；调试用 |
| `door.reconnect` | – | 强制重开串口 |
| `system.ping` | – | 存活探测 |

动作命令返回的 `reason`：

| reason | 含义 |
| --- | --- |
| `started` | 已上电开始运动（`ok:true`） |
| `already` | 目标限位已触发，未上电（`ok:true`） |
| `reached` | 监督到限位，已断电（出现在后续 status 里） |
| `timeout` | 超时未到位，已强制断电 |
| `stopped` | 被 stop / stop_all 停下 |
| `manual_override` | 通道被 `door.set_relay` 手动改写，放弃监督 |
| `link down` / `link lost` | 通讯中断，放弃监督 |

### `door.get_status` 应答

```json
{"jsonrpc":"2.0","id":1,"result":{
  "ok":true,"connected":true,
  "port":"/dev/serial/by-path/platform-fc880000.usb-usb-0:1.1.1:1.0-port0",
  "baud":38400,"addr":1,
  "hatch":  {"state":"closed","moving":false,"opened":false,"closed":true,
             "reason":"reached","elapsed_ms":4200,"timeout_ms":20000},
  "helipad":{"state":"top","moving":false,"top":true,"bottom":false,
             "reason":"reached","elapsed_ms":8100,"timeout_ms":30000},
  "in_count":8,"out_count":8,
  "inputs":[1,0,0,1,0,0,0,0],"outputs":[0,0,0,0,0,0,0,0],
  "input_age_ms":25,"last_change_ms":8300,"change_seq":4}}
```

`state` 取值：

- 舱门：`opening` / `closing` / `opened` / `closed` / `between` / `fault` / `unknown`
- 停机坪：`rising` / `lowering` / `top` / `bottom` / `between` / `fault` / `unknown`

`fault` = 同一轴的两个限位同时有效 → 传感器或接线问题。
`change_seq` 每次输入变化 +1，可以用来判断"有没有新事件"而不用比数组。

### 示例

```bash
# 直连 proc_door
printf '{"id":1,"method":"door.get_status"}\n' | nc -U /tmp/uav_proc_door.sock

# 经 proc_gateway（HostGUI 走这条）
printf '{"id":2,"method":"door.open"}\n'      | nc 127.0.0.1 7001
printf '{"id":3,"method":"helipad.up"}\n'     | nc 127.0.0.1 7001
printf '{"id":4,"method":"door.stop_all"}\n'  | nc 127.0.0.1 7001
printf '{"id":5,"method":"door.raw","hex":"01 05 00 00 FF 00"}\n' | nc 127.0.0.1 7001
```

## 5. 协议实现

`core/modbus_rtu.cpp` 是一个最小 Modbus-RTU 主站（8N1，CRC16 低字节在前），
只实现 DioPanel 用到的功能码：

| 功能码 | 用途 | 对应 DioPanel |
| --- | --- | --- |
| `0x01` | 读线圈 = 继电器输出状态 | 输出按钮回读 |
| `0x02` | 读离散输入 = 干接点/限位 | 输入状态灯 |
| `0x03` | 读保持寄存器 `0x32` | 连接时的握手 |
| `0x05` | 写单个线圈 | 单路开/关 |
| `0x0F` | 写多个线圈 | 全开 / 全关 |

通道编号对外一律 **从 1 开始**（`Y1..Yn` / `X1..Xn`，与模块丝印和面板一致），
内部减 1 转成 Modbus 地址。与手册一致：协议地址 `0000H` = 通道 1，无偏移。

## 6. 现场实测记录（RK3588，2026-08-08）

| 项目 | 结果 |
| --- | --- |
| 串口 | USB hub **1 号口** 的 CH340（`1a86:7523`），枚举成 `/dev/ttyUSB0` |
| 稳定路径 | `/dev/serial/by-path/platform-fc880000.usb-usb-0:1.1.1:1.0-port0` |
| 握手 `0x03` | `module online, station id = 1` |
| 读输入 `0x02` | 通过；触发 X4 后日志出现 `input X4 1` / `input X4 0` |
| 读输出 `0x01` | 通过 |
| 写线圈 `0x05` | 通过（在**未接负载**的 Y5 上验证：ON→回读 1→OFF→回读 0） |
| 网关 `:7001` | `door.*` / `helipad.*` 转发正常 |
| 电机实动 | 用户在 HostGUI 上首测：两轴都能动，但转向与初版接线说明相反，已按上表改正配置 |
| 限位停止 | 通过——舱门关闭 11.2s 后被 X4 停住，`reason:"reached"`，输出自动归零 |

`/dev/ttyUSB0` 同时也是旧夹爪（`UAV_GRIPPER_UART_PATH`）的默认节点。
当前 `UAV_ARM_BACKEND=piper`、`proc_gripper` 不运行所以不冲突；
用 by-path 绑定物理口可以彻底避开这个问题。

## 7. 配置

全部环境变量见 `systemd/uav_robot.env.example` 的 `proc_door` 段。常改的：

```bash
# 绑物理口，别写 ttyUSBn（枚举顺序不稳，且与旧夹爪抢 ttyUSB0）
UAV_DOOR_UART_PATH="/dev/serial/by-path/platform-fc880000.usb-usb-0:1.1.1:1.0-port0"
UAV_DOOR_ADDR="1"
UAV_DOOR_POLL_MS="150"            # 监督周期 = 限位超程上限
UAV_DOOR_DIR_SETTLE_MS="150"      # 断电→换向的间隔

UAV_DOOR_HATCH_RUN_CH="3"         # Y3 使能
UAV_DOOR_HATCH_DIR_CH="4"         # Y4 方向
UAV_DOOR_HATCH_OPEN_LEVEL="0"     # Y4=0 为开（现场实测）
UAV_DOOR_HATCH_OPEN_IN="3"        # X3
UAV_DOOR_HATCH_CLOSE_IN="4"       # X4
UAV_DOOR_HATCH_TIMEOUT_MS="20000"

UAV_DOOR_PAD_UP_CH="2"            # Y2 上升（现场实测）
UAV_DOOR_PAD_DOWN_CH="1"          # Y1 下降
UAV_DOOR_PAD_TOP_IN="1"           # X1
UAV_DOOR_PAD_BOTTOM_IN="2"        # X2
UAV_DOOR_PAD_TIMEOUT_MS="30000"
```

限位是常闭型（压到时读 0）就把 `UAV_DOOR_IN_ACTIVE_LOW` 置 1。
换 USB 口后 by-path 会变，用 `ls -l /dev/serial/by-path/` 重新确认。

## 8. 构建与部署

```bash
make -C proc_door                                        # 单独编译
sudo ./tools/install_autostart.sh --services proc_door   # 装 systemd 并启动
sudo journalctl -u uav-proc-door.service -f
```

`proc_door` 已加入顶层 `make`、`install_autostart.sh`、`start_all.sh` /
`stop_all.sh`、`deploy.py` 的默认服务列表，以及 gateway 的
`system.get_logs`（`"source":"proc_door"`）。

## 9. 排错

| 现象 | 排查方向 |
| --- | --- |
| 日志 `no handshake response` | A/B 反接、波特率或站号不对、模块没上电 |
| `connected:false` + `open ... failed` | 串口路径不对或被别的进程占用（`fuser` 查） |
| `crc mismatch` 反复出现 | 485 没接终端电阻 / 干扰 / 多主站抢总线 |
| 输入状态与实际相反 | 置 `UAV_DOOR_IN_ACTIVE_LOW=1` |
| 开门却往关的方向走 | `UAV_DOOR_HATCH_OPEN_LEVEL` 改成 0 |
| 停机坪上下反了 | 对调 `UAV_DOOR_PAD_UP_CH` / `UAV_DOOR_PAD_DOWN_CH` |
| 一直 `timeout` | 限位序号配错，或行程时间超过 `*_TIMEOUT_MS` |
| `state` 长期 `fault` | 同轴两个限位同时有效，查传感器/接线 |
| RPC 全部超时、`ls /proc/<pid>/fd` 见大量 socket 泄漏 | accept 新连接时没初始化 `pfds[]` 那一格的 `revents`，残留值导致新客户端刚接进来就被当成断开丢弃。已修（2026-08-08）。<br>同类写法已一并修正：`proc_airport`、`proc_car`、`proc_arm`、`proc_gripper`，以及 `proc_npu`/`proc_grasp`/`proc_realsense` 的 `ctrl_server`（后者是 `pfds[]` 越界读） |
| 继电器板断电后整个 HostGUI 卡死 | 已修（2026-08-08）。断链降频 + 网关转发超时 30s→2s。<br>若复现：`ss -x \| grep -c uav_proc_door` 看积压，`cat /proc/$(pgrep proc_gateway)/syscall` 看是否阻塞在 read |
| 板子重新上电后不通 | 面板上点「重连」（`door.reconnect`），或等后端 2s 自动重试 |

先用 `doorMonitor/继电器控制.bat`（Windows）确认板子本身工作正常，
再用 `door.raw` 在板卡侧复现同一帧，能很快分清是接线还是软件问题。
