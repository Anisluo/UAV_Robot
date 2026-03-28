# uav_robotd 主机通信测试

## 1. 测试目的

验证主机可以通过 UDP 向 `uav_robotd` 指定端口发送控制指令。

## 2. 前置条件

- 目标设备 IP 为 `192.168.10.2`
- `uav_robotd` 已启动，并监听 UDP 端口 `19090`
- 主机已安装 `nc`（netcat）

示例启动命令：

```bash
./build/bin/uav_robotd --listen-port 19090
```

## 3. 测试指令

发送 `START` 指令：

```bash
echo -e "START\n" | nc -u -w1 192.168.10.2 19090
```

## 4. 预期结果

- 目标端 `uav_robotd` 成功收到 `START` 指令
- 日志中可看到命令路由与任务启动信息
- 系统开始执行对应动作流程

## 5. 可扩展测试

可以按相同方式发送其他控制指令：

```bash
echo -e "ESTOP\n" | nc -u -w1 192.168.10.2 19090
echo -e "RESET\n" | nc -u -w1 192.168.10.2 19090
echo -e "SHUTDOWN\n" | nc -u -w1 192.168.10.2 19090
```
