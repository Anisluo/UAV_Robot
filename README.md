# UAV_Robot 项目说明

## 1. 项目简介

`UAV_Robot` 是一个面向无人机/机器人协同控制的工程仓库，包含：

- 主控应用层任务（`app/`）
- 核心调度与通信框架（`core/`）
- 设备与 I/O 驱动（`drv/`）
- 公共头文件与 ABI 定义（`include/`）
- 独立处理进程（`proc_npu/`、`proc_realsense/`）

## 2. 目录结构

```text
UAV_Robot/
|- app/            # 业务任务与主程序入口
|- core/           # bus/channel/router/reactor/scheduler 等核心模块
|- drv/            # 设备驱动与底层 I/O
|- include/        # 对外接口头文件与 ABI
|- proc_npu/       # NPU 相关进程
|- proc_realsense/ # Realsense 相关进程
|- tools/          # 辅助工具
|- Makefile        # 顶层构建入口
|- spec.md         # 需求/设计说明
|- spec2.md        # 补充说明
```

## 3. 构建与运行

首次拉取并进入项目目录：

```bash
git clone git@github.com:Anisluo/UAV_Robot.git
cd UAV_Robot
```

编译整个项目：

```bash
make
```

仅编译主控程序 `uav_robotd`：

```bash
make build/bin/uav_robotd
```

编译命令行工具 `uav_cli`：

```bash
make build/bin/uav_cli
```

运行主控程序：

```bash
./build/bin/uav_robotd
```

运行主控程序并监听指定 UDP 端口：

```bash
./build/bin/uav_robotd --listen-port 19090
```

运行自测：

```bash
./build/bin/uav_robotd --self-test
```

清理编译产物：

```bash
make clean
```

> 说明：不同子模块可能有各自依赖（如摄像头/NPU SDK），请按模块实际环境准备。

## 4. 开发与提交流程

常用 Git 流程：

```bash
git add .
git commit -m "feat: 描述本次改动"
git push
```

当前远程仓库：

- HTTPS: `https://github.com/Anisluo/UAV_Robot.git`
- SSH: `git@github.com:Anisluo/UAV_Robot.git`

仓库页面：

- [https://github.com/Anisluo/UAV_Robot](https://github.com/Anisluo/UAV_Robot)

## 5. 维护建议

- 新增模块时同步更新本 README 的目录说明。
- 关键协议、接口变更请同步更新 `spec.md` / `spec2.md`。
- 建议按功能拆分提交，便于回溯与评审。
