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

编译 `proc_gateway`：

```bash
make -C proc_gateway
```

清理编译产物：

```bash
make clean
```

> 说明：不同子模块可能有各自依赖（如摄像头/NPU SDK），请按模块实际环境准备。

## 3.1 配置开机自启动

项目内已提供 systemd 安装脚本，可把 `uav_robotd`、`proc_gateway`、`proc_realsense`、`proc_npu` 配置为开机自启动。

在目标机本地安装：

```bash
cd UAV_Robot
chmod +x tools/install_autostart.sh
sudo ./tools/install_autostart.sh
```

如果只想安装部分服务，例如主控和网关：

```bash
sudo ./tools/install_autostart.sh --services uav_robotd,proc_gateway
```

首次安装会生成环境文件：

```bash
/etc/default/uav_robot
```

可在其中调整 `uav_robotd` 启动参数，以及底盘 CAN 参数，例如 `UAV_CAR_CAN_IFACE=can3`。

如果要从开发机直接推送到远程 Ubuntu 主机：

```bash
cd UAV_Robot
chmod +x tools/deploy_autostart_remote.sh
./tools/deploy_autostart_remote.sh \
  --host 192.168.1.101 \
  --user ubuntu \
  --password ubuntu \
  --remote-path /home/ubuntu/UAV_Robot
```

安装完成后可用以下命令检查状态：

```bash
systemctl status uav-robotd.service
systemctl status uav-proc-gateway.service
journalctl -u uav-robotd.service -f
```

## 4. GitHub 依赖下载受限时的处理流程

如果目标板无法直接访问 GitHub，建议统一按以下流程处理外部依赖：

1. 在目标板上扫描所有需要在线下载的依赖
2. 在主机上根据扫描结果提前下载依赖
3. 将依赖拷贝回目标板，并禁用构建过程中的在线下载

### 4.1 第一步：在目标板扫描依赖

在项目目录执行：

```bash
#!/bin/bash
# scan_deps.sh - 在目标板上运行，扫描所有需要下载的依赖
echo "=== 扫描所有 download.cmake.in 文件 ==="
for f in $(find . -name "*download*.cmake.in" 2>/dev/null); do
    echo ""
    echo "--- $f ---"
    grep -E "GIT_REPOSITORY|GIT_TAG|DOWNLOAD_COMMAND|--branch" "$f" | grep -v "^#"
done
```

用途：

- 找出所有 `download.cmake.in` 中声明的 Git 仓库地址、分支和 tag
- 用于后续在可联网主机上手工下载依赖

### 4.2 第二步：在主机下载依赖并传到目标板

在可访问 GitHub 的主机上执行：

```bash
#!/bin/bash
# fetch_deps.sh - 在主机上运行，根据扫描结果下载所有依赖
RK3588_USER="ubuntu"
RK3588_IP="192.168.10.2"
BUILD_DIR="~/scripts_test/librealsense/build"  # 改成实际路径

# ===== 在这里填写扫描到的依赖 =====
declare -A REPOS=(
    ["pybind11"]="https://github.com/pybind/pybind11.git|v2.13.6|third-party/pybind11"
    ["pybind11-json"]="https://github.com/pybind/pybind11_json.git|0.2.15|third-party/pybind11-json"
    ["json"]="https://github.com/nlohmann/json.git|v3.11.3|third-party/json"
    ["catch2"]="https://github.com/catchorg/Catch2.git|v3.4.0|third-party/catch2"
)
# ==================================

mkdir -p deps && cd deps

for name in "${!REPOS[@]}"; do
    IFS='|' read -r url tag dest <<< "${REPOS[$name]}"
    echo "=== 克隆 $name $tag ==="
    git clone --depth 1 --branch "$tag" "$url" "$name"

    echo "=== 创建远程目录 $dest ==="
    ssh "$RK3588_USER@$RK3588_IP" "mkdir -p $BUILD_DIR/$dest"

    echo "=== 传输 $name ==="
    scp -r "$name"/* "$RK3588_USER@$RK3588_IP:$BUILD_DIR/$dest/"
done

echo "=== 全部完成！==="
```

说明：

- 只需要修改目标板 IP、用户名、构建目录和依赖列表
- 如果主机也无法访问 GitHub，再考虑镜像源、压缩包中转或离线 U 盘传输

### 4.3 第三步：在目标板禁用在线下载

将依赖传到目标板后，再执行项目里的禁用下载脚本，例如：

```bash
bash disable_downloads.sh
```

如果项目没有现成脚本，建议按相同思路把 `download` 行为改为读取本地目录。

### 4.4 推荐执行顺序

建议固定按下面顺序处理：

```bash
# 目标板：先扫描依赖
bash scan_deps.sh

# 主机：根据扫描结果下载并传输依赖
bash fetch_deps.sh

# 目标板：禁用在线下载后重新编译
bash disable_downloads.sh
make -j6
```

这个流程适合作为以后在板卡环境里遇到 GitHub 下载受限问题时的统一处理准则。

## 5. 开发与提交流程

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

## 6. 维护建议

- 新增模块时同步更新本 README 的目录说明。
- 关键协议、接口变更请同步更新 `spec.md` / `spec2.md`。
- 建议按功能拆分提交，便于回溯与评审。
