<h1 align="center">NavCraft</h1>

<p align="center"><strong>自主导航 · 可观测控制 · 可复现仿真</strong></p>
<p align="center">面向仓库场景的 ROS 2 自主导航、运动控制与路径跟踪实验平台。</p>

<p align="center">
  <img alt="ROS 2 Jazzy" src="https://img.shields.io/badge/ROS_2-Jazzy-22314E?style=flat-square">
  <img alt="Gazebo Harmonic" src="https://img.shields.io/badge/Gazebo-Harmonic-ED8B00?style=flat-square">
  <img alt="Ubuntu 24.04" src="https://img.shields.io/badge/Ubuntu-24.04-E95420?style=flat-square">
  <img alt="Release v0.1" src="https://img.shields.io/badge/release-v0.1-2563EB?style=flat-square">
</p>

<p align="center"><a href="README.md">English</a> · <strong>简体中文</strong></p>

[快速开始](#快速开始) · [仓库演示](#仓库演示) · [系统架构](#系统架构) · [文档导航](#文档导航) · [验证](#验证)

---

NavCraft 将定位、规划、控制与任务执行组织到一个单车仓库仿真中。下发六站任务、放置实体障碍、观察路径重规划，再通过暂停和取消检查机器人实际运动，让导航过程可以运行、观察和验证。

项目基于 **ROS 2 Jazzy、Nav2 与 Gazebo Harmonic**，集成已有地图导航、航点任务状态管理、速度保护、互斥启动模式和独立到站验收。另提供 **SLAM Toolbox / Cartographer** 建图，以及 **Pure Pursuit / Stanley** 独立跟踪实验。

## 项目亮点

- **复杂仓库中的导航。** 保留货架、工位与交叉通道，六站路线连接 receiving、storage、packing、dock 接近点与 home。
- **可检查的障碍处理。** 实体障碍经激光进入代价地图并触发重规划；仓库测试使用 Gazebo 世界位姿独立检查实际轨迹与到站位置。
- **完整的任务控制过程。** 有限航点执行器处理异步 Action、暂停/恢复、取消、迟到回调和故障状态；下游速度门检查执行状态与命令新鲜度。
- **互斥的控制链。** 导航、建图和跟踪分模式启动，同一 ROS 域内保持唯一最终速度发布者与底盘路径。
- **可深入理解的跟踪实现。** 独立 Pure Pursuit 与 Stanley 用于观察路径跟随行为，并导出 CSV/JSON 指标。
- **职责明确的工作区。** 13 个 ROS 包分别组织接口、仿真、导航、控制、传感器、底盘适配、诊断和评估。

## 快速开始

### 1. 构建工作区

准备 Ubuntu 24.04 或 WSL，并安装 **ROS 2 Jazzy、Gazebo Harmonic 与 rosdep**。在 Bash 中运行：

```bash
git clone https://github.com/Vitolearns/navcraft.git
cd navcraft
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
bash scripts/build_ws.sh
```

已有源码时进入实际项目目录，无需重复克隆。构建和安装产物会在本机生成。

### 2. 启动仓库

```bash
bash scripts/demo.sh up gui:=true use_rviz:=true
```

保持此终端运行，另开 Bash 并进入项目根目录，下发任务：

```bash
bash scripts/demo.sh warehouse
```

**`up` 启动系统，`warehouse` 才开始六站任务。** 短距离双航点演示使用 `bash scripts/demo.sh run`。同一时刻只提交一项任务。

无显示环境可用 `gui:=false use_rviz:=false`。结束时在启动终端按 Ctrl+C，并等待进程回收。

## 仓库演示

六站顺序为 **receiving → storage B → storage A → packing → dock 接近点 → home**。可同时在 Gazebo 观察场景，在 RViz 查看地图、定位、激光、规划路径与控制器可视化。

在其他终端进入项目根目录后操作：

| 操作 | 命令 |
|---|---|
| 查看执行状态 | `bash scripts/demo.sh status` |
| 暂停 / 恢复 | `bash scripts/demo.sh pause` / `resume` |
| 取消当前任务 | `bash scripts/demo.sh cancel` |
| 启用 / 解除导航急停 | `bash scripts/demo.sh estop` / `release` |
| 放置 / 移除实体障碍 | `bash scripts/demo.sh obstacle block` / `clear` |

`status` 持续输出，Ctrl+C 结束查看。六站命令不会自动放置障碍；手动障碍固定在 `(-1.5, 0)`，建议出发前放置，避免生成在车身里。[仓库演示指南](docs/warehouse-demo.md) 说明路线坐标、自动障碍注入和独立验收判据。

默认 `ROS_DOMAIN_ID=73`，demo 自动配置 `GZ_PARTITION=navcraft-73`、本机发现范围与域锁。修改 ROS 域后，所有控制终端也要设置相同的 `ROS_DOMAIN_ID`。

## 系统架构

![NavCraft 总体架构](docs/images/architecture-overview.svg)

默认导航由航点执行器调用 Nav2，经导航速度门向 Gazebo DiffDrive 输出速度。建图与独立跟踪使用各自的实验仲裁链。**三种模式每次只运行一种。**

默认导航使用激光、轮式里程计与 TF。IMU 和相机提供数据用于观察，但没有接入默认导航融合。Gazebo 世界位姿仅供独立验收，不反馈导航定位。

<details>
<summary><strong>展开数据链路与验收边界</strong></summary>

### 仓库导航与独立验收

![仓库导航](docs/images/warehouse-navigation.svg)

### 逐条传感器数据链

![传感器数据链](docs/images/sensor-dataflow.svg)

### TF 树与发布者

![TF 发布关系](docs/images/tf-ownership.svg)

</details>

13 个包的职责、启动边界与源码位置见 [架构文档](docs/architecture.md)。

## 建图与跟踪

停止当前模式，再选择以下一项入口：

```bash
# 建图：遥控机器人探索场景
bash scripts/demo.sh up mode:=slam slam_mode:=slam_toolbox gui:=true
bash scripts/demo.sh up mode:=slam slam_mode:=cartographer gui:=true

# 跟踪：独立控制器跟随预设路径
bash scripts/demo.sh up mode:=tracking controller:=pure_pursuit gui:=true
bash scripts/demo.sh up mode:=tracking controller:=stanley gui:=true
```

建图不会自动驱动车辆。独立跟踪不包含全局规划或激光避障。Cartographer 当前关闭 IMU 输入。

SLAM 与跟踪使用 `/enable_emergency_stop`；`demo.sh estop/release` 面向导航模式。统一入口的 `use_rviz` 当前仅在导航模式生效。遥控、保存地图、路径配置与指标输出见 [维护指南](docs/development.md)。

## 文档导航

| 阅读入口 | 内容 |
|---|---|
| [架构](docs/architecture.md) | 包职责、控制链与源码索引 |
| [传感器与 TF](docs/sensors-and-tf.md) | 来源、频率、话题、消费者与坐标变换发布责任 |
| [仓库演示](docs/warehouse-demo.md) | 路线、实体障碍与世界位姿到站检查 |
| [开发与维护](docs/development.md) | 配置、可选入口、排错与清理 |
| [验收记录](docs/validation.md) | 复现命令、带日期的结果与验证边界 |
| [项目讲解](docs/project-guide.md) | 工程取舍与系统讲解路线 |

详细指南与图中文字目前为中文。[English](README.md) 提供相同的项目入口。

源码位于 `src/`，构建、演示与验收工具位于 `scripts/`。`docker/` 和 `.github/` 提供容器与 CI 定义。`build-navcraft/`、`install-navcraft/`、`log-navcraft/` 和 `artifacts/` 为本机生成目录，不进入 Git。

## 验证

验收范围为 **仿真与软件 mock**。`v0.1` 已记录的基线包括：

| 检查 | 已记录结果 |
|---|---|
| 工作区构建 | 13 包构建通过 |
| colcon 测试 | 108 项，零失败 |
| 任务与速度保护 | 37 项检查通过 |
| 基础 Gazebo 导航与传感器 | 16 项检查通过 |
| 仓库路线与障碍处理 | 26 项检查通过 |

这些检查来自不同验证轮次，命令、日期、测量值与覆盖范围见 [validation.md](docs/validation.md)。构建后可复现：

```bash
python3 scripts/check_docs.py
bash scripts/test.sh
source /opt/ros/jazzy/setup.bash
source install-navcraft/local_setup.bash
ROS_DOMAIN_ID=187 python3 scripts/test_simulation.py
ROS_DOMAIN_ID=188 python3 scripts/test_warehouse.py
```

选择空闲 ROS 域，与正在运行的演示隔离。`bash scripts/validate.sh` 执行完整 13 组基线，仓库专用测试单独运行。

### 当前边界

- 轮式里程计与世界位置仍有明显偏离，不能作为到站真值。
- 尚未完成 EKF 融合精度、全场景建图精度或公平的控制器性能排名。
- Serial、UDP、mock、ros2_control 和 EKF 属于可选底盘入口；软件验收不能代替实车性能验证。
- 门、电梯和充电桩属于场景资产；订单、车队调度、设施预约和自动充电工作流不在当前范围。
- 已提供容器与托管 CI 定义，尚未验证其实际运行。

## 参与贡献

欢迎问题反馈与聚焦的改进。报告问题时请附环境、运行模式、复现命令、预期行为和相关日志。修改前阅读 [维护指南](docs/development.md)，按受影响模块执行必要检查，并同步相关文档。

## 致谢

NavCraft 基于 [ROS 2](https://www.ros.org/)、[Nav2](https://docs.nav2.org/)、[Gazebo](https://gazebosim.org/)、[SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)、[Cartographer](https://github.com/ros2/cartographer)、[ros2_control](https://control.ros.org/) 与 [twist_mux](https://github.com/ros-teleop/twist_mux)。导航算法主要复用 Nav2，本仓库重点在系统集成、任务执行、速度保护、独立跟踪和可复现评估。

## 许可证

[MIT](LICENSE)。
