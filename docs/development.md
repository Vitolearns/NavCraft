# 开发与运行指南

首次构建与三模式启动见 [README](../README.zh-CN.md)。本页补充仓库操作、参数配置和排错；验收结果统一记录在 [validation.md](validation.md)。所有命令从项目根目录用 Bash 执行。

## 构建与环境补充

`build_ws.sh` 清除旧 overlay 的 CMake/ament 前缀，加载 Jazzy，生成地图，并构建到 `build-navcraft/` 和 `install-navcraft/`，日志进入 `log-navcraft/`。构建前不要加载其他工作区的 overlay。生成地图有差异时先核查几何与生成器。

直接使用 `ros2` 命令的终端需加载 `/opt/ros/jazzy/setup.bash` 与 `install-navcraft/local_setup.bash`，并设置与演示相同的 ROS 域。`demo.sh` 会自行处理这些环境设置。

## 仓库导航与障碍演示

按 [README](../README.zh-CN.md#1-已有地图导航) 启动导航并提交 `warehouse` 任务；以下命令均在另一个项目根目录终端运行。启动场景本身不会下发任务，`run` 是短距离双航点检查。初始位置为 `(0, 0, 0)`，与 AMCL 初值一致；预置地图由场景碰撞几何生成。

### 六站路线

| 顺序 | 站点 | 地图坐标（米） |
|---|---|---|
| 1 | receiving 接近点 | (-3.2, 0.4) |
| 2 | storage B 货架外侧 | (-2.0, 3.0) |
| 3 | storage A 货架外侧 | (1.5, 2.6) |
| 4 | packing 工位外侧 | (3.2, -2.0) |
| 5 | dock 接近点 | (-3.2, -3.2) |
| 6 | home 初始点 | (0.0, 0.0) |

收货点位于低货箱北侧，为车体转向留出空间。路线定义见 [warehouse_route.py](../scripts/warehouse_route.py)。目标位于设施外侧；货架或工作台中心不是可用目标，地面装饰线也不是规划器必须跟随的轨道。

### 实体障碍与任务控制

```bash
bash scripts/demo.sh obstacle block
bash scripts/demo.sh warehouse
```

障碍为 0.42 × 0.42 × 0.70 米，固定在 `(-1.5, 0)`，位于起点到 receiving 的路线上。手动演示先在机器人仍处于起点时放置，再提交任务，避免生成在车身里。`warehouse` 不会自动放置障碍。

| 操作 | 命令 |
|---|---|
| 观察状态 | `bash scripts/demo.sh status`（Ctrl+C 结束查看） |
| 暂停 / 恢复 | `bash scripts/demo.sh pause` / `bash scripts/demo.sh resume` |
| 取消 | `bash scripts/demo.sh cancel` |
| 导航急停 / 解除 | `bash scripts/demo.sh estop` / `bash scripts/demo.sh release` |
| 清除障碍 | `bash scripts/demo.sh obstacle clear` |

清除障碍会把实体移到地下，不改写静态地图；激光与代价地图随后更新。取消任务不会清除障碍。结束演示时在启动终端 Ctrl+C，等待进程回收。修改 ROS 域时，各控制终端也要设置相同的 `ROS_DOMAIN_ID`。

### 观察与验收

RViz 显示 `/map`、机器人、`/scan`、绿色全局路径 `/plan`、局部代价地图及橙色 `/lookahead_collision_arc`。橙色弧线是 Nav2 Regulated Pure Pursuit 的碰撞检查轨迹，不是另一套局部优化路径。观察障碍入图、全局路径变化和 Gazebo 中的实际绕行，再检查暂停、恢复和取消后的实际运动。

门、电梯、充电和交通状态装饰只是场景背景。自动验收负责在指定时机注入障碍，并通过独立世界位姿检查六站到达、绕障距离与停车；命令和判据统一见 [验收记录](validation.md#复现命令)。

## 模式与急停

| 模式 | 速度来源 | 最终门控包 | 急停 SetBool 服务 |
|---|---|---|---|
| navigation | `/nav2_cmd_vel` | navcraft_safety | `/emergency_stop` |
| slam | `/teleop_cmd_vel` | navcraft_teleop | `/enable_emergency_stop` |
| tracking | `/tracking_cmd_vel`，遥控可高优先级接管 | navcraft_teleop | `/enable_emergency_stop` |

实验默认 twist_mux 的 teleop/tracking/nav2 优先级为 100/60/40，输入超时均为 0.5 s，下游实验门再检查超时、急停和安全状态。配置有 nav2 输入不代表默认导航走该链。use_twist_mux=false 是另一实现，不同时启动。

demo.sh estop/release 只用于导航。SLAM/跟踪控制终端手动加载环境并设置同域（示例默认 73）：

```bash
source /opt/ros/jazzy/setup.bash
source install-navcraft/local_setup.bash
export ROS_DOMAIN_ID=73
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 service call /enable_emergency_stop std_srvs/srv/SetBool '{data: true}'
ros2 service call /enable_emergency_stop std_srvs/srv/SetBool '{data: false}'
```

SLAM 可运行 `ros2 run navcraft_teleop keyboard_teleop_node --ros-args -p topic:=/teleop_cmd_vel`，显式覆盖默认的 `/virtual_rc/cmd_vel`，对接当前仲裁输入。W/S 调整前进速度，A/D 调整转向，X/空格停车，Q 停车退出；松开按键不会自动清零。无键盘交互时可发短时低速：

```bash
timeout 2s ros2 topic pub --rate 10 /teleop_cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.10}, angular: {z: 0.0}}'
ros2 topic pub --once /teleop_cmd_vel geometry_msgs/msg/Twist '{}'
```

timeout 到时退出码 124 属于预期，随后发零速并观察停车；遥控不等于自主避障。保存地图：`bash scripts/save_map.sh my_map artifacts/maps`，不会自动切换当前 Nav2 地图。

统一入口的 use_rviz 目前仅导航分支有效。其他模式可在同域单独开 RViz；跟踪 fixed frame 设 odom，不要另开整套仿真。

## 参数与入口

| 目的 | 文件 / 入口 | 注意 |
|---|---|---|
| 模式 | navcraft_bringup/launch/navcraft.launch.py | 一次一条链 |
| 几何与安装 | navcraft_description/config/robot_geometry.yaml | 同步 URDF、SDF、footprint |
| 传感器频率、相机、物理步长 | navcraft_simulation/config/simulation_physics.yaml | 配置不等于实测精度 |
| 默认导航 | navcraft_navigation/config/nav2_basic.yaml | AMCL、NavFn、RPP 和激光/深度双源障碍层 |
| 仿真运动估计 | navcraft_simulation/config/navigation_ekf.yaml | 轮式里程计与 IMU，EKF 独占导航 odom TF |
| footprint | navcraft_navigation/config/robot_footprint.yaml | 模型改变后核查 |
| 六站与障碍 | scripts/warehouse_route.py、warehouse_obstacle.py | map 坐标，目标在设施外，障碍不与车身重叠 |
| 跟踪路径 | navcraft_evaluation/paths/ | 当前路径配置为 odom 帧 |
| 地图目录 | navcraft_navigation/launch/map_manager.launch.py | 默认导航已启动，不在线切换 map_server |
| 区域/高级 profile | navcraft_navigation/launch/profiles.launch.py、zone_filters.launch.py | 可选，默认关闭 |
| 诊断 | navcraft_diagnostics/launch/health.launch.py | 默认关闭；急停面向实验仲裁 |
| 底盘 | navcraft_hardware/launch/hardware.launch.py | mock/Serial/UDP，与 Gazebo 互斥 |
| 硬件 EKF | navcraft_hardware/launch/hardware_ekf.launch.py | 独立底盘入口 |
| ros2_control | navcraft_hardware/launch/ros2_control_hardware.launch.py | 与 Gazebo DiffDrive 互斥 |

跟踪输出路径可指定：

```bash
bash scripts/demo.sh up mode:=tracking controller:=stanley \
  output_csv:=/tmp/navcraft-stanley.csv output_json:=/tmp/navcraft-stanley.json
```

更换 path_file 使用独立 [tracking.launch.py](../src/navcraft_bringup/launch/tracking.launch.py)，统一入口当前不转发此参数。独立入口需自行保证底盘、时钟和域一致，以及唯一速度发布者；不要同时运行完整 demo。参数先用 `ros2 launch <包> <文件> --show-args` 核实。

## 故障排查

| 现象 | 检查和处理 |
|---|---|
| up 后不动 | 导航还需 warehouse/run，SLAM 需遥控；检查 execution_state |
| 任务被拒 | 已有任务、FAULT、非 map 目标、空目标或循环请求 |
| 服务/话题不存在 | 核对域、overlay 和模式；普通控制终端不继承 demo 环境 |
| 急停解除后不动 | 需要新速度和合法任务状态，不复用旧命令；服务名称按模式区分 |
| FAULT | 下层接收/取消结果未知，先确保旧导航停止，再整体重启，不能强改 RUNNING |
| domain already has a managed demo | 回到已有启动终端正常关闭并等回收；不删锁绕过活跃进程 |
| 多个 cmd_vel 发布者 | `ros2 topic info /cmd_vel --verbose` 查找并关闭自己重复启动的链 |
| 定位跳变/TF 冲突 | 检查 odom TF 所属，AMCL/SLAM 是否同开，真值是否误进 tf |
| 模型有传感器但没数据 | 查 bridge、clock、frame 和新消息，模型存在不是链路完成证据 |
| 导航里程计异常 | 检查 `/wheel/odom`、`/imu/data` 的时间戳、`imu_link` 的 TF，以及 `/navigation_ekf` 是否独占 `/odom` |
| 相机障碍没有入图 | 检查 `/sim/camera/depth_image`、`/sim/camera/camera_info` 和 `/camera/scan`。障碍需在前向视场中央扫描带与标记量程内 |
| 地图与世界不符 | 重新生成地图，核对碰撞几何、原点、AMCL 初值和路线，再仓库验收 |
| WSLg/显示不可用 | gui:=false use_rviz:=false；用无界面验收，不把仅 RViz 画面误认 Gazebo 不存在 |
| CLI 显示旧节点 | 先查域与存活进程，必要时重启该域 CLI daemon；发现缓存不代表进程活跃 |

## 保留和清理

| 内容 | 原则 |
|---|---|
| 源码、配置、地图、被引用 OBJ/MTL | 保留并入 Git |
| SVG 与生成器 | 小型可复现文档资产，入 Git |
| build-navcraft、install-navcraft | 本机构建产物，不提交；删除后需重新运行 build_ws.sh |
| log-navcraft | 本机构建与测试日志，可按需清理 |
| artifacts/ | 测试生成的日志、轨迹与报告，不提交；摘要记入 validation.md |
| 缓存、旧预览、无引用资源 | 确认后删除 |
| Git 历史、未提交用户文件、原版目录 | 不在常规清理范围 |

场景使用 27 个 OBJ 与共享 MTL。删除媒体前检查 SDF、OBJ 材质与 MTL 依赖，不能仅凭名称判断资源用途。

## 文档和图维护

编辑 scripts/render_architecture.py 后运行：

```bash
python3 scripts/render_architecture.py
python3 scripts/check_docs.py
```

标准库生成 SVG，不依赖在线服务。检查器验证 Markdown 相对文件链接、图源一致性及场景媒体依赖，不能替代语义核对与目视布局检查。中文字体按 Noto Sans CJK SC、Microsoft YaHei、Droid Sans Fallback 回退；离线预览需本机安装其中一种中文字体。SVG 保留可选取文字，浏览器可直接打开，缩放不会失真。

总体图只保留三模式的入口、控制和反馈关系；导航闭环、数据链和 TF 细节留在架构文档。新增节点先确认实际 launch 条件、话题与消费者，再安排连线；箭头不能穿过无关节点，IMU 应指向里程计估计器，深度扫描应指向代价地图。使用 `python3 scripts/render_architecture.py --check` 可只检查生成结果而不改文件。

提交前检查 `git diff --check` 与完整 diff，运行相关验证并更新 [验收记录](validation.md)。结果需注明执行日期、命令和覆盖范围。

## GitHub Actions CI

`.github/workflows/ci.yml` 仅响应 `workflow_dispatch`。需要构建时，进入 **Actions → ros2-ci → Run workflow** 选择分支，或运行 `gh workflow run ci.yml --ref main`。同一 ref 上再次手动运行会取消上一轮。任务位于 GitHub 的临时 Ubuntu 24.04 runner，不会连接本机机器人。

流水线依次检查文档与资源、配置 ROS 软件源和开发工具、安装 Jazzy ROS base 与声明的依赖、构建、运行 colcon 单元测试、Action 故障注入及 mock 底盘回归；当前不包含 Gazebo 仿真矩阵。ROS base 和声明依赖分步安装并保存日志，不额外安装整个 desktop 元包。总任务上限为 45 分钟，工具配置、ROS base、声明依赖分别限制为 10、10、15 分钟。单元测试通过不等于后续故障注入也通过。

失败时先看报错步骤和断言上下文。任务回归会写出 `artifacts/navcraft-regression/result.json`，检查断言失败时另写 `failure.json`，包含实际 Action 状态、结果消息、期望/实际重试次数与最近执行状态。CI 无论成功失败都会尝试上传 `regression-<attempt>` 日志产物，保存 7 天。云端结果以对应提交的 Actions 运行记录为准，本地通过不能替代云端结果。
