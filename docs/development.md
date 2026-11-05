# 开发、运行与维护

## 环境与构建

目标 Ubuntu 24.04、ROS 2 Jazzy、Gazebo Harmonic。所有命令从项目根目录用 Bash 执行。首次机器先安装对应 ROS/Gazebo 与 rosdep。

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
bash scripts/build_ws.sh
source install-navcraft/local_setup.bash
colcon list --base-paths src
```

脚本清除旧 overlay 的 CMake/ament 前缀，加载 Jazzy，重新生成地图，构建到 build-navcraft/install-navcraft，日志进 log-navcraft。构建前不要加载其他工作区的 overlay。受控地图有差异时先核查几何与生成器，不直接忽略。

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
| 默认导航 | navcraft_navigation/config/nav2_basic.yaml | AMCL、NavFn、RPP、代价地图 |
| footprint | navcraft_navigation/config/robot_footprint.yaml | 模型改变后核查 |
| 六站与障碍 | scripts/warehouse_route.py、warehouse_obstacle.py | map 坐标，目标在设施外，障碍不与车身重叠 |
| 跟踪路径 | navcraft_evaluation/paths/ | 当前路径配置为 odom 帧 |
| 地图目录 | navcraft_navigation/launch/map_manager.launch.py | 默认导航已启动，不在线切换 map_server |
| 区域/高级 profile | navcraft_navigation/launch/profiles.launch.py、zone_filters.launch.py | 可选，默认关闭 |
| 诊断 | navcraft_diagnostics/launch/health.launch.py | 默认关闭；急停面向实验仲裁 |
| 底盘 | navcraft_hardware/launch/hardware.launch.py | mock/Serial/UDP，与 Gazebo 互斥 |
| EKF | navcraft_hardware/launch/hardware_ekf.launch.py | 独占 odom TF |
| ros2_control | navcraft_hardware/launch/ros2_control_hardware.launch.py | 与 Gazebo DiffDrive 互斥 |

跟踪输出路径可指定：

```bash
bash scripts/demo.sh up mode:=tracking controller:=stanley \
  output_csv:=/tmp/navcraft-stanley.csv output_json:=/tmp/navcraft-stanley.json
```

更换 path_file 使用独立 tracking.launch.py，统一入口当前不转发此参数。独立入口需自行保证底盘、时钟和域一致，以及唯一速度发布者；不要同时运行完整 demo。参数先用 `ros2 launch <包> <文件> --show-args` 核实。

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
| IMU/相机不影响路线 | 当前默认如此，见传感器消费者表 |
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

两张主图沿用浅蓝圆角流程图版式。新增节点先确认实际 launch 条件、话题与消费者，再安排连线；箭头不能穿过无关节点。另两张图分别维护每条数据链和 TF 发布责任，不能把 IMU/相机的存在画成默认已融合。使用 `python3 scripts/render_architecture.py --check` 可只检查生成结果而不改文件。

提交前检查 `git diff --check` 与完整 diff，运行相关验证并更新 [验收记录](validation.md)。结果需注明执行日期、命令和覆盖范围。

## GitHub Actions CI

`.github/workflows/ci.yml` 在 main 推送、目标为 main 的 Pull Request 或手动触发时运行。标签推送不重复启动同一组检查；同一 ref 的新运行会取消旧运行。任务位于 GitHub 的临时 Ubuntu 24.04 runner，不会连接本机机器人。

流水线依次检查文档与资源、安装 Jazzy 依赖、构建、运行 colcon 单元测试、Action 故障注入及 mock 底盘回归；当前不包含 Gazebo 仿真矩阵。单元测试通过不等于后续故障注入也通过。

失败时先看报错步骤和断言上下文。任务回归会写出 `artifacts/navcraft-regression/result.json`，检查断言失败时另写 `failure.json`，包含实际 Action 状态、结果消息、期望/实际重试次数与最近执行状态。CI 无论成功失败都会尝试上传 `regression-<attempt>` 日志产物，保存 7 天。云端结果以对应提交的 Actions 运行记录为准，本地通过不能替代云端结果。
