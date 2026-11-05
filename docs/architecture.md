# 架构与能力边界

本页描述当前源码的启动与数据关系。默认入口是 `scripts/demo.sh`，13 包名称统一为 `navcraft_*`。图中的可选模式不是同时运行；默认、可选和仅用于验收的路径分别注明。

两张主图保留浅蓝圆角节点、灰色折线箭头和从上到下的流程版式，增加输入话题、执行节点、互斥模式与验收边界；两张专项图进一步展开传感器与 TF。实线表示运行时数据或控制流，虚线表示启动选择或可选关系；交叉但没有共同端点的线不表示数据合并。以下链接可以单独打开 SVG 放大查看：

| 图 | 阅读重点 |
|---|---|
| [总体架构](images/architecture-overview.svg) | 启动、三种模式、两套速度保护、执行和反馈 |
| [仓库导航与验收](images/warehouse-navigation.svg) | 障碍感知、AMCL、Nav2 内部分工、任务控制与世界坐标验收 |
| [逐条数据链](images/sensor-dataflow.svg) | 六类数据的来源、桥接、处理和实际消费者 |
| [TF 发布责任](images/tf-ownership.svg) | 坐标树、动态发布者、固定安装与可选底盘替代关系 |

## 1. 系统总览

![系统总览](images/architecture-overview.svg)

入口调用链是 [demo.sh](../scripts/demo.sh) → [launch_demo.py](../scripts/launch_demo.py) → [navcraft.launch.py](../src/navcraft_bringup/launch/navcraft.launch.py)。启动器管理 ROS 域锁、Gazebo 分区和自身子进程回收；launch 根据 mode 选一条控制链。

| 模式 | 定位/数据 | 如何产生速度 | 最终发布者 |
|---|---|---|---|
| navigation | 预置地图 + `/scan` + odom TF；AMCL 定位 | 航点执行器驱动 Nav2，controller/behavior 输出 `/nav2_cmd_vel` | navcraft_safety/cmd_vel_safety_gate_node |
| slam | 激光与 odom/TF；SLAM Toolbox 或 Cartographer | 人工遥控 `/teleop_cmd_vel`；SLAM 只建图 | 默认 navcraft_teleop/cmd_vel_safety_gate_node |
| tracking | odom 帧参考路径 + `/odom` | 独立 Pure Pursuit 或 Stanley → `/tracking_cmd_vel` | 默认 navcraft_teleop/cmd_vel_safety_gate_node |

三模式的默认执行端都是 Gazebo DiffDrive。实验模式默认使用 twist_mux → `/muxed_cmd_vel` → 实验速度门；配置关闭 twist_mux 时由该包的自定义 mux 替代。Serial、UDP、mock、ros2_control 是独立底盘入口，不会自动加入默认仿真。

默认导航不运行独立 Stanley/PP。Nav2 默认控制器虽然也属于 Pure Pursuit 家族，实际用的是外部 **Regulated Pure Pursuit** 插件；仓库里的两种自研跟踪节点用于独立实验，不串联在 Nav2 后面。

## 2. 启动清单与可选模块

| 组件 | 导航 | 建图 | 跟踪 |
|---|---|---|---|
| sim.launch.py：Gazebo、桥接、传感器处理、URDF、odom TF | 启用 | 启用 | 启用 |
| AMCL、map_server、Nav2 导航服务 | 启用 | 关闭 | 关闭 |
| navcraft_mission + navcraft_safety | 启用 | 关闭 | 关闭 |
| map_manager | 启用 | 关闭 | 关闭 |
| 选定 SLAM 后端 | 关闭 | 启用 | 关闭 |
| path_publisher + 选定跟踪器 + experiment_runner | 关闭 | 关闭 | 启用 |
| navcraft_teleop 仲裁/保护 | 关闭 | 启用 | 启用 |
| RViz 参数 use_rviz | 控制是否开启 | 当前统一入口不使用此参数 | 当前统一入口不使用此参数 |
| EKF、区域过滤器、高级 profile、health.launch.py | 默认均未启用 | 默认均未启用 | 默认均未启用 |

配置文件存在、包有依赖、模型有传感器，都不等于当前运行入口已使用它。节点是否启动以 launch 分支和条件为准。

## 3. 传感器与坐标关系

![每条传感器链](images/sensor-dataflow.svg)

默认导航用激光、轮式里程计及 TF；IMU 只做帧名规范化，相机只桥接图像。两者有数据，但未加入默认导航融合。SLAM Toolbox 不使用 IMU，相机未参与两套 SLAM；Cartographer 配置明确 `use_imu_data=false`。

![坐标发布关系](images/tf-ownership.svg)

导航中 AMCL 发布 map→odom；Gazebo odom 适配节点发布 odom→base_footprint；URDF/robot_state_publisher 提供实体和传感器安装关系。每条动态变换只能有一个负责发布的路径。跟踪使用 odom 帧，不需要 AMCL。

频率、传感器 frame、桥接话题、每种消费者、可选 EKF 与显示用 joint_states 的限制，完整列于 [sensors-and-tf.md](sensors-and-tf.md)。TF 本身不是传感器；Gazebo 世界位置也不能与轮式里程计混为一谈。

## 4. 仓库导航闭环

![仓库导航与独立验收](images/warehouse-navigation.svg)

1. `warehouse` 通过 [mission.py](../scripts/mission.py) 读取 [warehouse_route.py](../scripts/warehouse_route.py)，提交 `/navigate_sequence`。
2. [航点执行器](../src/navcraft_mission/src/navigate_sequence_server_node.cpp) 依次调用 `/navigate_to_pose`，同时发布 `/execution_state`。
3. [nav.launch.py](../src/navcraft_navigation/launch/nav.launch.py) 启动定位和导航服务。map_server 发布预置地图，AMCL 结合 scan 与 odom TF 输出 map→odom。
4. [nav2_basic.yaml](../src/navcraft_navigation/config/nav2_basic.yaml) 配置 NavFn（use_astar=false，即 Dijkstra 路径搜索）、Regulated Pure Pursuit 控制、全局/局部代价地图以及恢复行为。两张代价地图均含 static、obstacle、inflation 层；实时 scan 会标记和清除障碍。
5. Nav2 产生 `/nav2_cmd_vel`，导航速度门过滤状态/超时/急停/限幅后，向 `/cmd_vel` 发布唯一最终指令，Gazebo DiffDrive 执行并产生新观测。
6. 仓库测试额外桥接世界位姿，核验六站位置和绕障距离，绝不将真值反馈导航。

`/plan` 是全局路径；`/lookahead_collision_arc` 是 RPP 碰撞检查弧，不是另一套局部优化轨迹。`warehouse` 本身不放障碍；`obstacle block/clear` 或 `test_warehouse.py` 才操作实体障碍。操作及站点坐标见 [warehouse-demo.md](warehouse-demo.md)。

## 5. 航点状态机与导航速度门

状态机由单线程、互斥回调驱动，不用 detached worker 或阻塞 future。有限序列仅接收 1–100 个 map 帧、有限坐标、近似单位四元数目标；loop 请求被拒绝，同时只占用一项任务。

| 状态 | 行为与退出条件 | 速度门 |
|---|---|---|
| IDLE | 等待合法序列；受理时立即预留为 RESERVED | 零速 |
| RESERVED / WAIT_SERVER | 完成任务占用，等待 Nav2 server；默认 10 s 超时失败 | 零速 |
| SENDING | 异步下发并等响应；默认 5 s 超时进入 FAULT | 零速 |
| RUNNING | 执行当前点；成功后下一个，ABORTED 每点最多重试一次；默认目标超时 120 s | 满足其他门控条件时放行 |
| PAUSING | 请求取消下层，等待目标结束；默认取消超时 5 s → FAULT | 零速 |
| PAUSED | 下层目标已结束，保留航点索引；恢复后重新等待/派发 | 零速 |
| CANCELING | 取消或目标超时收尾，确认下层结束后返回终态 | 零速 |
| FAULT | 下层接收/取消结果未知；拒绝新任务，迟到接收仍尝试取消 | 零速 |

Action 取消用 canceled 终态；通过服务取消时若外层 Action 尚未处于 canceling，则返回 abort，结果消息说明 cancelled。不能只看服务 success 就认为下层已停车。FAULT 不被迟到回调自动清除；需确认旧导航停止后整体重启。

[导航速度门](../src/navcraft_safety/src/cmd_vel_safety_gate_node.cpp) 每 20 ms 用 steady_clock 检查：RUNNING、未锁存急停、状态和速度各自小于 300 ms、linear.x/angular.z 为有限值。放行时限幅 ±0.5 m/s 与 ±1.2 rad/s，其他分量保持零；否则发零速。非 RUNNING 或急停切换清除缓存，解除需新速度。它不直接检查 scan/odom 新鲜度，也不等同于对所有传感器的安全监控。

实验门使用不同输入和超时配置，不能把上述导航故障测试结论套用到所有模式。默认导航不自动加载健康监督；手动急停服务可用。

## 6. 13 包职责与源码入口

| 包 | 核心职责 | 主要入口 / 边界 |
|---|---|---|
| navcraft_bringup | 模式编排 | launch/navcraft.launch.py；只选一条链 |
| navcraft_interfaces | 单车公共接口 | NavigateSequence Action、状态/误差消息、地图/底盘/控制服务；不是进程 |
| navcraft_mission | 航点与异步生命周期 | navigate_sequence_server_node；仅调用 Nav2，不做路径规划 |
| navcraft_navigation | AMCL/Nav2/SLAM 配置、地图目录和区域 | nav.launch.py 默认；profiles/zone_filters 为可选 |
| navcraft_tracking | 独立 PP/Stanley 与参考路径发布 | path_publisher_node、两个 controller；不做障碍规划 |
| navcraft_safety | 导航任务感知门控 | execution_state + nav2_cmd_vel + emergency_stop |
| navcraft_teleop | 实验仲裁、人工输入、急停 | cmd_vel_stack.launch.py；与导航门互斥 |
| navcraft_description | 统一几何与 URDF | robot_geometry.yaml、robot.urdf.xacro |
| navcraft_simulation | 仓库、桥接、平面 odom/TF | sim.launch.py、indoor_room.sdf.xacro |
| navcraft_sensors | 激光规范/过滤、IMU 帧适配、可选硬件解析 | 默认处理节点由 sim.launch.py 直接创建 |
| navcraft_hardware | 底盘协议、适配、标定、可选 EKF/ros2_control | 默认导航不调用该底盘路径 |
| navcraft_diagnostics | 聚合诊断、系统监测、故障监督 | 可选 health.launch.py；监督面向实验仲裁服务 |
| navcraft_evaluation | 跟踪误差/运动记录及 CSV/JSON | experiment_runner_node，跟踪模式启用；不是算法排名结论 |

依赖方向以 package.xml 为准：bringup 组织上层运行包；simulation 依赖 description/sensors；mission、tracking、evaluation 等共享 interfaces；hardware 依赖 description/interfaces。导航包的依赖还覆盖可选与测试入口，不能用 package.xml 推断运行拓扑。

## 7. 地图、扩展与设计取舍

预置地图由 `generate_map.py` 读取场景碰撞几何生成；它不是通过 SLAM 实测生成。map_manager 管理目录、元数据和区域查询，set_active_map 不等于在线热切 Nav2 map_server；区域 YAML 不代表默认已启用 costmap filter。

公共接口包集中定义消息和服务，航点 Action 服务负责有限序列的生命周期。仓库环境用于展示规划与避障；订单、设施预约、车队和充电业务不在平台范围。独立跟踪节点用于算法实验，主导航使用 Nav2。

轮式 odom 与世界坐标仍有偏离，世界到站结果和里程计精度分开解释。最新验证日期、误差和未执行项见 [validation.md](validation.md)。

## 8. 图的维护

四张 SVG 的唯一可执行源为 [render_architecture.py](../scripts/render_architecture.py)，仅依赖 Python 标准库。中文字体回退见维护文档。修改配置、话题或职责时，同步修改图源与本页：

```bash
python3 scripts/render_architecture.py
python3 scripts/render_architecture.py --check
python3 scripts/check_docs.py
```

图中按消费者分别连接每类数据：SLAM 的运动来自遥控，AMCL 通过 TF 获取里程计运动；轮式 odom、TF 和世界位姿分别表示运动估计、坐标关系与测量真值。可选模块标注启用条件，验收桥接与导航 TF 分离。
