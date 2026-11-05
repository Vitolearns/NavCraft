# 仓库导航与动态避障演示

默认 Gazebo 世界包含货架、工位、托盘、区域标签、通道与站点装饰，使用 NavCraft 单车控制链。机器人初始位置为地图原点 `(0, 0, 0)`，与 AMCL 初值一致。静态地图由场景碰撞几何重新生成。

## 启动与路线

在项目根目录运行：

```bash
bash scripts/build_ws.sh
bash scripts/demo.sh up gui:=true use_rviz:=true
```

启动完成后，另开 Bash 终端运行：

```bash
cd ~/robot
bash scripts/demo.sh warehouse
```

启动场景本身不会自动发导航任务。`warehouse` 提交六个航点；`run` 仍保留两个航点的快速检查。

| 顺序 | 站点 | 地图坐标（米） |
|---|---|---|
| 1 | receiving 接近点 | (-3.2, 0.0) |
| 2 | storage B 货架外侧 | (-2.0, 3.0) |
| 3 | storage A 货架外侧 | (1.5, 2.6) |
| 4 | packing 工位外侧 | (3.2, -2.0) |
| 5 | dock 接近点 | (-3.2, -3.2) |
| 6 | 初始点 | (0.0, 0.0) |

目标点放在设施外侧；不要把货架或工作台中心直接作为目标。地面装饰线用于说明区域关系，不是规划器强制跟踪的轨道。

## 放置与清除障碍

```bash
bash scripts/demo.sh obstacle block
bash scripts/demo.sh obstacle clear
```

红色实体障碍为 0.42 × 0.42 × 0.70 米，放置位置固定为 `(-1.5, 0)`，位于初始点到 receiving 的路径上。它同时有视觉和碰撞几何；清除将其移到地下，不写入静态地图。命令使用与 demo 相同的 Gazebo 分区，服务失败会返回错误。

手动演示可在机器人仍位于初始点时先放置障碍，再运行 `warehouse`。行驶中放置时必须确保机器人尚未接近该位置，避免把障碍直接生成在车身里。精确的行驶中注入由下方自动验收完成。障碍清除后，激光与代价地图更新，机器人继续完成路线。

暂停、恢复与取消：`bash scripts/demo.sh pause`、`resume`、`cancel`。急停：`bash scripts/demo.sh estop`，解除：`release`。`cancel` 只取消导航，不移除障碍；需要时另执行 `obstacle clear`。结束后在启动终端 Ctrl+C。

## 观察内容

RViz 显示 `/map`、机器人、`/scan`、绿色全局路径 `/plan`、局部代价地图及橙色 `/lookahead_collision_arc`。默认局部控制器是 Nav2 Regulated Pure Pursuit；橙色弧线表示控制器的碰撞检查轨迹，不是另一套局部轨迹优化算法。

对比障碍出现前后的绿色路径、代价地图新增占据区域及 Gazebo 中的实际绕行。门、电梯、充电和交通状态装饰保留作场景背景，不包含相应业务控制系统；这些装饰不能作为门控、电梯或充电功能的验收依据。

## 自动验收

关闭同域演示后：

```bash
source /opt/ros/jazzy/setup.bash
source install-navcraft/local_setup.bash
ROS_DOMAIN_ID=187 python3 scripts/test_warehouse.py
python3 scripts/test_simulation.py
```

仓库验收自动启动无界面 Gazebo，确认原始路径穿过注入点，等机器人开始移动且距注入点超过 0.9 米时放置障碍，检查局部代价地图、绕行路径、真实运动距离、六站任务和最终停车。结束时清除障碍并回收自己启动的仿真。JSON 和轨迹保存在 `artifacts/navcraft-warehouse/`；失败时不保留旧的成功报告。

验收通过模型上的 `PosePublisher` 取得世界位姿，并桥接到独立的 `/warehouse_ground_truth` 话题；它不参与导航定位，也不发布到 `/tf`。每站同时记录世界位姿、地图定位和轮式里程计，要求世界坐标和地图坐标到目标的距离均小于 0.35 米，并检查世界位姿数据新鲜度。不能把 `odom` 坐标直接当成 `map` 或 Gazebo 世界坐标判断到站。

`minimum_obstacle_centre_distance_m` 是机器人模型原点与障碍中心的平面距离，不是车体表面的净间隙。位姿采样频率为 20 Hz；结果只覆盖这条路线和这次障碍工况。

已记录的测试结果见 [运行与验收](validation.md)。本脚本验证指定路线和障碍工况，不代表任意场景的无碰撞保证。
