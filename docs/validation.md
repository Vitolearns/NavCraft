# 运行与验收

环境：Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic。构建使用 Jazzy underlay；生成目录为 `build-navcraft/`、`install-navcraft/`、`log-navcraft/` 和 `artifacts/`。

## 导航验收（2026-11-05）

以下记录来自已完成的运行验收，范围为仿真与软件 mock。原始运行日志不随 v0.1 源码快照保留；测试脚本可重新生成日志、轨迹和 JSON 报告。

| 验证 | 结果 | 执行入口 |
|---|---|---|
| Jazzy 构建 | 13 包通过，退出码 0 | `bash scripts/build_ws.sh` |
| colcon 与任务回归 | 108 项，0 错误/失败/跳过；37 项任务与速度保护检查通过 | `ROS_DOMAIN_ID=201 bash scripts/test.sh` |
| 基础 Gazebo 仿真 | 16 项通过，退出码 0 | 域 202，`python3 scripts/test_simulation.py` |
| 仓库六站与动态绕障 | 26 项通过，退出码 0 | 域 203，`python3 scripts/test_warehouse.py` |
| 文档与媒体完整性 | 四张 SVG 与图源一致，27 处场景引用和 28 个媒体依赖完整 | `python3 scripts/check_docs.py` |

仓库世界轨迹长 **27.680 米**；机器人模型原点到障碍中心最近 **0.689 米**，该值不是车体表面净间隙。

| 站点 | 世界坐标到目标误差（米） | 地图定位到目标误差（米） |
|---|---:|---:|
| receiving | 0.180 | 0.210 |
| storage B | 0.133 | 0.206 |
| storage A | 0.148 | 0.176 |
| packing | 0.242 | 0.198 |
| dock 接近点 | 0.218 | 0.206 |
| home | 0.205 | 0.192 |

六站世界与地图误差均低于 0.35 米。各站轮式 odom 与世界位置相差 0.99–2.79 米；到站成功不能视为里程计精度通过。滑移与里程计误差来源尚未逐项量化。

## 扩展模式验收（2026-11-04）

| 验证 | 已记录结果 |
|---|---|
| 软件与运行矩阵 | 12 组通过：单元/任务回归、底盘 mock、UDP、Gazebo 导航、ros2_control、两种 mock 跟踪、两种 Gazebo 跟踪、两套 SLAM、独立导航 |
| 启动生命周期 | 11 项通过：导航激活、重复启动拦截、退出释放域锁、同域重启、唯一最终速度发布者 |

扩展模式的结果早于上表导航验收，未在 2026-11-05 重新执行完整矩阵。完整入口 `validate.sh` 现包含生命周期检查，共 13 组；仓库专用验收另跑。

## 复现命令

从项目根目录运行。测试选择空闲 ROS 域，与交互演示隔离；每个运行期脚本回收自己启动的进程。

```bash
bash scripts/build_ws.sh
bash scripts/test.sh
source /opt/ros/jazzy/setup.bash
source install-navcraft/local_setup.bash
ROS_DOMAIN_ID=185 python3 scripts/test_simulation.py
ROS_DOMAIN_ID=186 python3 scripts/test_warehouse.py
ROS_DOMAIN_ID=187 python3 scripts/test_mock_chassis.py
ROS_DOMAIN_ID=188 python3 scripts/test_modes.py ros2_control
ROS_DOMAIN_ID=189 bash scripts/udp_sim_acceptance.sh
ROS_DOMAIN_ID=190 python3 scripts/test_lifecycle.py
```

完整矩阵：`bash scripts/validate.sh`。`test_modes.py` 还支持 `navigation`、`pure_pursuit`、`stanley`、`gazebo_pure_pursuit`、`gazebo_stanley`、`slam_toolbox` 和 `cartographer`。可用 `--output artifacts/v0.1/<模式>` 指定模式结果目录。

仓库输出 `artifacts/navcraft-warehouse/result.json` 与 `observations.json`，基础仿真输出 `artifacts/navcraft-simulation/`，任务保护输出 `artifacts/navcraft-regression/`。这些目录由测试生成，不是发布包中的预置报告。

导航急停服务 `/emergency_stop`；SLAM/跟踪使用 `/enable_emergency_stop`。SLAM 遥控输入 `/teleop_cmd_vel`。详细操作见 [维护文档](development.md)。

## 发布检查（2026-11-05）

发布前重新执行 13 包构建、108 项 colcon 测试及 37 项任务与速度保护检查，均通过。文档检查覆盖 7 份 Markdown、61 个本地链接、4 张 SVG、27 处场景引用和 28 个媒体依赖；仅含公开文件的副本也通过检查。

本次发布检查未重跑 Gazebo、SLAM、跟踪及底盘运行矩阵；上文运行指标属于对应日期的验收。源码快照不包含本机构建、安装目录或运行日志，首次运行需先构建。

## CI 诊断复验（2026-11-05）

调整 CI 安装阶段和失败报告后，在已有 Jazzy 工作区执行 `ROS_DOMAIN_ID=187 bash scripts/test.sh`，退出码 0：108 项 colcon 测试和 37 项 Action/速度保护检查通过。同域执行 `python3 scripts/test_mock_chassis.py`，8 项检查通过，退出码 0。

另核验 Bash 语法、workflow YAML 与安装阶段时限；受控失败分别保留 colcon test 的退出码 17、test-result 的退出码 19，且两种失败都会打印结果、不继续运行 Action 回归。这次本地复验未重新构建或运行 Gazebo，不验证 GitHub runner 的网络与全新依赖安装。云端 CI 结论仍需查看对应提交的运行记录。

## 结果适用范围

建图验收检查 OccupancyGrid 生成，未评估全场景建图精度。跟踪验收检查运动、急停与唯一速度发布者，未完成公平算法性能比较。Serial、UDP 与 ros2_control 适配不代表实车验证；Docker 构建尚未验证；托管 CI 状态以 [Actions](https://github.com/Vitolearns/NavCraft/actions/workflows/ci.yml) 中对应提交的运行结果为准，覆盖构建、单元、任务故障注入和 mock 底盘，不覆盖 Gazebo 矩阵。仓库指标仅适用于指定路线、障碍和这次运行。
