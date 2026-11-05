<h1 align="center">NavCraft</h1>

<p align="center"><strong>Autonomous navigation. Observable control. Reproducible simulation.</strong></p>
<p align="center">A ROS 2 workspace for warehouse navigation, motion control, and path-tracking experiments.</p>

<p align="center">
  <img alt="ROS 2 Jazzy" src="https://img.shields.io/badge/ROS_2-Jazzy-22314E?style=flat-square">
  <img alt="Gazebo Harmonic" src="https://img.shields.io/badge/Gazebo-Harmonic-ED8B00?style=flat-square">
  <img alt="Ubuntu 24.04" src="https://img.shields.io/badge/Ubuntu-24.04-E95420?style=flat-square">
  <img alt="Release v0.1" src="https://img.shields.io/badge/release-v0.1-2563EB?style=flat-square">
</p>

<p align="center"><strong>English</strong> · <a href="README.zh-CN.md">简体中文</a></p>

[Quick start](#quick-start) · [Warehouse demo](#warehouse-demo) · [Architecture](#architecture) · [Documentation](#documentation) · [Validation](#validation)

---

NavCraft brings localization, planning, control, and mission execution into a single-robot warehouse simulation. Send a six-stop mission, introduce a physical obstacle, inspect the replanned path, and pause or cancel execution while observing the robot's actual motion.

Built on **ROS 2 Jazzy, Nav2, and Gazebo Harmonic**, it combines a known-map navigation stack with mission state handling, velocity protection, isolated launch modes, and independent arrival checks. Separate modes support **SLAM Toolbox / Cartographer** mapping and **Pure Pursuit / Stanley** tracking experiments.

## Highlights

- **A warehouse worth navigating.** Shelves, workstations, intersecting aisles, and a six-stop route spanning receiving, storage, packing, dock approach, and home.
- **Obstacle handling you can inspect.** A physical obstacle is observed by the laser, enters the costmap, and triggers replanning; the warehouse test checks the robot's path and arrival against Gazebo world pose.
- **Mission control beyond sending goals.** A finite waypoint executor handles asynchronous Actions, pause/resume, cancellation, late callbacks, and failure states. A downstream velocity gate checks execution state and command freshness.
- **One active control chain.** Navigation, mapping, and tracking run as separate modes, with one final velocity publisher and one chassis path per ROS domain.
- **Tracking algorithms with visible internals.** Standalone Pure Pursuit and Stanley implementations expose path-following behavior and export CSV/JSON metrics.
- **A workspace you can extend.** Thirteen ROS packages separate interfaces, simulation, navigation, control, sensors, chassis adapters, diagnostics, and evaluation.

## Quick start

### 1. Build the workspace

Start with Ubuntu 24.04 or WSL with **ROS 2 Jazzy, Gazebo Harmonic, and rosdep** installed. Run in Bash:

```bash
git clone https://github.com/Vitolearns/navcraft.git
cd navcraft
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
bash scripts/build_ws.sh
```

Already have the sources? Use your checkout directory. Build and installation outputs are generated locally.

### 2. Launch the warehouse

```bash
bash scripts/demo.sh up gui:=true use_rviz:=true
```

Keep this terminal running. In a second Bash terminal, enter the project root and submit the mission:

```bash
bash scripts/demo.sh warehouse
```

**`up` starts the system; `warehouse` starts the six-stop mission.** For a shorter two-waypoint run, use `bash scripts/demo.sh run`. Submit one mission at a time.

Without a display, launch with `gui:=false use_rviz:=false`. Stop the system with Ctrl+C in the launch terminal and wait for process cleanup.

## Warehouse demo

The six-stop mission moves through **receiving → storage B → storage A → packing → dock approach → home**. RViz exposes the map, localization, scan, planned path, and controller visualization alongside the Gazebo scene.

From another terminal in the project root:

| Operation | Command |
|---|---|
| Observe execution state | `bash scripts/demo.sh status` |
| Pause / resume | `bash scripts/demo.sh pause` / `resume` |
| Cancel the current mission | `bash scripts/demo.sh cancel` |
| Engage / release navigation emergency stop | `bash scripts/demo.sh estop` / `release` |
| Insert / remove a physical obstacle | `bash scripts/demo.sh obstacle block` / `clear` |

`status` streams until Ctrl+C. The mission command does not insert obstacles automatically. The manual obstacle has a fixed position at `(-1.5, 0)`; insert it before departure to avoid spawning it inside the robot. The [warehouse guide](docs/warehouse-demo.md) covers route coordinates, automated injection, and the independent validation criteria.

The default ROS domain is `73`. The demo configures `GZ_PARTITION=navcraft-73`, local discovery, and a domain lock. If you change `ROS_DOMAIN_ID`, use the same value in every control terminal.

## Architecture

![NavCraft system architecture](docs/images/architecture-overview.svg)

The default navigation chain runs from the waypoint executor through Nav2 and the navigation velocity gate to Gazebo DiffDrive. Mapping and standalone tracking use their own arbitration chain. **Only one mode runs at a time.**

Default navigation consumes laser scans, wheel odometry, and TF. IMU and camera data are available for inspection, but are not fused into default navigation. Gazebo world pose is reserved for independent validation and never feeds navigation localization.

<details>
<summary><strong>Explore the data paths and validation boundary</strong></summary>

### Warehouse navigation and independent validation

![Warehouse navigation](docs/images/warehouse-navigation.svg)

### Sensor data paths

![Sensor data paths](docs/images/sensor-dataflow.svg)

### TF ownership

![TF ownership](docs/images/tf-ownership.svg)

</details>

See the [architecture guide](docs/architecture.md) for all 13 package responsibilities, launch boundaries, and source references.

## Mapping and tracking

Stop the active mode before starting another. Choose one entry:

```bash
# Mapping: teleoperate the robot to explore the scene
bash scripts/demo.sh up mode:=slam slam_mode:=slam_toolbox gui:=true
bash scripts/demo.sh up mode:=slam slam_mode:=cartographer gui:=true

# Tracking: follow a predefined path with a standalone controller
bash scripts/demo.sh up mode:=tracking controller:=pure_pursuit gui:=true
bash scripts/demo.sh up mode:=tracking controller:=stanley gui:=true
```

Mapping does not drive the robot automatically. Standalone tracking does not include global planning or laser-based obstacle avoidance. Cartographer currently runs without IMU input.

SLAM and tracking use `/enable_emergency_stop`; `demo.sh estop/release` target navigation mode. The unified `use_rviz` option currently applies only to navigation. Teleoperation, map saving, path configuration, and metrics are documented in the [development guide](docs/development.md).

## Documentation

| Start here | What you will find |
|---|---|
| [Architecture](docs/architecture.md) | Package responsibilities, control chains, and source map |
| [Sensors and TF](docs/sensors-and-tf.md) | Sources, rates, topics, consumers, and transform ownership |
| [Warehouse demo](docs/warehouse-demo.md) | Route, physical obstacle, and world-pose arrival checks |
| [Development](docs/development.md) | Configuration, optional launches, troubleshooting, and cleanup |
| [Validation](docs/validation.md) | Reproduction commands, dated results, and coverage limits |
| [Project walkthrough](docs/project-guide.md) | Engineering choices and a guided explanation of the system |

Detailed guides and diagram labels are currently in Chinese. [简体中文首页](README.zh-CN.md) provides the same entry points.

Source lives in `src/`; build, demo, and validation tools live in `scripts/`. `docker/` and `.github/` contain container and CI definitions. Generated `build-navcraft/`, `install-navcraft/`, `log-navcraft/`, and `artifacts/` directories stay outside Git.

## Validation

Validation covers **simulation and software mocks**. The recorded `v0.1` baseline includes:

| Check | Recorded result |
|---|---|
| Workspace build | 13 packages built |
| colcon tests | 108 tests, zero failures |
| Mission and velocity protection | 37 checks passed |
| Basic Gazebo navigation and sensors | 16 checks passed |
| Warehouse route and obstacle handling | 26 checks passed |

These checks were run in separate validation rounds; commands, dates, measurements, and scope are recorded in [validation.md](docs/validation.md). To reproduce after building:

```bash
python3 scripts/check_docs.py
bash scripts/test.sh
source /opt/ros/jazzy/setup.bash
source install-navcraft/local_setup.bash
ROS_DOMAIN_ID=187 python3 scripts/test_simulation.py
ROS_DOMAIN_ID=188 python3 scripts/test_warehouse.py
```

Choose unused ROS domains, isolated from any running demo. `bash scripts/validate.sh` runs the full 13-group baseline; the warehouse test runs separately.

### Current boundaries

- Wheel odometry diverges noticeably from world position; it is not arrival ground truth.
- EKF fusion accuracy, whole-scene mapping accuracy, and a fair controller performance ranking have not been established.
- Serial, UDP, mock, ros2_control, and EKF are optional chassis entries; software validation does not establish real-hardware performance.
- Doors, lifts, and charging stations are scene assets. Order processing, fleet scheduling, facility reservations, and automatic charging workflows are outside the current scope.
- Container and hosted CI definitions are provided; their execution has not yet been validated.

## Contributing

Bug reports and focused improvements are welcome. Include your environment, active mode, reproduction command, expected behavior, and relevant logs. For changes, start with the [development guide](docs/development.md), run the checks appropriate to the affected module, and update the related documentation.

## Acknowledgments

NavCraft builds on [ROS 2](https://www.ros.org/), [Nav2](https://docs.nav2.org/), [Gazebo](https://gazebosim.org/), [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox), [Cartographer](https://github.com/ros2/cartographer), [ros2_control](https://control.ros.org/), and [twist_mux](https://github.com/ros-teleop/twist_mux). Navigation algorithms primarily come from Nav2; this repository focuses on integration, mission execution, velocity protection, standalone tracking, and reproducible evaluation.

## License

[MIT](LICENSE).
