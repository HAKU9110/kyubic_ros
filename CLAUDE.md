# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

ROS 2 (Jazzy, Ubuntu 24.04) workspace for **KYUBIC**, an autonomous underwater vehicle built by Kyutech Underwater for RoboSub-style competitions. The same workspace also drives a **BlueROV** platform — most subsystems (drivers, planners) exist in vehicle-specific pairs and are selected at launch time rather than by branch.

All development happens inside a Docker container; there is no supported native/host build.

## Environment: everything runs in Docker

- `. install.sh` (add `-c` on client machines, `--nvidia` for CUDA) builds the image, creates the container, and registers a `ros2_start[_<project>]` shell alias plus `KYUBIC_ROS*`/`KYUBIC_ROS_COMPOSE*` env vars in `~/.bashrc`.
- `ros2_start` (from `ros2_start.sh`) starts the container and drops into a `bash` shell as the `ros` user (`gosu ros`). Pass a command after `--` to run something else instead (e.g. `ros2_start -- byobu`).
- Multiple isolated environments can coexist via `-p <name>` at install time (see README "Tips").
- Inside the container, the Python env is a `uv`-managed venv at `~/kyubic_ros/.venv`, auto-activated by `~/.bashrc`. Root `pyproject.toml`/`uv.lock` define Python deps (`uv sync` to update).
- `. uninstall.sh` tears the environment down.

Everything below assumes you are already inside the container, in `kyubic_ws/`.

## Common commands (inside the container)

```bash
# Build (alias defined in ~/.bash_aliases by the container setup)
build                      # = colcon build --symlink-install --cmake-args -GNinja
colcon build --packages-select <pkg> --symlink-install --cmake-args -GNinja  # single package

# Run the full stack (what the byobu session in docker/script/kyubic_byobu.sh does)
ros2 launch kyubic_bringup kyubic.launch.py        # drivers + localization + planners/controller
ros2 launch kyubic_bringup kyubic_post.launch.py   # DVL + actuator + localization components (started after the above is up)
ros2 launch behavior_tree behavior_tree.launch.py  # mission executor

# Vehicle-specific driver sets (see kyubic_ws/src/driver/driver_launcher/README.md)
ros2 launch driver_launcher kyubic_driver.launch.py     # Logic Distro RP2040, Sensors ESP32, IMU
ros2 launch driver_launcher blue_rov_driver.launch.py   # MAVLink + DVL-75 (normal-ops config)

# Lint/format (pre-commit is installed automatically in kyubic_ws by the container setup)
pre-commit run --all-files
ament_clang_format --reformat <file>...   # C++, config: kyubic_ws/.clang-format
uv run ruff format                        # Python, config: pyproject.toml (line-length 100)

# Tests
colcon test --packages-select <pkg>
colcon test-result --verbose
```

`byobu` (aliased) opens a pre-built tmux/byobu session (`docker/script/kyubic_byobu.sh` on the robot, `client_byobu.sh` on client machines) with bringup, monitoring, and nvim windows already laid out — the de facto way this project is operated day-to-day.

Doxygen docs (`kyubic_ws/Doxyfile`) are built and published to GitHub Pages by `.github/workflows/doxygen-gh-pages.yml`.

## Git submodules

`kyubic_ws/src/extra/{BehaviorTree.CPP,protolink}` are submodules. Run `git submodule update --init --recursive` after cloning/pulling if code there looks missing.

## Architecture

### Package layout (`kyubic_ws/src/`)

- `driver/kyubic/*` and `driver/blue_rov/*` — vehicle-specific hardware drivers, paired 1:1 by role (e.g. `dvl_driver` ↔ `dvl75_driver`, `actuator_rp2040_driver` ↔ `mavlink_driver`). `driver/driver_launcher` picks the right set per vehicle; `driver/driver_msgs` holds shared driver message types.
- `localization/` — sensor fusion node(s) (`localization`) + its message package (`localization_msgs`, includes an `Odometry`/`Pose`/`GlobalPos` family and a `Reset` service).
- `control/manual/` — joystick input path (`joy_common` → `joy2wrench`).
- `control/automatic/planning/` — goal/path generation: `path_planner` (CSV-based paths under `assets/`), `projection_dynamic_look_ahead_planner` (PDLA), `qr_planner`, `wrench_planner` (incl. a zero-order-hold variant), tied together by `planner_launcher`. Shared types in `planner_msgs`.
- `control/automatic/emergency/` — safety/abort logic, plugin-based (see below).
- `control/controller/` — low-level force/position controller (`p_pid_controller`) driven by `Targets`/`BaseAxes` messages (`p_pid_controller_msgs`).
- `behavior_tree/` — top-level mission executor built on BehaviorTree.CPP (`bt_executor.cpp`). Mission logic lives as XML trees in `bt_xml/` (`base.xml`, per-competition trees like `kobe2025.xml`, `qr.xml`), executing custom BT action/condition nodes in `src/bt_nodes/` (waypoint following, QR handling, pinger search, battery/sensor checks, lifecycle/mode management).
- `kyubic_bringup/` — top-level launch composition only (no nodes of its own): `kyubic.launch.py`/`kyubic_post.launch.py` (robot-side), `client.launch.py`/`client_visualizer.launch.py`/`web_visualizer.launch.py` (operator-side), `manual.launch.py` (joystick teleop).
- `system_health_check/` — cross-cutting diagnostics framework (see below).
- `common/` — small shared libraries (`pid_controller`, `geodetic_converter`, `custom_socket`, `serial`, `timer`) and `common_msgs`.
- `visualizer/` — operator-facing tools (`dashboard`, `web_controller`, `rt_pose_plotter`, `trajectory_viewer`; `archive/` holds superseded versions).
- `tools/` — `path_generator`, `plotjuggler` layout fixes, `trdi_toolz`.
- `oak_create_mapping/` — OAK camera node (DepthAI) that captures stills/H.265 video on trigger, geotagged via `localization_msgs/GlobalPose`, for mapping/photogrammetry use.
- `sample/` — minimal example packages (action server, BT switch) used as references, not part of the runtime stack.
- `extra/` — git submodules (BehaviorTree.CPP, protolink), not first-party code.

### Composable-node pattern

Most C++ nodes are `rclcpp_components` plugins, not standalone executables, and are grouped into a single `ComposableNodeContainer` per subsystem for intra-process communication (`use_intra_process_comms: true`). E.g. `planner_launcher.launch.py` loads the PDLA planner, QR planner, and (zero-order-hold) wrench planner as components into one container. When adding a new planner/controller node, follow this pattern (register as a component, add to the relevant `*_launcher` launch file) rather than writing a standalone `rclcpp::spin` executable.

### system_health_check plugin framework

`system_health_check` defines a `system_health_check::base::SystemCheckBase` pluginlib interface. Individual packages (e.g. `wrench_planner`) implement checks against it in a `src-check/` directory and register them in their own `plugins.xml` (topic pub/sub liveness checks, etc.), which are then loaded by `system_health_check`'s own launch/config. When adding a new node that should be health-monitored, add a matching `src-check/*.cpp` + `plugins.xml` entry rather than hand-rolling monitoring in the node itself.

### Message/service packages

Each functional area keeps its interfaces in a dedicated `*_msgs` (or `*_msgs`/`.action`/`.srv`) package separate from the node implementation (`driver_msgs`, `blue_rov_msgs`, `localization_msgs`, `planner_msgs`, `joy_common_msgs`, `p_pid_controller_msgs`, `common_msgs`). Follow this split for new interfaces instead of adding messages into an implementation package.
