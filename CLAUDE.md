# CLAUDE.md

Guidance for Claude Code (and other agents) working in this repository.

## What this is

ROS 2 software for a **two-wheeled self-balancing robot** (an inverted-pendulum
control problem). The robot has an **MPU-6050 IMU**, two **DC "TT" gearmotors
without encoders**, and a **TB6612FNG** driver. Balance is therefore **IMU-only**
— there is no wheel odometry, no closed-loop velocity/position control, and the
robot drifts in position by design.

Target compute: **ESP32-S3** (primary, via micro-ROS) or **Raspberry Pi 3B**
(fallback, full ROS 2). ROS distro: **Jazzy**. Simulator: **Gazebo Harmonic**.

## Golden rule: keep the control core ROS-free

`sbr_control::BalanceController` and `sbr_control::Pid`
(`src/sbr_control/include/sbr_control/`, `src/sbr_control/src/pid.cpp`,
`balance_controller.cpp`) must stay **free of any ROS / rclcpp includes**. This
is the one piece of logic shared between the Linux/sim node and the ESP32-S3
micro-ROS firmware (`firmware/`, which compiles these exact files via the
`firmware/lib/sbr_control` symlinks). Only `balance_controller_node.cpp` may use rclcpp.
If you change the control law, update the gtests in
`src/sbr_control/test/test_balance_controller.cpp`.

## Build / test / run

```bash
# Build (from repo root)
colcon build --symlink-install
source install/setup.bash

# Run the C++ unit tests
colcon test --packages-select sbr_control
colcon test-result --verbose

# Simulation (Gazebo + balancing robot)
ros2 launch sbr_simulation simulation.launch.py            # add use_rviz:=true
ros2 run teleop_twist_keyboard teleop_twist_keyboard       # drive it

# Real robot (Raspberry Pi path)
ros2 launch sbr_bringup robot.launch.py

# View the model only
ros2 launch sbr_description display.launch.py
```

There is **no ROS 2 toolchain in this sandbox** — validate with
`python3 -m py_compile` for Python and rely on CI (`.github/workflows/ci.yml`)
for the real `colcon build`/`test`. CI builds the core packages on Jazzy and
skips `sbr_simulation` (heavy Gazebo deps).

The **ESP32-S3 firmware** lives in `firmware/` (PlatformIO + micro-ROS). It is
built locally with `pio run` (no ESP toolchain in the sandbox) and is ignored by
colcon/CI via `firmware/COLCON_IGNORE`. It reuses the control core by symlink, so
gains/pins in `firmware/include/sbr_config.hpp` must track `sbr_params.yaml`.

## Packages

| Package | Type | Role |
|---------|------|------|
| `sbr_msgs` | ament_cmake (rosidl) | `BalanceState.msg` telemetry |
| `sbr_description` | ament_cmake | URDF/xacro, RViz, `display.launch.py` |
| `sbr_control` | ament_cmake (C++) | Portable control core + `balance_controller_node` + gtests |
| `sbr_drivers` | ament_python | `imu_node` (MPU-6050), `motor_node` (TB6612) — Pi path |
| `sbr_simulation` | ament_cmake | Gazebo world, `ros2_control`, gz bridge, sim launch |
| `sbr_bringup` | ament_python | Top-level launch + `config/sbr_params.yaml` |

## Key topics

| Topic | Type | Direction |
|-------|------|-----------|
| `/imu/data` | `sensor_msgs/Imu` | IMU/Gazebo → controller |
| `/cmd_vel` | `geometry_msgs/Twist` | teleop → controller |
| `/wheel_cmd` | `std_msgs/Float64MultiArray` `[left, right]` | controller → motors (remapped to `/wheel_effort_controller/commands` in sim) |
| `/balance_state` | `sbr_msgs/BalanceState` | controller → telemetry |

## Conventions & gotchas

- **Pitch sign:** positive pitch = leaning forward = positive (forward) wheel
  effort. IMU mounting matters; use the `invert_pitch` param to flip.
- **Hardware deps are lazy-imported.** `smbus2` (I2C) and `lgpio` (GPIO) are
  imported inside the driver classes so the package builds and runs in `mock`
  mode anywhere. Do **not** add them to `package.xml`.
- **Drivers have a `mock` parameter** and auto-fall-back to mock if the hardware
  can't be opened — keep that behaviour when editing.
- **Same Imu interface in sim and hardware:** `imu_node` publishes a fused
  orientation so the controller reads `sensor_msgs/Imu` identically to Gazebo.
- **Two parameter files:** `sbr_bringup/config/sbr_params.yaml` (hardware,
  `output_scale: 1.0`) and `sbr_simulation/config/sim_balance.yaml` (sim,
  `output_scale` maps normalized effort → N·m torque). Keep them in sync in
  structure.
- **Pin numbers are BCM** in the Pi `motor_node`. The canonical ESP32 GPIO map
  lives in `docs/hardware.md` / firmware, not here.
- **Safety:** `fall_threshold` (~0.78 rad ≈ 45°) cuts the motors (tip-kill).
  Preserve the motor watchdog in `motor_node` and the fall check in the
  controller.

## Git

- **Do not create new branches unless the user explicitly asks.** Commit to the
  current branch (`main`, now that the initial PR is merged).
- **Do not open a PR unless the user explicitly asks.**
- Commit as `lewisf94 <85638536+lewisf94@users.noreply.github.com>` (the
  GitHub noreply identity — never put a personal email in commits or files).
