# Self-Balancing Robot

A two-wheeled self-balancing robot — the classic **inverted pendulum** control
problem — built as a hands-on way to put control theory into practice, and
documented openly so others can learn from the process (and the failures).

> A normal pendulum hanging down is *stable*: disturb it and it returns to rest.
> Flip it so the mass is balanced *above* the pivot and it becomes the **inverted
> pendulum** — inherently unstable. Remove the control system and it falls. A
> two-wheeled robot is exactly this: unlike a four-wheeled car it cannot stay
> upright on its own, so it needs a feedback controller reacting many times a
> second to stay balanced.

This repository contains the **ROS 2 software stack** for that robot: a portable
PID balance controller, hardware drivers, a Gazebo simulation for tuning without
risking the real hardware, and the documentation of the build.

---

## Status

🟡 **Work in progress.** The robot balances under P-control on the bench; the
ROS 2 / simulation stack here generalises that into a tunable, reusable system.
See the [roadmap](#roadmap) and [docs/build-log.md](docs/build-log.md).

## The robot

| Subsystem | Part | Notes |
|-----------|------|-------|
| Compute (primary) | **ESP32-S3** | Runs the real-time loop as micro-ROS firmware |
| Compute (fallback) | **Raspberry Pi 3B** | Runs the full ROS 2 stack natively |
| IMU | **MPU-6050** (GY-521) | Accel + gyro, fused with a complementary filter |
| Motors | 2× **DC "TT" gearmotors** (CamJam EduKit 3) | No encoders — balance is IMU-only |
| Motor driver | **TB6612FNG** | Low drop-out, thermal protection, STBY enable |
| Power | **PowerBoost 1000C** + 2000 mAh Li-ion | 5 V boost + charger |
| Extras | SSD1306 OLED, low-battery (LBO) cut-off | Telemetry + safety |

Full bill of materials, wiring and the GPIO map: **[docs/hardware.md](docs/hardware.md)**.

## How "ROS 2" runs on this robot

ROS 2 is the right backbone for the *ecosystem* (simulation, visualisation,
teleop, logging, room to grow into navigation), but it is **not hard real-time**,
and the balance loop is timing-sensitive. So the design splits the work:

```
            +------------------------------ ROS 2 graph ------------------------------+
            |                                                                         |
  +---------+----------+    /imu/data     +-----------------------+   /wheel_cmd      |
  |  IMU  (MPU-6050 /  | ---------------> |  balance_controller   | ------------------+--> motors
  |       Gazebo)      |                  |  (PID, ROS-free core)  |                   |   (TB6612)
  +--------------------+    /cmd_vel      +-----------------------+   /balance_state  |
        teleop --------------------------------->                                     |
            |                                                                         |
            +-------------------------------------------------------------------------+
```

- The **balance logic is a plain C++ class** (`sbr_control::BalanceController`)
  with **no ROS dependencies**. That one piece of code is the "brain".
- On a **Linux SBC / in simulation** it is wrapped in an `rclcpp` node.
- On the **ESP32-S3** the *same logic* compiles into **micro-ROS** firmware that
  runs the loop on the microcontroller and joins the ROS 2 graph over serial/Wi-Fi.

This is why the controller was deliberately kept ROS-free — see
[docs/architecture.md](docs/architecture.md).

## Repository layout

```
src/
  sbr_msgs/         Custom messages (BalanceState telemetry)
  sbr_description/  URDF/xacro model, RViz config, display launch
  sbr_control/      C++ balance controller (portable core + ROS node + tests)
  sbr_drivers/      Python nodes: MPU-6050 IMU + TB6612 motor driver (Pi path)
  sbr_simulation/   Gazebo world, ros2_control, bridge + sim launch
  sbr_bringup/      Top-level launch files + parameters
docs/               Architecture, hardware, setup, tuning, simulation, build log
firmware/           (planned) ESP32-S3 micro-ROS firmware
```

## Quickstart — simulation

Requires ROS 2 **Jazzy** + Gazebo **Harmonic** on Ubuntu 24.04. Full instructions:
[docs/setup.md](docs/setup.md).

```bash
# from the repo root
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash

# launch Gazebo + the balancing robot
ros2 launch sbr_simulation simulation.launch.py

# drive it (separate terminal, after sourcing)
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Tune the gains live in `src/sbr_simulation/config/sim_balance.yaml` —
see [docs/control_tuning.md](docs/control_tuning.md).

## Quickstart — real robot (Raspberry Pi path)

```bash
pip install smbus2 lgpio          # hardware I2C + GPIO (Pi only)
colcon build --symlink-install
source install/setup.bash
ros2 launch sbr_bringup robot.launch.py
```

The ESP32-S3 (micro-ROS) path lives in `firmware/` — see
[docs/architecture.md](docs/architecture.md).

## Documentation

| Doc | What's in it |
|-----|--------------|
| [architecture.md](docs/architecture.md) | Nodes, topics, ESP32 vs Pi, the shared control core |
| [hardware.md](docs/hardware.md) | BOM, wiring, GPIO map, power tree, hard-won lessons |
| [setup.md](docs/setup.md) | Installing ROS 2, building, running |
| [simulation.md](docs/simulation.md) | Running and tuning in Gazebo |
| [control_tuning.md](docs/control_tuning.md) | Complementary filter + PID tuning recipe |
| [build-log.md](docs/build-log.md) | The real build diary: power, drivers, IMU, first balance |

## Roadmap

- [x] Bench P-controller balancing
- [x] ROS 2 workspace + portable control core
- [x] Gazebo simulation for gain tuning
- [ ] Add D and I terms; find damping sweet-spot
- [ ] ESP32-S3 micro-ROS firmware
- [ ] Tip-kill + low-battery safety wired into firmware
- [ ] Live PID tuning from a phone / rqt
- [ ] Final chassis + cable management

## Credits & references

Built and documented by **Lewis Fowler** ([@lewisf94](https://github.com/lewisf94)).
References that helped along the way are collected in
[docs/build-log.md](docs/build-log.md#references).

## License

[MIT](LICENSE) © 2026 Lewis Fowler.
