# Getting started

New here? This is the one-page route from "fresh clone" to "watching the robot
balance." It's written for beginners — pick a path and follow the links for the
detail.

**What this is:** software for a two-wheeled robot that balances itself upright
(an inverted-pendulum problem) — like a tiny Segway. The *same* balancing code
runs in three places: a **Gazebo simulation**, a **Raspberry Pi**, and an
**ESP32-S3** microcontroller. See [docs/architecture.md](docs/architecture.md)
for how the pieces fit together.

## Pick your path

| I want to… | Go to |
|------------|-------|
| **See it balance in simulation** (no hardware — start here) | [Run the simulation](#run-the-simulation) |
| **Build the real robot** | [Build the real robot](#build-the-real-robot) |
| **Understand the design** | [docs/architecture.md](docs/architecture.md) |

---

## Run the simulation

The safest, cheapest way to see it work — a fall in sim costs nothing.

### 0. You need Linux
- **Ubuntu 24.04** → continue below.
- **Windows** → set up WSL2 first ([docs/windows-wsl.md](docs/windows-wsl.md)), then continue.
- **macOS** → ROS 2 Jazzy + Gazebo Harmonic aren't practical natively; use an
  Ubuntu 24.04 VM or a separate Linux machine.

### 1. Install ROS 2 Jazzy + the sim packages
Install ROS 2 Jazzy (<https://docs.ros.org/en/jazzy/Installation.html> →
`ros-jazzy-desktop`), then:
```bash
sudo apt install -y \
  ros-jazzy-ros-gz ros-jazzy-gz-ros2-control \
  ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
  ros-jazzy-teleop-twist-keyboard \
  python3-colcon-common-extensions python3-rosdep
sudo rosdep init 2>/dev/null; rosdep update
```

### 2. Get the code and build
```bash
git clone https://github.com/lewisf94/Self-Balancing-Robot.git
cd Self-Balancing-Robot
source /opt/ros/jazzy/setup.bash
make deps        # install dependencies (rosdep)
make build       # compile the workspace
make test        # optional: run the control-core unit tests
```

### 3. Run it
```bash
make sim         # opens Gazebo with the robot  (use `make sim-rviz` to add RViz)
```
Drive it from a **second terminal** (every new terminal needs sourcing):
```bash
cd Self-Balancing-Robot
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### What success looks like
- A Gazebo window with a **blue box on two black wheels**, standing upright (it
  may jitter — that's the controller doing its job).
- `ros2 topic echo /balance_state` shows `balancing: true` and a small `pitch`.
- The teleop keys (`i`, `,`, `j`, `l`) lean and drive it.

### If it misbehaves
- **Falls over immediately** → usually the pitch sign; flip `invert_pitch` in
  `src/sbr_simulation/config/sim_balance.yaml` and relaunch.
- **Errors / black window / no IMU data** → the troubleshooting table in
  [docs/simulation.md](docs/simulation.md#troubleshooting) covers the usual suspects.
- **Stuck?** Grab the output of `make sim` plus `ros2 topic list` and
  `ros2 topic echo /imu/data --once` — that pinpoints most problems.

### Tune it
Edit gains in `src/sbr_simulation/config/sim_balance.yaml`, relaunch, and watch
`ros2 topic echo /balance_state`. Full recipe: [docs/control_tuning.md](docs/control_tuning.md).

---

## Build the real robot

> ⚠️ Balancing robots flail. **Read the pre-flight & safety checklist in
> [docs/security.md](docs/security.md)** before powering the motors, and do your
> first tests with the wheels off the ground (robot on a stand).

1. **Parts & wiring** — [docs/hardware.md](docs/hardware.md): bill of materials,
   GPIO map, power tree, wiring diagrams.
2. **ESP32-S3 firmware (primary)** — [firmware/README.md](firmware/README.md):
   build and flash with PlatformIO (`pio run`). It reuses the same balancing
   code as the sim, so tuning carries over.
3. **Raspberry Pi (alternative)** — see "Run on the real robot" in
   [docs/setup.md](docs/setup.md).

---

## Repo map

| Path | What |
|------|------|
| `src/sbr_control/` | The portable balance "brain" (PID + controller) + unit tests |
| `src/sbr_msgs/` | `BalanceState` telemetry message |
| `src/sbr_description/` | Robot model (URDF/xacro) |
| `src/sbr_drivers/` | IMU + motor drivers (Raspberry Pi path) |
| `src/sbr_simulation/` | Gazebo world, controllers, sim launch |
| `src/sbr_bringup/` | Top-level launch + parameters |
| `firmware/` | ESP32-S3 micro-ROS firmware (reuses the control core by symlink) |
| `docs/` | Hardware, setup, simulation, tuning, architecture, security |

## All the docs
- [docs/setup.md](docs/setup.md) — full install / build / run
- [docs/simulation.md](docs/simulation.md) — sim detail + troubleshooting
- [docs/windows-wsl.md](docs/windows-wsl.md) — running on a Windows laptop
- [docs/hardware.md](docs/hardware.md) — BOM, wiring, GPIO map
- [docs/control_tuning.md](docs/control_tuning.md) — tuning the balance
- [docs/architecture.md](docs/architecture.md) — how it all fits together
- [docs/security.md](docs/security.md) — security posture + pre-flight checklist
- [docs/build-log.md](docs/build-log.md) — running notes
- [firmware/README.md](firmware/README.md) — ESP32-S3 firmware
