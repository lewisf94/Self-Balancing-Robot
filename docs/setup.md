# Setup

## Choose your ROS 2 distribution

This stack targets **ROS 2 Jazzy** on **Ubuntu 24.04** with **Gazebo Harmonic**.
That is the current LTS (supported to 2029) and the recommended choice for a
fresh install.

> **On Ubuntu 22.04?** Use **ROS 2 Humble** instead. The nodes, messages and
> launch files are the same, but the **simulation** package targets Gazebo
> Harmonic — on Humble you'd pair with Gazebo Fortress/Garden and adjust the
> plugin names in `sbr_description/urdf/sbr.gazebo.xacro` and the world SDF.
> Tell me which Ubuntu your laptop runs and I'll wire the sim to match.

## 1. Install ROS 2 (Jazzy)

Follow the official guide:
<https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html>. In short:

```bash
sudo apt update && sudo apt install ros-jazzy-desktop
# extras this project uses:
sudo apt install ros-jazzy-ros-gz ros-jazzy-gz-ros2-control \
                 ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
                 ros-jazzy-teleop-twist-keyboard \
                 python3-colcon-common-extensions python3-rosdep
```

Initialise rosdep once:

```bash
sudo rosdep init   # skip if already done
rosdep update
```

## 2. Get the code and build

```bash
git clone https://github.com/lewisf94/Self-Balancing-Robot.git
cd Self-Balancing-Robot

source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Run the unit tests for the control core:

```bash
colcon test --packages-select sbr_control
colcon test-result --verbose
```

## 3. Run the simulation

```bash
ros2 launch sbr_simulation simulation.launch.py            # add use_rviz:=true
```

Drive it from another (sourced) terminal:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

More detail and troubleshooting: [simulation.md](simulation.md).

## 4. Run on the real robot

### Raspberry Pi 3B (full ROS 2)

Install ROS 2 on the Pi (Ubuntu Server 24.04 + `ros-jazzy-ros-base`), then:

```bash
pip install smbus2 lgpio          # I2C + GPIO access
sudo usermod -aG i2c,gpio "$USER" # then re-login
colcon build --symlink-install
source install/setup.bash
ros2 launch sbr_bringup robot.launch.py
```

Set `mock: true` in `sbr_bringup/config/sbr_params.yaml` to dry-run the nodes
without hardware (they also fall back to mock automatically if the bus can't be
opened).

### ESP32-S3 (micro-ROS — primary)

The microcontroller runs the balance loop and joins the ROS 2 graph through a
**micro-ROS agent** on your laptop. Outline (firmware lands in `firmware/`):

```bash
# on the laptop: run the agent (serial transport over USB)
sudo apt install ros-jazzy-micro-ros-agent       # or build micro_ros_setup
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200
```

The firmware is built with PlatformIO/ESP-IDF using the micro-ROS component and
links the shared `BalanceController` source. See
[architecture.md](architecture.md#micro-ros-on-the-esp32-s3-primary-path).

## Using your Ubuntu laptop as the ground station

Your laptop is the ideal dev machine and **micro-ROS agent host**: build and run
the Gazebo sim there to tune gains, run `micro_ros_agent` to talk to the ESP32,
and use `rviz2` / `rqt_plot` / `ros2 bag` for visualisation and logging — no Pi
required for the primary path.

## Troubleshooting

- **`colcon build` can't find a dependency:** re-run
  `rosdep install --from-paths src --ignore-src -r -y`.
- **`ros2: command not found`:** you didn't `source /opt/ros/<distro>/setup.bash`.
- **Your packages aren't found at runtime:** `source install/setup.bash` in each
  new terminal (or add it to `~/.bashrc`).
