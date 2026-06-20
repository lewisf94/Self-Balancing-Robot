# Running on a Windows laptop (WSL2)

ROS 2 Jazzy and Gazebo Harmonic have no practical **native Windows** path, so on
Windows the supported route is **WSL2** — a real Ubuntu 24.04 running inside
Windows. Build, tests, the Gazebo sim, and the firmware build all work there, and
GUI windows (Gazebo, RViz) appear on your Windows desktop via WSLg.

| Task | Native Windows | WSL2 (Ubuntu 24.04) | Dual-boot Ubuntu |
|------|----------------|---------------------|------------------|
| ROS 2 Jazzy | not practical | ✅ | ✅ |
| Gazebo sim (`make sim`) | ✗ | ✅ (WSLg GUI) | ✅ (fastest) |
| Build / test the ROS code | ✗ | ✅ | ✅ |
| ESP32 firmware (`pio run`) | ⚠️ symlinks need care | ✅ | ✅ |
| Flash the ESP32 over USB | ✅ | ⚠️ needs `usbipd-win` | ✅ |

> **Why not native Windows for the firmware?** This repo reuses the control core
> through **symlinks** (`firmware/lib/sbr_control`), which native-Windows git
> checkouts mangle by default. Build the firmware in WSL2, or enable Windows
> Developer Mode and `git config --global core.symlinks true` before cloning.

## 1. Install WSL2 + Ubuntu 24.04

In **PowerShell (Administrator)**:

```powershell
wsl --install -d Ubuntu-24.04
```

Reboot if prompted. The Ubuntu window then asks for a **Linux username and
password** (separate from your Windows login; the password is what `sudo` uses).

Confirm you're on WSL **2**:

```powershell
wsl -l -v        # VERSION should be 2
# if it shows 1:  wsl --set-version Ubuntu-24.04 2
```

## 2. Update Ubuntu

Inside the Ubuntu terminal:

```bash
sudo apt update && sudo apt upgrade -y
```

## 3. Confirm GUI windows work — before the big install

```bash
sudo apt install -y x11-apps
xeyes        # a pair of eyes should appear on your Windows desktop; Ctrl+C to close
```

If nothing appears: run `wsl --update` then `wsl --shutdown` in PowerShell,
reopen Ubuntu, and retry. Built-in GUI (WSLg) needs Windows 11 or a fully
updated Windows 10.

## 4. Keep the code on the Linux filesystem

Work under your Linux home (`~`), **not** under `/mnt/c/...`. The Windows mount
is slow and does not preserve the symlinks this repo uses.

```bash
cd ~
git clone https://github.com/lewisf94/Self-Balancing-Robot.git
cd Self-Balancing-Robot
```

## 5. Install ROS 2 + the sim packages, then build and run

Install ROS 2 Jazzy (see <https://docs.ros.org/en/jazzy/Installation.html> →
`ros-jazzy-desktop`), then:

```bash
sudo apt install -y \
  ros-jazzy-ros-gz ros-jazzy-gz-ros2-control \
  ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
  ros-jazzy-teleop-twist-keyboard \
  python3-colcon-common-extensions python3-rosdep
sudo rosdep init 2>/dev/null; rosdep update

source /opt/ros/jazzy/setup.bash
make deps
make build
make sim
```

Drive it from a **second** Ubuntu terminal:

```bash
cd ~/Self-Balancing-Robot
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

See `docs/setup.md` and `docs/simulation.md` for the Linux-side details, and
`docs/security.md` for the pre-flight checklist.

## Gotchas

- **Black/empty Gazebo window or very slow rendering:** WSL's GPU is virtual.
  Recent WSL renders via Direct3D and is usually fine; if you get a black window,
  run `export LIBGL_ALWAYS_SOFTWARE=1` before `make sim` (software rendering —
  slower but reliable). For smooth sim performance, dual-boot Ubuntu.
- **New terminals start unsourced:** run
  `source /opt/ros/jazzy/setup.bash && source install/setup.bash` in each.
- **Flashing the ESP32 later:** the board appears as a COM port to Windows and
  PlatformIO flashes it fine. To run the micro-ROS *agent* from WSL against that
  board, pass the USB device in with [`usbipd-win`](https://github.com/dorssel/usbipd-win).
