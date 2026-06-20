# ESP32-S3 micro-ROS firmware

On-robot firmware that balances the robot directly on the **ESP32-S3**. It is
the microcontroller equivalent of `sbr_control`'s `balance_controller_node`: it
**reuses the exact ROS-free control core** and wraps it with hardware I/O
(MPU-6050, TB6612FNG) and [micro-ROS](https://micro.ros.org/) transport.

```
/cmd_vel  (geometry_msgs/Twist)   --->  [ ESP32-S3 ]  --->  /balance_state (sbr_msgs/BalanceState)
                                          |  reads MPU-6050  (I2C)
                                          |  runs sbr_control::BalanceController
                                          |  drives TB6612FNG (LEDC PWM)
```

## Reuse, not a fork of the control law

`lib/sbr_control/` **symlinks** the canonical sources — there is no second copy
of the control law to keep in sync:

| In the firmware | Symlink to |
|-----------------|------------|
| `lib/sbr_control/sbr_control/` | `../../src/sbr_control/include/sbr_control/` |
| `lib/sbr_control/pid.cpp` | `../../src/sbr_control/src/pid.cpp` |
| `lib/sbr_control/balance_controller.cpp` | `../../src/sbr_control/src/balance_controller.cpp` |
| `extra_packages/sbr_msgs` | `../../src/sbr_msgs` |

So a fix to `BalanceController` is picked up by the ROS node, the gtests **and**
this firmware. (`balance_controller_node.cpp`, which needs rclcpp, is *not*
symlinked.) Gains and pins live in [`include/sbr_config.hpp`](include/sbr_config.hpp)
and mirror `sbr_bringup/config/sbr_params.yaml` + `docs/hardware.md`.

> Symlinks need a symlink-capable checkout (fine on Linux/macOS; on Windows use
> WSL or `git config core.symlinks true`). `firmware/COLCON_IGNORE` keeps the
> symlinked `sbr_msgs` from colliding with the real package during `colcon build`.

## Prerequisites

- [PlatformIO Core](https://docs.platformio.org/en/latest/core/installation/) (`pip install platformio`)
- The **micro-ROS agent** on your laptop (needs the workspace built — it's the
  bridge between the board's serial port and the ROS 2 graph):
  ```bash
  ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200
  # (install once: sudo apt install ros-jazzy-micro-ros-agent, or build micro_ros_setup)
  ```

## Build & flash

```bash
cd firmware
pio run                       # build
pio run -t upload             # flash (set the port with -t upload --upload-port /dev/ttyACM0)
pio device monitor            # serial monitor
```

### One-time: custom message (`sbr_msgs`) into micro-ROS

`BalanceState` is a custom message, so it has to be compiled into the micro-ROS
library. `microros_extra_packages.py` (a PlatformIO pre-build hook) links
`extra_packages/sbr_msgs` into the micro-ROS build automatically. Because that
library is **built once and cached**, the reliable sequence is:

```bash
pio run                       # 1st pass fetches micro_ros_platformio
pio run -t clean_microros     # drop the cached lib so sbr_msgs gets generated
pio run                       # rebuilds the lib WITH sbr_msgs, then the firmware
```

If your `micro_ros_platformio` version ignores the hook, copy it manually:
`cp -r extra_packages/sbr_msgs .pio/libdeps/esp32-s3/micro_ros_platformio/extra_packages/`
then `pio run -t clean_microros && pio run`.

## Topics

| Topic | Type | Direction |
|-------|------|-----------|
| `/cmd_vel` | `geometry_msgs/Twist` | teleop → firmware (`linear.x`, `angular.z`) |
| `/balance_state` | `sbr_msgs/BalanceState` | firmware → telemetry (~50 Hz) |

Drive it exactly like the sim/Pi robot:
`ros2 run teleop_twist_keyboard teleop_twist_keyboard`.

## Design notes

- **Balancing is independent of the network.** `control_task` runs on **core 0**
  at `kLoopRateHz` (200 Hz) and never blocks on micro-ROS; the Arduino `loop()`
  on **core 1** owns the agent connection. Pull the USB / kill the agent and the
  robot keeps standing — `/cmd_vel` just falls back to "hold position".
- **Auto-engage.** On boot (or after a fall) the robot is past `fall_threshold`,
  so the controller holds the motors off until you stand it up — then it grabs.
- **Safety**, matching the rest of the stack:
  - tip-kill in `BalanceController` (`|pitch| > fall_threshold`),
  - stale-`/cmd_vel` watchdog (`kCmdTimeoutS`),
  - low-battery cut via the PowerBoost `LBO` pin → `STBY` LOW,
  - IMU read-failure streak → motors stopped.

## Tuning

Edit gains in `include/sbr_config.hpp` (they mirror `sbr_params.yaml`). Tune on a
stand first — see [`docs/control_tuning.md`](../docs/control_tuning.md). If the
robot drives *into* its fall, set `kInvertPitch = true`; if one wheel turns the
wrong way, flip `kInvertLeft` / `kInvertRight`.

## Caveats / before you flash

- **Confirm the GPIO map for your exact S3 board** (`docs/hardware.md`). The pin
  table came from the ESP32 DevKit-v1 bring-up; the S3 has a different strap-pin
  map — PWM on a strap pin can stop it booting.
- **USB mode:** defaults assume native-USB CDC (`/dev/ttyACM*`). For the
  UART-bridge port (`/dev/ttyUSB*`), drop the two `ARDUINO_USB_*` flags in
  `platformio.ini`.
- **LEDC API:** code uses the arduino-esp32 **v2.x** API; the platform is pinned
  to `espressif32@^6.6.0`. For v3.x, switch to `ledcAttach` / `ledcWrite(pin,…)`.
- **Not built in CI** — there's no ESP toolchain in the sandbox. CI keeps
  building the ROS packages (it ignores `firmware/`). Build locally with `pio run`.
