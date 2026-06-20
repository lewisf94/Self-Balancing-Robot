# Architecture

## The core idea: separate the fast loop from the ecosystem

A self-balancing robot needs a control loop running at a few hundred Hz with low
timing jitter. ROS 2 gives us a fantastic *ecosystem* — simulation, RViz,
teleop, logging, navigation later — but the ROS 2 executor is **not hard
real-time**. So the design keeps those two concerns apart:

1. **The balance logic is a plain C++ class** with no ROS dependencies:
   `sbr_control::BalanceController` (built on `sbr_control::Pid`). It takes the
   tilt angle, tilt rate and a motion request, and returns wheel efforts. It can
   be unit-tested on a laptop and compiled for a microcontroller unchanged.

2. **ROS 2 wraps that core** for whichever platform you run on.

```
   sensor_msgs/Imu        +------------------------+     std_msgs/Float64MultiArray
   /imu/data  ----------> |  BalanceController     | --> /wheel_cmd  [left, right]
   geometry_msgs/Twist    |  (pitch PID + steering)|
   /cmd_vel   ----------> |                        | --> sbr_msgs/BalanceState
                          +------------------------+     /balance_state
```

## Three ways to run the same brain

| Target | How ROS 2 runs | Who runs the balance loop | Use it for |
|--------|----------------|---------------------------|------------|
| **Laptop / Gazebo** | Native ROS 2 (Jazzy) | `balance_controller_node` (rclcpp) | Developing & tuning gains safely |
| **Raspberry Pi 3B** | Native ROS 2 on the Pi | `balance_controller_node` + `imu_node` + `motor_node` | Running everything on Linux |
| **ESP32-S3** *(primary)* | **micro-ROS** firmware | The MCU itself, reusing `BalanceController` | The real robot, with tight timing |

The MPU-6050 driver publishes a fused `sensor_msgs/Imu` (orientation from a
complementary filter), which is exactly what Gazebo's IMU publishes — so the
controller sees one identical interface in all three cases.

## micro-ROS on the ESP32-S3 (primary path)

[micro-ROS](https://micro.ros.org) puts a ROS 2 node *inside* a microcontroller
(FreeRTOS + a thin client library). The ESP32-S3 runs the IMU read → PID →
motor PWM loop locally (deterministic timing), and exchanges messages with the
rest of the ROS 2 graph through a **micro-ROS agent** running on a Linux host —
e.g. your **Ubuntu laptop**:

```
   ESP32-S3 (micro-ROS client)            Ubuntu laptop (ROS 2 + agent)
  +-----------------------------+        +-------------------------------+
  | MPU-6050 -> PID -> TB6612   |  USB/  |  micro_ros_agent              |
  | publishes /imu/data,        |  Wi-Fi |  rviz2 / rqt / rosbag         |
  | /balance_state              | <====> |  teleop_twist_keyboard        |
  | subscribes /cmd_vel         |        |  (and the Gazebo sim)         |
  +-----------------------------+        +-------------------------------+
```

Transport is either **serial over USB** (simplest, great for tuning at a desk)
or **Wi-Fi/UDP** (untethered). The firmware lives in `firmware/` and links the
same `BalanceController` source as `sbr_control`.

> **Why not full ROS 2 on the ESP32-S3?** You can't — rclcpp/rclpy need a Linux
> userland. micro-ROS is the supported way to be a first-class ROS 2 participant
> from a microcontroller. If you'd rather keep the MCU as plain Arduino firmware
> and only use ROS 2 on the laptop for sim/telemetry, that's a valid simpler
> option too.

## Nodes and topics

| Node | Package | Lang | Publishes | Subscribes |
|------|---------|------|-----------|------------|
| `balance_controller_node` | `sbr_control` | C++ | `/wheel_cmd`, `/balance_state` | `/imu/data`, `/cmd_vel` |
| `imu_node` | `sbr_drivers` | Py | `/imu/data` | — |
| `motor_node` | `sbr_drivers` | Py | — | `/wheel_cmd` |
| Gazebo IMU (bridged) | `sbr_simulation` | — | `/imu/data` | — |
| `wheel_effort_controller` | `ros2_control` | — | wheel effort | `/wheel_effort_controller/commands` |

In simulation, `/wheel_cmd` is remapped onto
`/wheel_effort_controller/commands`, so the controller drives the Gazebo wheels
through `ros2_control` exactly as it would drive `motor_node` on hardware.

## Consequence of having no wheel encoders

The TT motors have no encoders, so there is **no wheel odometry**:

- Balance is closed-loop on **pitch only** (from the IMU).
- Forward/back motion is **open-loop**: a `/cmd_vel.linear.x` request leans the
  pitch setpoint so the robot accelerates; there is no velocity feedback, so it
  will **drift** and cannot hold a precise position.
- Turning is an **open-loop differential** between the wheels from
  `/cmd_vel.angular.z`.

This is the expected behaviour for this class of robot. Adding encoders later
would enable an outer velocity/position loop and proper odometry for navigation
— see the roadmap in the [README](../README.md).

## Message: `sbr_msgs/BalanceState`

Published every control cycle for live debugging (plot it with `rqt_plot` or
record with `ros2 bag`): pitch, pitch rate, setpoint, the linear/angular
requests, the left/right efforts, and a `balancing` flag that goes false on a
tip-kill.
