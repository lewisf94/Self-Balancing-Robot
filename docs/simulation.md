# Simulation

The Gazebo (Harmonic) simulation lets you tune the balance controller without
risking the real robot — a fall in sim costs nothing. The **same**
`balance_controller_node` and the **same** `sensor_msgs/Imu` interface are used,
so gains and behaviour transfer to hardware.

## What's modelled

- The URDF/xacro chassis as an **inverted pendulum**: a tall body whose centre
  of mass sits well above the wheel axle, on two continuous-joint wheels.
- A **Gazebo IMU** on `imu_link`, bridged to `/imu/data`.
- `ros2_control` (via `gz_ros2_control`) exposing an **effort** command on each
  wheel joint, driven by a `forward_command_controller`.

```
 balance_controller  --/wheel_cmd-->  (remap)  --> /wheel_effort_controller/commands
                                                   --> gz_ros2_control --> wheel torque
        ^                                                                     |
        |  /imu/data  <-- ros_gz_bridge <-- Gazebo IMU sensor <---------------+
```

## Run it

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch sbr_simulation simulation.launch.py            # use_rviz:=true to add RViz
```

The launch file starts Gazebo, spawns the robot, brings up the controllers in
order (`joint_state_broadcaster` → `wheel_effort_controller` → balance
controller), and bridges the clock + IMU.

Drive it from another sourced terminal:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Tuning in sim

Edit `src/sbr_simulation/config/sim_balance.yaml` and relaunch. Watch the live
state:

```bash
ros2 topic echo /balance_state
rqt_plot /balance_state/pitch /balance_state/pitch_setpoint /balance_state/left_effort
```

Follow the recipe in [control_tuning.md](control_tuning.md). Note `output_scale`
in sim maps the normalized [-1, 1] command onto wheel **torque (N·m)**, so the
numeric gains differ from hardware (where the command is PWM duty).

## Troubleshooting

| Symptom | Cause / fix |
|---------|-------------|
| Robot falls over immediately | Expected before tuning. Lower `output_scale` or raise `pitch_kd`; confirm pitch sign (`invert_pitch`). |
| `/imu/data` has no publishers | The gz→ROS bridge topic name must match the sensor. Check `gz topic -l` for the IMU topic and align `config/gz_bridge.yaml`. |
| Controllers never activate | `gz_ros2_control` couldn't load `controllers.yaml`. Check the `controllers_config` path passed by the launch file and that `ros-<distro>-gz-ros2-control` is installed. |
| `gz_sim.launch.py` not found | Install `ros-<distro>-ros-gz`. |
| Wheels spin but the robot won't balance | Effort too weak/strong — adjust `output_scale`; verify the command joint order is `[left, right]`. |
| Time looks frozen / TF errors | `use_sim_time` must be true (it is in the launch); ensure `/clock` is bridged. |

## Notes & limitations

- **No wheel encoders** are modelled for control: like the real robot, balance
  is pitch-only and forward motion is open-loop, so the robot drifts.
- This is a tuning aid, not a high-fidelity digital twin — friction, motor
  dynamics and IMU noise are approximate. Use it to get *close*, then finish
  tuning on hardware.
