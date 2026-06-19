# Control & tuning

## 1. Estimating the tilt angle

The MPU-6050 gives two noisy, complementary signals:

- the **accelerometer** measures the gravity vector → an absolute tilt angle,
  but it's noisy and corrupted by the robot's own acceleration;
- the **gyroscope** measures tilt *rate* → smooth and fast, but integrating it
  to an angle **drifts** over time.

A **complementary filter** fuses them — trust the gyro short-term, the
accelerometer long-term:

```
angle = alpha * (angle + gyro_rate * dt) + (1 - alpha) * accel_angle
```

with `alpha` ≈ 0.98. This is implemented in `imu_node` (Python, Pi path) and
should be mirrored in the ESP32 firmware. A Kalman filter is the more advanced
alternative; the complementary filter is far simpler and plenty good enough
here. Background reading:
[Van Hunter Adams — complementary filters](https://vanhunteradams.com/Pico/ReactionWheel/Complementary_Filters.html).

## 2. The control law

`BalanceController::update()` runs three small pieces each cycle:

1. **Pitch PID (the balance loop).** Error = `setpoint − pitch`. The derivative
   term uses the **gyro rate directly** (derivative-on-measurement) so we don't
   amplify noise by differentiating the angle. Output is normalized to ~[-1, 1].
2. **Open-loop drive.** `cmd_vel.linear.x` leans the pitch *setpoint*
   (`lean_per_velocity`) so the robot tips slightly and accelerates. With no
   encoders there is no velocity feedback — this is intentional.
3. **Steering.** `cmd_vel.angular.z` adds a differential effort between the
   wheels (`steer_gain`).

Final per-wheel command = `clamp(pid ∓ steer) * output_scale`. If
`|pitch| > fall_threshold` the motors are cut (**tip-kill**).

## 3. Tuning recipe (do it in simulation first)

Tune in Gazebo (`ros2 launch sbr_simulation simulation.launch.py`) where a fall
costs nothing, then transfer gains to hardware. Plot `/balance_state` with
`rqt_plot /balance_state/pitch /balance_state/left_effort` while you tune.

Work the gains in this order — the classic P → D → I progression:

1. **Set `pitch_ki = 0`, `pitch_kd = 0`.** Raise **`pitch_kp`** until the robot
   reacts firmly to a tilt and roughly holds itself, accepting some oscillation.
   (On the bench, `Kp ≈ 5` gave a gentle proportional response; `Kp ≈ 15` was
   aggressive. Sim torque scaling differs — expect different numbers.)
2. **Add `pitch_kd`** to damp the oscillation. Increase until the wobble settles
   quickly; too much makes it twitchy/noisy.
3. **Add a little `pitch_ki`** only if there's a steady lean it never corrects
   (e.g. an off-centre CoG). Keep it small and rely on `integral_limit` to stop
   wind-up.
4. **Trim with `pitch_offset`** so the balance point matches the chassis'
   true upright (its CoG is rarely exactly over the axle).
5. **Confirm the tip-kill** (`fall_threshold`, ~0.78 rad ≈ 45°) cuts the motors
   when pushed past recovery.

## 4. Parameter reference

| Parameter | Meaning | Start |
|-----------|---------|-------|
| `loop_rate` | Control frequency [Hz] | 200 |
| `pitch_kp` | Proportional gain | tune |
| `pitch_ki` | Integral gain | 0 |
| `pitch_kd` | Derivative gain (on gyro rate) | tune |
| `integral_limit` | Anti-wind-up clamp on the integral | 0.5 |
| `output_limit` | Clamp on normalized PID output | 1.0 |
| `output_scale` | Normalized → output units (duty on HW, N·m in sim) | 1.0 / 3.5 |
| `pitch_offset` | Balance-point trim [rad] | 0.0 |
| `fall_threshold` | Tip-kill angle [rad] | 0.78 |
| `lean_per_velocity` | Setpoint lean per m/s of forward request | 0.08 |
| `steer_gain` | Differential effort per rad/s of yaw request | 0.4 |
| `invert_pitch` | Flip pitch sign for IMU mounting | false |
| `cmd_timeout` | Drop stale `/cmd_vel` after this [s] | 0.5 |

Hardware values live in `sbr_bringup/config/sbr_params.yaml`; simulation values
in `sbr_simulation/config/sim_balance.yaml`.

## 5. Common failure modes

| Symptom | Likely cause | Fix |
|---------|--------------|-----|
| Falls instantly the "wrong" way | Pitch sign inverted | `invert_pitch: true` |
| Drives off in one direction | `pitch_offset` wrong, or motor inverted | trim offset; check `invert_left/right` |
| Fast buzzing oscillation | `pitch_kp` too high / `pitch_kd` too low | lower Kp, add Kd |
| Slow growing wobble | Not enough damping | raise `pitch_kd` |
| Slowly leans then gives up | Needs a touch of integral | small `pitch_ki` |
| Motors whine audibly | PWM at 1 kHz | raise PWM to ~20 kHz (firmware/LEDC) |
