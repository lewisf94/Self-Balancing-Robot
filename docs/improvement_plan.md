# Improvement Plan — Reliability, Accuracy, Tests, and Next-Level Features

**Audience:** an AI coding agent (or human) executing this plan step by step with
no prior context. Everything needed is in this file plus the repo. Do **not**
assume anything not written here — when in doubt, read the file in question
first.

**Status:** designed 2026-07-06 from a full-code review. Nothing in this plan is
implemented yet.

---

## How to execute this plan (read first)

1. **Work top to bottom.** Phases are ordered by priority (safety → correctness
   → tests/CI → features → docs). Steps within a phase are ordered so the repo
   is always green after every commit.
2. **One step = one commit = one push.** Commit straight to `main` and push
   after every step (this is the repo rule, see CLAUDE.md). Author:
   `lewisf94 <85638536+lewisf94@users.noreply.github.com>`. Use the commit
   message given at the end of each step.
3. **Line numbers in this plan are hints, not gospel.** They were correct when
   the plan was written; always `Read` the file and match on code content, not
   line position.
4. **The golden rule is inviolate:**
   `src/sbr_control/include/sbr_control/*.hpp`, `src/sbr_control/src/pid.cpp`,
   `src/sbr_control/src/balance_controller.cpp` must never include any ROS
   header. They are compiled unchanged into the ESP32-S3 firmware via symlinks
   at `firmware/lib/sbr_control/`. Any change to these files must also be
   verified with `cd firmware && pio run`.
5. **Green gate — run after every step before committing:**
   ```bash
   cd /home/lewis-fowler/Documents/Self-Balancing-Robot
   source /opt/ros/jazzy/setup.bash
   colcon build --symlink-install            # or --packages-select as listed per step
   source install/setup.bash
   colcon test --packages-select sbr_control --return-code-on-test-failure
   colcon test-result --verbose
   python3 -m pytest src/sbr_drivers/test src/sbr_bringup/test -v
   ```
   For steps that touch the shared control core or `firmware/`, additionally:
   ```bash
   cd firmware && pio run && cd ..
   ```
   (`pio` lives at `~/.platformio/penv/bin/pio`, symlinked into `~/.local/bin`;
   if `pio` is not on PATH run `export PATH="$HOME/.local/bin:$PATH"`.)
6. **Environment facts (verified):** this machine has the full ROS 2 Jazzy
   toolchain, Gazebo Harmonic, colcon, and PlatformIO. The sim can be launched
   headless: `ros2 launch sbr_simulation simulation.launch.py headless:=true`.
   git is configured with the correct identity and a stored credential —
   `git push origin main` just works. **No ESP32 board is connected** — steps
   marked **[HW]** compile here but need hardware for runtime validation.
7. **Kill sim processes between runs** or they pile up:
   ```bash
   pkill -9 -f "gz sim|balance_controller_node|parameter_bridge|robot_state_publisher|ros2 launch sbr"
   ```

## Context — why this plan exists

The project is a two-wheeled inverted-pendulum robot (MPU-6050 IMU, TT
gearmotors **without encoders**, TB6612FNG driver) with three compute targets
sharing one ROS-free control core: Gazebo sim (rclcpp node), Raspberry Pi
(same node + Python drivers), and ESP32-S3 micro-ROS firmware.

Current state: the sim balances well (pitch PD tuned: kp=6.0, kd=0.3,
output_scale=0.4 → RMS pitch ≈0.07°, zero saturation), and the firmware builds
clean with PlatformIO (RAM 13.9%, Flash 11%). Two known problems motivated this
review:

- **The robot "runs away"** — with no encoders, velocity is unobservable from
  pitch alone, so the robot settles into a tiny lean and accelerates
  indefinitely. No pitch gain can fix this; it needs a velocity signal
  (Phase 3 adds a gated outer loop; Phase 4 preps hardware encoders).
- **A full-code review found safety and robustness gaps** — most seriously:
  if the IMU goes stale, the controller keeps driving motors on old data
  forever, and no watchdog catches it (Phase 0 fixes this).

Complete findings list (each is fixed by the step in brackets):

| # | Finding | Fix |
|---|---------|-----|
| 1 | No IMU-freshness watchdog in `balance_controller_node.cpp`; motor watchdog only trips on command *absence*, and the controller never stops publishing | 0.1 |
| 2 | `imu_node.py` silently falls back to mock on hardware failure → publishes "perfectly upright" forever; a fallen robot gets driven as if level | 0.2 |
| 3 | No `respawn` on any node in `robot.launch.py` | 0.3 |
| 4 | `mpu6050.py` (Pi): no WHO_AM_I identity check, no gyro-bias calibration | 0.4 |
| 5 | Fixed nominal `dt = 1/rate` in controller node and imu_node instead of measured elapsed time | 1.1 |
| 6 | `motor_node.py` code default `pwm_frequency=1000` (docs + params say 20000) | 1.2 |
| 7 | `robot.launch.py` passes raw `Command(...)` without `ParameterValue(..., value_type=str)` | 1.3 |
| 8 | No validation that `loop_rate`/`publish_rate` > 0 before `create_timer(1/rate)` | 1.4 |
| 9 | `main()` finally blocks in both Python drivers can NameError if `__init__` throws | 1.5 |
| 10 | Only 6 gtests; ki/anti-windup, kd path, lean, offset, fall recovery, reset, dt≤0 all untested; no direct Pid tests | 2.1, 2.2 |
| 11 | Tip-kill auto-resumes the instant \|pitch\| < threshold — chattering risk at the boundary | 2.2 |
| 12 | Python drivers have zero behavioral tests | 2.3 |
| 13 | Firmware gains duplicated by hand from `sbr_params.yaml` with no automated check | 2.4 |
| 14 | CI never builds `sbr_simulation` or the firmware; no linting | 2.5, 2.6 |
| 15 | Sim runaway (velocity unobservable) | 3.1 |
| 16 | Firmware lacks gyro-bias calibration (Pi gets it in 0.4) | 5.1 |
| 17 | No automated end-to-end "does it still balance" regression test | 6.1, 6.2 |
| 18 | README stale (says firmware "(planned)"), LICENSE says "Lewis" not "Lewis Fowler" | 8.1 |

---

## Phase 0 — Safety (do these first)

### Step 0.1 — IMU-freshness watchdog in the controller node

**File:** `src/sbr_control/src/balance_controller_node.cpp` (node only — the
ROS-free core is untouched).

The node currently has a one-shot `have_imu_` flag and a `cmd_timeout_` for
`/cmd_vel`, but nothing checks IMU *age*. Model the fix on the existing
`cmd_timeout_` pattern:

1. In the constructor, next to the `cmd_timeout_` declaration, add:
   ```cpp
   imu_timeout_ = declare_parameter<double>("imu_timeout", 0.2);
   ```
2. Add members `double imu_timeout_{0.2};` and `rclcpp::Time last_imu_time_;`.
   Initialize `last_imu_time_ = now();` in the constructor beside
   `last_cmd_time_ = now();`.
3. In `on_imu()`, set `last_imu_time_ = now();` alongside `have_imu_ = true;`.
4. Also in `on_imu()`, at the very top, reject samples flagged invalid (the
   degraded-IMU convention introduced in Step 0.2):
   ```cpp
   if (msg->orientation_covariance[0] < 0.0) {
     RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
       "Ignoring IMU sample flagged invalid (orientation_covariance[0] < 0)");
     return;  // do not refresh last_imu_time_
   }
   ```
5. In `on_timer()`, immediately after the `!have_imu_` early-return, add:
   ```cpp
   if ((now() - last_imu_time_).seconds() > imu_timeout_) {
     controller_.reset();
     std_msgs::msg::Float64MultiArray stop_msg;
     stop_msg.data = {0.0, 0.0};
     wheel_pub_->publish(stop_msg);           // actively command zero
     sbr_msgs::msg::BalanceState state;
     state.header.stamp = now();
     state.header.frame_id = "base_link";
     state.pitch = pitch_;
     state.balancing = false;
     state_pub_->publish(state);
     RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
       "IMU data stale (> %.2f s) - motors commanded to zero", imu_timeout_);
     return;
   }
   ```
   Publishing **zeros** (not going silent) is deliberate: `motor_node`'s
   watchdog only fires on command *absence*, and the Gazebo effort controller
   holds the last commanded value.
6. Add `imu_timeout` handling to `on_set_params()` — note it is a **node
   member**, not a `params_` field:
   ```cpp
   } else if (name == "imu_timeout") {
     imu_timeout_ = p.as_double();
   ```
7. Add `imu_timeout: 0.2` under `balance_controller_node.ros__parameters` in
   **both** `src/sbr_bringup/config/sbr_params.yaml` and
   `src/sbr_simulation/config/sim_balance.yaml`. (The firmware already has an
   equivalent — the `imu_fail > 5` streak in `firmware/src/main.cpp` — no
   firmware change.)

**Build:** `colcon build --symlink-install --packages-select sbr_control sbr_bringup sbr_simulation`

**Verify:**
```bash
source install/setup.bash
ros2 run sbr_control balance_controller_node &
ros2 topic pub --times 100 -r 50 /imu/data sensor_msgs/msg/Imu "{orientation: {w: 1.0, y: 0.05}}"
# while that pub runs, in another shell: ros2 topic echo /wheel_cmd --once  -> nonzero efforts
# ~0.2 s after the pub finishes:
ros2 topic echo /wheel_cmd --once      # expect data: [0.0, 0.0]
ros2 topic echo /balance_state --once  # expect balancing: false
kill %1
```
Then run the headless sim ~30 s and confirm the robot still balances (Gazebo's
IMU is fresh so the watchdog must never trip):
`ros2 launch sbr_simulation simulation.launch.py headless:=true`
(watch the `BAL` log lines: pitch should stay sub-degree). Kill it afterwards.

**Commit:** `safety: IMU-freshness watchdog in balance controller (zero-effort cut on stale /imu/data)`

### Step 0.2 — Degraded-mode signalling in imu_node (keep the mock fallback)

**File:** `src/sbr_drivers/sbr_drivers/imu_node.py`

CLAUDE.md requires keeping the auto-fallback-to-mock behaviour — do **not**
remove it. Instead make it *visible* and make its output *unusable for
control*:

1. Add member `self._degraded = False`. In the `except` block where the sensor
   open fails and mock is enabled, also set `self._degraded = True`.
2. Track runtime failures: add `self._read_failures = 0` and declare an int
   parameter `max_read_failures` (default 10). In `_tick()`'s read `except`
   branch: increment `self._read_failures`; when it exceeds
   `max_read_failures`, set `self._degraded = True` and
   `self.get_logger().error(..., throttle_duration_sec=5.0)`. On a successful
   read, reset `self._read_failures = 0` and clear `_degraded` **only if** it
   was set by read failures (keep a separate `self._fallback = True` flag for
   the open-time fallback, which never clears).
3. When publishing while degraded, flag the message invalid so the controller
   (Step 0.1 item 4) ignores it:
   ```python
   if self._degraded or self._fallback:
       msg.orientation_covariance[0] = -1.0
   ```
   An **explicit** `mock:=true` (user asked for mock, e.g. on a dev machine)
   must keep covariance 0 so sim/dev flows still work — only *fallback* mock
   is flagged.
4. Add a status publisher in `__init__`:
   ```python
   from std_msgs.msg import Bool
   self._status_pub = self.create_publisher(Bool, 'imu/hardware_ok', 10)
   self._status_timer = self.create_timer(1.0, self._publish_status)
   ```
   with `_publish_status()` publishing
   `Bool(data=(self._sensor is not None and not self._degraded and not self._fallback))`.
5. While degraded/fallback, emit a throttled `error()` log in `_tick()` saying
   the IMU is degraded and the controller will cut the motors.

**Build:** `colcon build --symlink-install --packages-select sbr_drivers`

**Verify:**
```bash
source install/setup.bash
ros2 run sbr_drivers imu_node &          # no smbus2 on this machine -> auto-fallback
ros2 topic echo /imu/hardware_ok --once  # expect data: false
ros2 topic echo /imu/data --once         # orientation_covariance[0] == -1.0
kill %1
ros2 run sbr_drivers imu_node --ros-args -p mock:=true &   # explicit mock
ros2 topic echo /imu/data --once         # covariance[0] == 0.0 (valid)
kill %1
```
End-to-end: run the fallback-degraded `imu_node` together with
`balance_controller_node` → `/wheel_cmd` must settle at `[0.0, 0.0]`.

**Commit:** `safety: signal degraded IMU (hardware_ok topic + invalid-orientation flag); controller ignores flagged samples`

### Step 0.3 — Respawn on the robot launch

**File:** `src/sbr_bringup/launch/robot.launch.py`

Add `respawn=True, respawn_delay=2.0` to the `Node(...)` actions for
`imu_node`, `motor_node`, and `balance_controller_node` (robot_state_publisher
optionally too; not rviz).

**Verify:** build `sbr_bringup`; `ros2 launch sbr_bringup robot.launch.py`
(nodes run in mock fallback on this machine), then in another shell
`kill -9 $(pgrep -f imu_node)` and confirm the launch log shows a respawn
after ~2 s. Ctrl-C to exit.

**Commit:** `safety: respawn driver and controller nodes in robot.launch.py`

### Step 0.4 — WHO_AM_I check + gyro-bias calibration (Pi driver)

**Files:** `src/sbr_drivers/sbr_drivers/mpu6050.py`,
`src/sbr_drivers/sbr_drivers/imu_node.py`,
`src/sbr_bringup/config/sbr_params.yaml`

1. `mpu6050.py`: add register constant `_WHO_AM_I = 0x75`. In `__init__`,
   after waking the device:
   ```python
   whoami = self._bus.read_byte_data(self._address, _WHO_AM_I)
   if whoami != 0x68:
       raise RuntimeError(
           f'MPU6050 WHO_AM_I mismatch: got 0x{whoami:02x}, expected 0x68 '
           '(check wiring/AD0; some clones report other values)')
   ```
   A raise here lands in imu_node's existing open-`except` → degraded mock
   fallback from Step 0.2 → visible *and* safe.
2. `mpu6050.py`: rename the current `read()` body to `_read_raw()`; new
   `read()` returns `_read_raw()` with `self._gyro_bias` (init `(0.0, 0.0,
   0.0)`) subtracted from the three gyro values. Add:
   ```python
   def calibrate_gyro(self, samples=200, delay_s=0.002):
       """Average gyro output at rest. Robot MUST be stationary."""
       import time
       sx = sy = sz = 0.0
       for _ in range(samples):
           _, _, _, gx, gy, gz = self._read_raw()
           sx += gx; sy += gy; sz += gz
           time.sleep(delay_s)
       self._gyro_bias = (sx / samples, sy / samples, sz / samples)
       return self._gyro_bias
   ```
3. `imu_node.py`: declare int parameter `calibration_samples` (default 200).
   After a successful sensor open, if > 0: log "Calibrating gyro bias — keep
   the robot still...", call `self._sensor.calibrate_gyro(samples=n)`, log the
   resulting bias values.
4. Add `calibration_samples: 200` to the `imu_node` block in
   `sbr_params.yaml`.

**Verify:** build `sbr_drivers sbr_bringup`;
`python3 -m pytest src/sbr_drivers/test -v` still green. **[HW]** for a
real-sensor check; the code path gets unit tests in Step 2.3.

**Commit:** `drivers: MPU6050 WHO_AM_I check + startup gyro-bias calibration`

---

## Phase 1 — Correctness / robustness

### Step 1.1 — Measured dt instead of nominal dt

**Files:** `src/sbr_control/src/balance_controller_node.cpp`,
`src/sbr_drivers/sbr_drivers/imu_node.py` (the core's `update(..., dt)` API is
unchanged).

Node: add member `rclcpp::Time last_step_time_{0, 0, RCL_ROS_TIME};`. In
`on_timer()` replace `const double dt = 1.0 / loop_rate_;` with:
```cpp
const double nominal_dt = 1.0 / loop_rate_;
const rclcpp::Time step_now = now();
double dt = nominal_dt;
if (last_step_time_.nanoseconds() > 0) {
  dt = (step_now - last_step_time_).seconds();
  dt = std::max(0.25 * nominal_dt, std::min(dt, 4.0 * nominal_dt));  // clamp jitter
}
last_step_time_ = step_now;
```
(`now()` follows sim time under `use_sim_time`, which is what we want.)

imu_node `_tick()`: replace `dt = 1.0 / self._rate` with a node-clock delta
clamped to `[0.25/self._rate, 4.0/self._rate]`; store `self._last_tick`;
first tick uses the nominal value.

**Verify:** build `sbr_control sbr_drivers`; run the headless sim 30 s —
behaviour must be indistinguishable (BAL log pitch still ~±0.1°).

**Commit:** `control: use measured, clamped dt in controller node and imu_node`

### Step 1.2 — motor_node PWM default 1000 → 20000

**File:** `src/sbr_drivers/sbr_drivers/motor_node.py` — change
`self.declare_parameter('pwm_frequency', 1000)` to default `20000` (matches
`sbr_params.yaml` and docs/hardware.md; 1 kHz causes audible whine).

**Verify:** `ros2 run sbr_drivers motor_node --ros-args -p mock:=true` starts;
`ros2 param get /motor_node pwm_frequency` → 20000.

**Commit:** `drivers: default pwm_frequency 20000 (match docs and sbr_params.yaml)`

### Step 1.3 — ParameterValue wrapper in robot.launch.py

**File:** `src/sbr_bringup/launch/robot.launch.py` — add
`from launch_ros.parameter_descriptions import ParameterValue` and wrap:
```python
robot_description = ParameterValue(
    Command(['xacro ', xacro_file, ' use_sim:=false']), value_type=str)
```
(mirrors `simulation.launch.py` and `display.launch.py`, which already do this).

**Verify:** `ros2 launch sbr_bringup robot.launch.py` — robot_state_publisher
starts without the YAML-parse warning.

**Commit:** `bringup: wrap xacro Command in ParameterValue(value_type=str)`

### Step 1.4 — Validate rates before create_timer

- `balance_controller_node.cpp` constructor, right after reading `loop_rate_`
  (add `#include <stdexcept>`):
  ```cpp
  if (loop_rate_ <= 0.0) {
    RCLCPP_FATAL(get_logger(), "loop_rate must be > 0 (got %.3f)", loop_rate_);
    throw std::invalid_argument("loop_rate must be > 0");
  }
  ```
- `imu_node.py` after reading `publish_rate`:
  `if self._rate <= 0: raise ValueError(f'publish_rate must be > 0 (got {self._rate})')`

**Verify:** `ros2 run sbr_control balance_controller_node --ros-args -p loop_rate:=0.0`
exits with the FATAL message; same for
`ros2 run sbr_drivers imu_node --ros-args -p publish_rate:=0.0 -p mock:=true`.

**Commit:** `robustness: reject non-positive loop/publish rates at startup`

### Step 1.5 — Guard main() finally blocks

**Files:** `imu_node.py` and `motor_node.py`. Pattern for both:
```python
def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ImuNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            if getattr(node, '_sensor', None) is not None:   # imu_node
                node._sensor.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
```
(motor_node: `getattr(node, '_backend', None)` / `node._backend.close()`.)

**Verify:** the Step 1.4 zero-rate invocation now exits with only the
ValueError — no secondary NameError.

**Commit:** `drivers: guard main() cleanup when node construction fails`

---

## Phase 2 — Tests & CI

### Step 2.1 — Pid dt-guard + direct Pid unit tests

**Files:** `src/sbr_control/src/pid.cpp` (**core — pio run required**), new
`src/sbr_control/test/test_pid.cpp`, `src/sbr_control/CMakeLists.txt`.

1. In `Pid::update()`, guard the integral against non-positive dt:
   ```cpp
   double integral = integral_;
   if (dt > 0.0) {
     integral += error * dt;
   }
   ```
   then continue with the existing clamp/anti-windup logic. Document in
   `pid.hpp` that P and D terms are dt-independent by design.
2. New `test/test_pid.cpp` (same style as the existing test file, with its own
   `main()`), covering: proportional-only output; integral accumulation over
   two steps; `integral_limit` clamping; conditional-integration anti-windup
   (drive into saturation with huge kp, flip the error sign, assert the output
   leaves saturation immediately — i.e. the integral did not wind up);
   derivative-on-measurement sign (`kd=0.4, derivative=+1` → output −0.4);
   output saturation at exactly `output_limit`; `reset()` clears the integral;
   `dt=0` and `dt<0` return finite `kp*error` and leave the integral unchanged.
3. CMakeLists `BUILD_TESTING` block:
   ```cmake
   ament_add_gtest(test_pid test/test_pid.cpp)
   target_link_libraries(test_pid balance_lib)
   ```

**Verify:** colcon build + test `sbr_control` (all green);
`cd firmware && pio run` (pid.cpp is shared).

**Commit:** `control: guard Pid integral against dt<=0; add direct Pid gtests`

### Step 2.2 — Fall latch with hysteresis + expanded gtests

**Files (all in ONE commit — they are interlocked):**
`src/sbr_control/include/sbr_control/balance_controller.hpp`,
`src/sbr_control/src/balance_controller.cpp` (**core — pio run required**),
`src/sbr_control/test/test_balance_controller.cpp`,
`src/sbr_control/src/balance_controller_node.cpp`,
`src/sbr_bringup/config/sbr_params.yaml`,
`src/sbr_simulation/config/sim_balance.yaml`,
`firmware/include/sbr_config.hpp`.

Today `update()` auto-resumes the instant |pitch| < fall_threshold: a robot
teetering at the threshold chatters, and a fallen robot bounced by hand
re-arms at 44°. Add hysteresis:

1. `balance_controller.hpp` `Params`: add
   `double recover_threshold{0.4};  ///< |pitch| must drop below this to re-arm after a fall [rad]`
   and a private member `bool fallen_{false};`.
2. `balance_controller.cpp` `update()` — replace the current fall check with:
   ```cpp
   const double abs_pitch = std::fabs(pitch);
   if (!fallen_ && abs_pitch > params_.fall_threshold) {
     fallen_ = true;
   } else if (fallen_ && abs_pitch < params_.recover_threshold) {
     fallen_ = false;
     pitch_pid_.reset();
   }
   if (fallen_) {
     pitch_pid_.reset();
     return cmd;   // balancing=false, zero efforts
   }
   ```
   `reset()` becomes `{pitch_pid_.reset(); fallen_ = false;}`.
3. Node: `declare_parameter<double>("recover_threshold", 0.4)` into
   `params_.recover_threshold`; add to `on_set_params()`.
4. YAML: add `recover_threshold: 0.4` to **both** param files. Firmware:
   `p.recover_threshold = 0.4;` in `sbr_cfg::make_params()`.
5. New gtests: fall latches until recover threshold (fall at 0.7 with
   threshold 0.6/recover 0.3 → still cut at 0.5 → resumes at 0.2); `reset()`
   clears the latch; `lean_per_velocity` shifts the setpoint by exactly
   `linear_cmd * 0.08`; `pitch_offset` as setpoint (pitch==offset → ~0
   effort); kd opposes pitch rate (pitch=0, rate=+1 → left_effort > 0);
   ki integrates (constant error → |effort| grows between step 1 and step 50).

**Verify:** colcon build + test `sbr_control`; `cd firmware && pio run`;
30 s headless sim sanity run.

**Commit:** `control: latch tip-kill with recover_threshold hysteresis; expand BalanceController gtests`

### Step 2.3 — Behavioral tests for the Python drivers

**Files:** new `src/sbr_drivers/sbr_drivers/filters.py`, edits to
`imu_node.py`, new tests `src/sbr_drivers/test/test_filters.py`,
`test_mpu6050_behavior.py`, `test_motor_node.py`.

1. Extract the complementary filter so it is testable without rclpy —
   `filters.py`:
   ```python
   import math

   def complementary_update(roll, pitch, ax, ay, az, gx, gy, alpha, dt):
       acc_roll = math.atan2(ay, az)
       acc_pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az))
       roll = alpha * (roll + gx * dt) + (1.0 - alpha) * acc_roll
       pitch = alpha * (pitch + gy * dt) + (1.0 - alpha) * acc_pitch
       return roll, pitch
   ```
   Move `quaternion_from_euler` here too; `imu_node.py` imports both. **The
   math must remain literally identical** to `firmware/src/mpu6050.hpp` (the
   fusion line) — this is the cross-target consistency guarantee.
2. `test_filters.py`: quaternion vs known values (pitch=π/2 → y=w=√2/2);
   filter convergence (start pitch 0, feed the gravity vector of a 10° tilt —
   `ax=-9.80665*sin(10°), az=9.80665*cos(10°)`, zero gyro — for 500 steps at
   alpha=0.98, dt=0.01 → pitch within 0.5° of 10°); single-step gyro
   integration (gy=1.0 rad/s, level accel, one step → pitch ≈ alpha*dt).
3. `test_mpu6050_behavior.py`: inject a fake `smbus2` via
   `sys.modules['smbus2']` (a `types.ModuleType` with an `SMBus` class that
   records writes and serves canned register reads: WHO_AM_I → 0x68, 14-byte
   block = struct-packed known counts). Assert: scaling (16384 counts →
   9.80665 m/s²; 131 counts → 1 °/s in rad/s), WHO_AM_I mismatch raises
   RuntimeError, `calibrate_gyro` bias is subtracted in subsequent `read()`.
4. `test_motor_node.py` (rclpy, no spin): `rclpy.init()`, construct
   `MotorNode()` with mock enabled (parameter overrides), replace
   `node._backend` with a recording fake exposing `set/stop/close`. Assert:
   `_on_cmd([0.5,-0.5])` → `fake.set(0.5, -0.5)`; a 1-element array → no call;
   forcing the last-command time 1 s into the past and running the watchdog
   callback → `fake.stop()` called. Teardown `destroy_node()` + `shutdown()`.

**Verify:** `colcon build --symlink-install --packages-select sbr_drivers &&
python3 -m pytest src/sbr_drivers/test -v` — all new and old tests pass.

**Commit:** `drivers: extract complementary filter; add behavioral tests (filter, MPU6050 scaling/whoami/bias, motor watchdog)`

### Step 2.4 — Gain-sync guard between firmware and hardware params

**File:** new `src/sbr_bringup/test/test_gain_sync.py`

A pytest that parses `firmware/include/sbr_config.hpp` with regexes and
asserts value equality against `src/sbr_bringup/config/sbr_params.yaml`
(hardware file only — the sim file intentionally differs). Structure:

- Locate files relative to the test file; `pytest.mark.skipif` when the
  firmware dir is absent (installed-space runs).
- Helper `_member(text, path)` matching lines like
  `p.pitch_gains.kp = 6.0;` (regex
  `r'p\.' + re.escape(path) + r'\s*=\s*([-\d.eE]+)\s*;'`) and
  `_constexpr(text, name)` matching
  `r'constexpr\s+[\w:]+\s+' + name + r'\s*=\s*([-\d.eE]+)'`.
- Assert with `pytest.approx`: `pitch_kp↔pitch_gains.kp`,
  `pitch_ki↔pitch_gains.ki`, `pitch_kd↔pitch_gains.kd`, `integral_limit`,
  `output_limit`, `pitch_offset`, `output_scale`, `fall_threshold`,
  `recover_threshold`, `lean_per_velocity`, `steer_gain`; constexpr pairs
  `kLoopRateHz↔loop_rate`, `kCmdTimeoutS↔cmd_timeout`,
  `kComplementaryAlpha↔imu_node.complementary_alpha`,
  `kPwmFreqHz↔motor_node.pwm_frequency`, `kMaxDuty↔max_duty`,
  `kDeadband↔deadband`.

**Rule from here on:** any commit changing a shared value in either file must
change both **and** this test's pair list in the same commit.

**Verify:** `python3 -m pytest src/sbr_bringup/test -v` — must pass against
current values (they match today).

**Commit:** `test: gain-sync guard between firmware sbr_config.hpp and sbr_params.yaml`

### Step 2.5 — CI: build sbr_simulation + Python lint

**File:** `.github/workflows/ci.yml`

1. Add `sbr_simulation` to the `--packages-select` build list (and ensure the
   rosdep step covers `src/sbr_simulation`). This pulls ros_gz /
   gz_ros2_control — heavier, but launch/config breakage becomes visible.
2. Add a lint step after the Python tests:
   ```yaml
   - name: Lint Python
     shell: bash
     run: |
       apt-get update && apt-get install -y python3-flake8
       python3 -m flake8 --max-line-length=99 \
         src/sbr_drivers/sbr_drivers src/sbr_bringup src/sbr_simulation/launch \
         src/sbr_drivers/test src/sbr_bringup/test
   ```
   **Run the same flake8 command locally first** and fix any existing
   violations in the same commit.
3. Update the now-stale "core packages only" comment in the workflow.

**Verify:** local flake8 clean; push; `gh run watch` (or
`gh run list --limit 1`) until green.

**Commit:** `ci: build sbr_simulation and lint Python`

### Step 2.6 — CI: firmware build job

**File:** `.github/workflows/ci.yml` — add a parallel job:
```yaml
  firmware:
    name: PlatformIO build (ESP32-S3)
    runs-on: ubuntu-24.04
    steps:
      - uses: actions/checkout@v4
      - uses: actions/setup-python@v5
        with: {python-version: '3.11'}
      - uses: actions/cache@v4
        with:
          path: |
            ~/.platformio
            firmware/.pio
          key: pio-${{ hashFiles('firmware/platformio.ini') }}
      - name: Build firmware
        run: |
          pip install platformio
          cd firmware && pio run
```
First run builds the micro-ROS library (~10–15 min); the cache makes later
runs fast. The in-repo relative symlinks under `firmware/lib/sbr_control`
survive checkout. Also update the stale "no ESP toolchain in CI" comments in
`platformio.ini` and the workflow.

**Verify:** push; `gh run watch` until both jobs green.

**Commit:** `ci: add PlatformIO firmware build job`

---

## Phase 3 — Velocity cascade (fixes the sim runaway now; encoder-ready later)

### Step 3.1 — The cascade (ONE commit: msg + core + node + params + firmware + tests)

This must be a single commit because the message, core signature, node, both
YAML files, firmware `make_params()`, gtests, and the gain-sync test are
interlocked.

**Why:** with no velocity feedback the robot settles into a tiny lean and
accelerates forever (velocity is unobservable from pitch). A gated outer loop
turns measured wheel velocity into a small corrective lean. In sim the
velocity comes free from `/joint_states` (joint_state_broadcaster publishes
wheel joint velocities); on hardware the gains stay **0.0** until encoders
exist (Phase 4), so the encoder-less robot's behaviour is unchanged.

**A. Message** — `src/sbr_msgs/msg/BalanceState.msg`, append:
```
# Measured mean wheel ground-speed [m/s] (sim: joint_state_broadcaster; hardware:
# encoders when fitted; 0.0 and velocity_valid=false when unavailable).
float64 wheel_velocity
bool velocity_valid
```

**B. Core** — `balance_controller.hpp` / `.cpp` (ROS-free, C++17 only):
- `Params` additions:
  ```cpp
  double velocity_kp{0.0};             ///< outer-loop lean [rad] per (m/s) velocity error
  double velocity_ki{0.0};             ///< outer-loop integral gain [rad/m]
  double velocity_integral_limit{0.5}; ///< clamp on the velocity integral [m]
  double max_lean{0.15};               ///< clamp on the outer-loop lean output [rad]
  ```
- New private member `Pid velocity_pid_{};`, configured in `set_params()`:
  ```cpp
  Pid::Gains vg;
  vg.kp = params_.velocity_kp;
  vg.ki = params_.velocity_ki;
  vg.kd = 0.0;
  vg.integral_limit = params_.velocity_integral_limit;
  vg.output_limit = params_.max_lean;
  velocity_pid_.set_gains(vg);
  ```
- `update()` gains two **defaulted** trailing parameters so every existing
  caller (gtests, firmware main.cpp) compiles unchanged:
  ```cpp
  Command update(double pitch, double pitch_rate,
                 double linear_cmd, double angular_cmd, double dt,
                 double measured_velocity = 0.0, bool velocity_valid = false);
  ```
- Inside `update()`, replace the setpoint computation:
  ```cpp
  double setpoint = params_.pitch_offset + linear_cmd * params_.lean_per_velocity;
  const bool outer_active = velocity_valid &&
    (params_.velocity_kp != 0.0 || params_.velocity_ki != 0.0);
  if (outer_active) {
    // Cascade: drifting forward (measured > commanded) -> negative lean
    // (lean back) so the inner loop decelerates. Standard PID sign does this.
    setpoint += velocity_pid_.update(linear_cmd, measured_velocity, 0.0, dt);
  } else {
    velocity_pid_.reset();   // never integrate on stale/absent measurements
  }
  cmd.pitch_setpoint = setpoint;
  ```
- The fall latch (Step 2.2) and `reset()` must also reset `velocity_pid_`.
- Update the class doc comment about "no closed-loop velocity" to describe the
  gated cascade.

**C. Node** — `balance_controller_node.cpp`:
- New params (first four also live-tunable in `on_set_params()`):
  `velocity_kp` (0.0), `velocity_ki` (0.0), `velocity_integral_limit` (0.5),
  `max_lean` (0.15), `wheel_radius` (0.045 — matches `wheel_radius` in
  `sbr.urdf.xacro`), `velocity_timeout` (0.3), `joint_states_topic`
  ("/joint_states"), `left_wheel_joint` ("left_wheel_joint"),
  `right_wheel_joint` ("right_wheel_joint").
- Subscribe `sensor_msgs::msg::JointState`; in the callback find both joint
  names in `msg->name`, require `msg->velocity.size() == msg->name.size()`,
  then:
  ```cpp
  wheel_velocity_ = wheel_radius_ * 0.5 * (v_left + v_right);  // rad/s -> m/s
  last_joint_state_time_ = now();
  ```
- In `on_timer()`:
  ```cpp
  const bool vel_valid =
    (now() - last_joint_state_time_).seconds() < velocity_timeout_;
  const auto cmd = controller_.update(
    pitch_, pitch_rate_, linear_cmd_, angular_cmd_, dt, wheel_velocity_, vel_valid);
  ```
- Fill `state.wheel_velocity` / `state.velocity_valid` in the telemetry.

**D. Params:**
- `src/sbr_simulation/config/sim_balance.yaml` — add:
  ```yaml
      velocity_kp: 0.05          # rad of lean per m/s velocity error
      velocity_ki: 0.02
      velocity_integral_limit: 0.5
      max_lean: 0.15
      wheel_radius: 0.045
      velocity_timeout: 0.3
  ```
- `src/sbr_bringup/config/sbr_params.yaml` — same keys but
  `velocity_kp: 0.0`, `velocity_ki: 0.0` (**encoder-less hardware = outer loop
  off**).

**E. Firmware:**
- `firmware/include/sbr_config.hpp` `make_params()` — add
  `p.velocity_kp = 0.0; p.velocity_ki = 0.0; p.velocity_integral_limit = 0.5;
  p.max_lean = 0.15;` with a comment pointing to Phase 4 / `kHasEncoders`.
- `firmware/src/main.cpp` `publish_state()` — set
  `g_state_msg.wheel_velocity = 0.0; g_state_msg.velocity_valid = false;`.

**F. Gtests** (additions to `test_balance_controller.cpp`):
- Zero velocity gains + valid measurement → efforts identical to a legacy
  call (backwards compatibility).
- `velocity_kp=0.1`, zero cmd, measured +0.5 m/s valid → `pitch_setpoint < 0`
  (leans back against drift).
- Outer-loop output clamped by `max_lean` (huge kp, huge measured velocity).
- 100 updates with `velocity_valid=false` and `velocity_ki` nonzero, then one
  valid update → integral contribution starts from zero (compare to a fresh
  controller).

**G. Sync test** — extend `test_gain_sync.py` pairs with `velocity_kp`,
`velocity_ki`, `velocity_integral_limit`, `max_lean`.

**H. Docs** — `docs/control_tuning.md`: add an "Outer velocity loop" section
(sign convention, why hardware keeps gains 0 until encoders exist, tuning
recipe: raise `velocity_kp` from ~0.02 until drift dies without pitch
oscillation, then a small `velocity_ki` for steady-state creep).

**Build:**
```bash
colcon build --symlink-install --packages-select sbr_msgs sbr_control sbr_simulation sbr_bringup
colcon test --packages-select sbr_control --return-code-on-test-failure
python3 -m pytest src/sbr_bringup/test -v
cd firmware && pio run -t clean_microros && pio run && cd ..
```
The `clean_microros` is **required**: the message definition changed and the
cached micro-ROS library holds the old `sbr_msgs` typesupport.

**Verify (the acceptance criterion for the feature):**
```bash
ros2 launch sbr_simulation simulation.launch.py headless:=true &
sleep 25
ros2 topic echo /balance_state --once           # velocity_valid: true
timeout 60 ros2 topic echo /balance_state --field pos_x
```
Accept: |pos_x| stays **< 0.5 m over 60 s** with no cmd_vel (previously it
drifted unboundedly at ~1.5 m/s), pitch stays sub-degree RMS, no saturation
chatter. If the robot accelerates *harder* instead, the outer-loop sign is
wrong for the plant — negate the term (`setpoint -= ...`) — but the design
sign (drift forward → lean back) is correct for a wheel-velocity measurement.
If it oscillates slowly (~1 Hz), halve `velocity_kp`. Also verify driving
still works:
`ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"`
→ robot advances, stays up, and stops (holds position) after the pub is
killed. Kill all sim processes afterwards.

**Commit:** `feature: velocity cascade outer loop (gated by gains; sim uses /joint_states; hardware stays open-loop until encoders)`

### Step 3.2 — Re-tune sim gains objectively (recommended)

Method (this is how the pitch loop was tuned; it works): for each candidate
`velocity_kp`/`velocity_ki` pair, launch a **fresh** headless sim, wait for
balance, apply a repeatable disturbance:
```bash
gz service -s /world/empty/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean \
  --timeout 3000 --req 'name: "sbr", position: {x: 0, y: 0, z: 0.05}, orientation: {x: 0, y: 0.05234, z: 0, w: 0.99863}'
```
(that is a 6° pitch tilt), record `/balance_state` for 15–20 s, compute RMS
pitch, peak pitch, effort-saturation % (|effort| ≥ 0.99 × output_scale 0.4),
and pos_x drift. One sim per candidate — teleporting between trials leaves
residual velocity and corrupts Gazebo's odometry integrator. Kill each sim
fully before the next. Pick the candidate minimizing
`RMS + 10*drift + 0.05*sat%`; write the winner into `sim_balance.yaml`.
(The Step 7.1 recorder tool can be built first and reused here.)

**Commit:** `sim: tune velocity loop gains (objective harness: RMS/peak/saturation/drift)`

---

## Phase 4 — Encoder support path (firmware) **[HW for runtime]**

All code compiles and commits now; runtime validation waits for the ESP32-S3
board + encoder-equipped TT motors (quadrature, rear-shaft Hall type — plain
TT motors have no rear shaft; single-channel optical LM393 discs give no
direction and are unsuitable).

### Step 4.1 — Firmware encoder module + config flags (build-only here)

**Files:** `firmware/platformio.ini`, `firmware/include/sbr_config.hpp`, new
`firmware/src/encoders.hpp`, `firmware/src/main.cpp`.

1. `platformio.ini` `lib_deps` — add `madhephaestus/ESP32Encoder@^0.11.7`
   (wraps the ESP32 PCNT hardware pulse counter; zero CPU cost, S3-compatible).
2. `sbr_config.hpp` — new block:
   ```cpp
   // ---- Wheel encoders (quadrature TT motors) --------------------------------
   constexpr bool   kHasEncoders   = false;  // flip true once encoder motors are fitted
   constexpr int    kEncLeftA  = 4;          // VERIFY against your S3 board's free GPIOs
   constexpr int    kEncLeftB  = 5;
   constexpr int    kEncRightA = 6;
   constexpr int    kEncRightB = 7;
   constexpr double kCountsPerRev = 1920.0;  // quadrature counts per WHEEL revolution
   constexpr double kWheelRadiusM = 0.045;   // must match sbr.urdf.xacro wheel_radius
   constexpr bool   kInvertEncLeft  = false;
   constexpr bool   kInvertEncRight = false;
   ```
   In `make_params()`, keep velocity gains at 0.0 with a comment: set them
   when `kHasEncoders` goes true (start from the tuned sim values).
3. New `encoders.hpp` — thin wrapper:
   ```cpp
   #pragma once
   #include <ESP32Encoder.h>
   #include "sbr_config.hpp"

   class WheelEncoders {
   public:
     void begin();                     // attachFullQuad on the 4 pins; clearCount
     double velocity_mps(double dt);   // mean wheel ground speed, low-pass filtered
   private:
     ESP32Encoder left_, right_;
     int64_t last_left_{0}, last_right_{0};
     double v_filt_{0.0};
   };
   ```
   `velocity_mps`: take count deltas (negate per `kInvertEnc*`), convert
   `(delta / kCountsPerRev) * 2π * kWheelRadiusM / dt`, average both wheels,
   then a 1-pole low-pass `v_filt_ = 0.7 * v_filt_ + 0.3 * v_raw` (counts are
   quantized at 200 Hz).
4. `main.cpp` `control_task`: `static WheelEncoders g_encoders;`; call
   `g_encoders.begin()` after `g_motors.begin()` when `sbr_cfg::kHasEncoders`;
   each loop compute `wheel_v` (0.0 when disabled) and call:
   ```cpp
   g_controller.update(pitch, pitch_rate, lin, ang, dt,
                       wheel_v, sbr_cfg::kHasEncoders);
   ```
   Add the velocity to `TeleSnap` and `publish_state()`.
5. `docs/hardware.md` — new "Wheel encoders" section: encoder pinout (3V3,
   GND, A, B per side), chosen GPIOs, PCNT does the counting in hardware, the
   `kHasEncoders` flip procedure, and the bring-up checklist: flash → spin
   each wheel by hand with motors off → check `/balance_state.wheel_velocity`
   sign (forward hand-spin must read positive) → fix `kInvertEnc*` → set
   `kHasEncoders=true` + copy tuned velocity gains into `make_params()` **and**
   the same values into `sbr_params.yaml` (the gain-sync test enforces this)
   → re-flash.

**Verify:** `cd firmware && pio run` **twice** — once as-is
(`kHasEncoders=false`) and once with it temporarily flipped `true` (compile
check only; flip back before committing). RAM/Flash deltas < +2%.

**Commit:** `firmware: wheel-encoder support (ESP32Encoder/PCNT) behind kHasEncoders, off by default`

---

## Phase 5 — State-estimation improvements

### Step 5.1 — Firmware gyro-bias calibration at boot (parity with Step 0.4)

**Files:** `firmware/src/mpu6050.hpp`, `firmware/src/main.cpp`,
`firmware/include/sbr_config.hpp` (+ `test_gain_sync.py` pair).

1. `mpu6050.hpp`: add member `double gy_bias_ = 0.0;` and:
   ```cpp
   bool calibrateGyro(int samples)   // call with the robot stationary
   {
     double sum = 0.0; int ok = 0;
     for (int i = 0; i < samples; ++i) {
       double ax, ay, az, gx, gy, gz;
       if (readAll(ax, ay, az, gx, gy, gz)) { sum += gy; ++ok; }
       delay(2);
     }
     if (ok < samples / 2) { return false; }
     gy_bias_ = sum / ok;
     return true;
   }
   ```
   In `update()`, subtract: `gy -= gy_bias_;` **before** the fusion line, so
   the fusion math itself stays byte-identical to the Python path (where
   `read()` returns bias-subtracted rates after Step 0.4).
2. `sbr_config.hpp`: `constexpr int kGyroCalibSamples = 200;` (mirror of
   imu_node's `calibration_samples`; add the pair to `test_gain_sync.py`).
3. `main.cpp` `control_task`: after the `g_imu.begin(...)` retry loop
   succeeds: `g_imu.calibrateGyro(sbr_cfg::kGyroCalibSamples);` (~0.4 s; the
   motors are still idle at this point).

**Verify:** `cd firmware && pio run` clean;
`python3 -m pytest src/sbr_bringup/test -v` (sync test). **[HW]** for the
behavioral check (pitch should not drift with the robot lying still).

**Commit:** `firmware: gyro-bias calibration at boot (parity with imu_node)`

### Step 5.2 — Seed the Pi filter from gravity at startup

**File:** `src/sbr_drivers/sbr_drivers/imu_node.py` — after a successful
sensor open (and after calibration), do one `read()` and set the initial
pitch/roll from the accel-only formulas (the same two `atan2` lines as the
filter), mirroring what `firmware/src/mpu6050.hpp` `begin()` already does.
This avoids the first-second transient where the filter chases the true angle
from 0.

**Deliberate non-goal:** do NOT replace the complementary filter with
Mahony/Madgwick/EKF. The filter is shared verbatim across three targets;
changing it triples the regression surface for marginal gain at this loop
rate. Note this as a non-goal in `docs/control_tuning.md`.

**Verify:** build sbr_drivers + pytest green.

**Commit:** `drivers: seed complementary filter from gravity at startup (firmware parity)`

---

## Phase 6 — Automated sim regression test

### Step 6.1 — Pytest smoke test, env-gated

**File:** new `src/sbr_simulation/test/test_sim_balance.py`. The package is
ament_cmake; the test is run by **explicit pytest invocation only** (not wired
into colcon), so normal builds stay fast. Gate it:
```python
pytestmark = pytest.mark.skipif(
    os.environ.get('SBR_SIM_TEST') != '1', reason='set SBR_SIM_TEST=1 to run')
```
Test body: launch `ros2 launch sbr_simulation simulation.launch.py
headless:=true` via `subprocess.Popen(..., preexec_fn=os.setsid)`; subscribe
to `/balance_state` with rclpy; wait up to 120 s for the first message; then
record 20 s of samples and assert:
- more than 500 samples arrived,
- peak |pitch| < 0.3 rad (never close to falling),
- RMS pitch < 0.05 rad,
- final `balancing == true`,
- `max(pos_x) - min(pos_x) < 0.5` m (velocity loop holds station — this
  catches a Phase 3 regression).

Teardown in `finally`: `os.killpg(os.getpgid(proc.pid), signal.SIGINT)`, wait
30 s, then SIGKILL the group.

Optional second test: apply the 6° `gz service set_pose` kick (exact command
in Step 3.2) once balanced, then assert |pitch| < 0.05 rad continuously during
seconds 5–10 after the kick (disturbance recovery).

**Verify:**
```bash
source install/setup.bash
SBR_SIM_TEST=1 python3 -m pytest src/sbr_simulation/test -v   # 1-2 passes, ~1-3 min
python3 -m pytest src/sbr_simulation/test -v                  # skipped
```

**Commit:** `test: headless Gazebo balance regression (SBR_SIM_TEST=1 gated)`

### Step 6.2 — CI job (non-blocking first)

**File:** `.github/workflows/ci.yml` — add a `sim-regression` job modelled on
the existing build job (same `ubuntu:noble` container + setup-ros + rosdep +
full `colcon build`), then:
```yaml
      - name: Run sim smoke test
        shell: bash
        run: |
          source /opt/ros/jazzy/setup.bash
          source install/setup.bash
          SBR_SIM_TEST=1 python3 -m pytest src/sbr_simulation/test -v
```
Mark the job `continue-on-error: true` with a comment: the server-only Gazebo
run needs no GPU/X for an IMU-only robot, but container software-rendering
quirks are possible — the job is observational until it has 3+ green runs,
then a follow-up commit removes `continue-on-error` (or moves it to a nightly
`schedule:` trigger if it proves flaky).

**Verify:** push; `gh run watch`; inspect the sim-regression job log.

**Commit:** `ci: optional Gazebo balance smoke-test job`

---

## Phase 7 — Telemetry & tuning UX

### Step 7.1 — Recorder/metrics tool

**File:** new `tools/tuning/record_balance.py` (plain script, documented
`python3` invocation — not a ROS package).

argparse: `--duration` (s, default 20), `--csv` (optional output path),
`--saturation-limit` (default 0.4 — the sim `output_scale`). Subscribes
`/balance_state` (rclpy), on completion prints one summary line:
`samples, duration, rms_pitch_deg, peak_pitch_deg, saturation_pct, pos_drift_m,
mean_wheel_velocity` — and optionally writes CSV columns
`t,pitch,pitch_rate,pitch_setpoint,left_effort,right_effort,pos_x,wheel_velocity,balancing`.
Saturation % = fraction of samples with |left_effort| ≥ 0.99 × limit.

**Verify:** with the headless sim running:
`python3 tools/tuning/record_balance.py --duration 10` → metrics print;
RMS should be in the same ballpark as the documented baseline (~0.07° before
the velocity loop; record the new post-cascade baseline in
`docs/control_tuning.md`).

### Step 7.2 — Live-tuning docs (+ optional PlotJuggler layout)

**Files:** `docs/control_tuning.md` (append a "Live tuning workflow" section),
optionally `tools/plotjuggler/balance_layout.xml`.

Document: the full list of live-tunable params
(`ros2 param set /balance_controller_node pitch_kp 7.0` — pitch_kp/ki/kd,
integral_limit, output_limit, pitch_offset, output_scale, fall_threshold,
recover_threshold, lean_per_velocity, steer_gain, imu_timeout, velocity_kp,
velocity_ki, velocity_integral_limit, max_lean, log_state); the 6° disturbance
one-liner (Step 3.2); the recorder tool; and the sync rule: "when a hardware
value is finalized, write it to `sbr_params.yaml` AND
`firmware/include/sbr_config.hpp` in one commit — `test_gain_sync.py` enforces
this." PlotJuggler layout: commit an XML plotting `/balance_state/pitch`,
`pitch_setpoint`, `left_effort`, `wheel_velocity` if straightforward;
otherwise document the 30-second manual setup instead — do not block on it.

**Verify:** recorder works against a live sim; docs read correctly.

**Commit (both steps together):** `tooling: balance telemetry recorder + live tuning workflow docs`

---

## Phase 8 — Docs & hygiene (last)

### Step 8.1 — README, LICENSE, CLAUDE.md, architecture refresh

- `README.md`: remove "(planned)" from the firmware line in the repo layout;
  update the roadmap — check off `Add D and I terms`, `ESP32-S3 micro-ROS
  firmware`, `Tip-kill + low-battery safety wired into firmware`, `Live PID
  tuning`; add unchecked items `Encoder motors + hardware velocity-loop
  bring-up` and `Make the sim regression CI job required`. Add a short
  "Velocity control" paragraph (sim closed-loop via /joint_states; hardware
  gated on encoders).
- `LICENSE`: `Copyright (c) 2026 Lewis` → `Copyright (c) 2026 Lewis Fowler`.
- `CLAUDE.md`: replace the stale "no ROS 2 toolchain in this sandbox"
  paragraph with "toolchain availability varies — check `which colcon` /
  `which pio`; CI builds all packages including sbr_simulation and the
  firmware"; document the new safety params (`imu_timeout`,
  `recover_threshold`), the degraded-IMU convention (`imu/hardware_ok` +
  negative orientation covariance), the velocity-cascade gating rule (hardware
  gains stay 0 until encoders), and that the three-way param sync is now
  enforced by `src/sbr_bringup/test/test_gain_sync.py`.
- `docs/architecture.md`: one paragraph on the outer loop and the safety
  chain: IMU invalid → controller ignores sample → freshness watchdog →
  zero-effort publish → motor watchdog as last resort.

**Verify:** run the full green gate one final time (build all, sbr_control
tests, all pytest, `pio run`), push, and confirm `gh run list --limit 1` is
green.

**Commit:** `docs: refresh README/CLAUDE/architecture for firmware, safety chain and velocity loop; fix LICENSE name`

---

## Cross-cutting rules (repeated because they bite)

- **Sync-locked commits:** Steps 2.2 (`recover_threshold`), 3.1 (velocity
  params), and 5.1 (`kGyroCalibSamples`) each touch `sbr_params.yaml` +
  `sim_balance.yaml` + `firmware/include/sbr_config.hpp` +
  `test_gain_sync.py` **in the same commit**, or the sync test goes red.
- **Core-touching commits** (2.1, 2.2, 3.1) must pass both `colcon test` and
  `cd firmware && pio run` before committing.
- **Message changes** (3.1) require `pio run -t clean_microros && pio run` —
  the cached micro-ROS library holds stale typesupport otherwise.
- **Hardware-blocked runtime validation:** 0.4 (real sensor), 4.1's bring-up
  checklist, 5.1's drift check. Everything else completes on this machine.
- After sim experiments, always
  `pkill -9 -f "gz sim|balance_controller_node|parameter_bridge|robot_state_publisher|ros2 launch sbr"`.
