# Security & pre-flight checklist

This is a hobby robot in a public repo. The practical threat model: **don't leak
secrets, don't let an untrusted network drive the robot, and don't let the robot
hurt anyone (or itself) during bring-up.**

## Software / infosec

### Secrets
- **No credentials are committed, and none should be.** Don't put Wi-Fi
  passwords, tokens, or API keys in tracked files.
- For firmware Wi-Fi credentials, use a local **git-ignored** header
  (e.g. `firmware/include/secrets.hpp`) — never commit it.
- Commits use the GitHub no-reply identity (history was scrubbed of the personal
  email). Keep it that way, and enable on GitHub:
  **Settings → Emails → "Keep my email addresses private"** and
  **"Block command line pushes that expose my email."**

### micro-ROS transport (read before going wireless)
- The firmware defaults to the **USB-serial** micro-ROS transport — it needs
  physical access, so it's low risk.
- **The Wi-Fi (UDP) transport is unauthenticated and unencrypted.** Anyone who
  can reach the board can publish `/cmd_vel` and drive it, or impersonate the
  micro-ROS agent. If you switch to Wi-Fi:
  - put the robot + agent on an **isolated AP / VLAN**, not your main LAN;
  - don't expose the agent's UDP port to untrusted networks;
  - remember the tip-kill / watchdog are *safety* features, not access control.

### Supply chain / reproducibility
These are intentionally **not** changed in-repo yet because pinning needs commit
SHAs verified against GitHub, which can't be fetched from the CI sandbox. Do them
from a machine with network access:
- **CI actions** are pinned to major tags (`actions/checkout@v4`,
  `ros-tooling/setup-ros@v0.7`). Tags can be moved; for strict reproducibility
  pin to a SHA: `uses: actions/checkout@<40-char-sha>  # v4.x`.
- **`micro_ros_platformio`** is pulled from repo head in
  `firmware/platformio.ini`. Pin it for reproducible firmware:
  `https://github.com/micro-ROS/micro_ros_platformio#<commit-sha>`.
- The PlatformIO platform is already pinned (`espressif32@^6.6.0`).

### CI
- The workflow runs with a read-only token and **no repository secrets**, so a
  malicious pull request can't exfiltrate anything. Keep secrets out of CI.

## Pre-flight & safety checklist

Balancing robots flail and pinch. Treat the first power-on like a small power tool.

### 1. Prove it in simulation first
- [ ] `make sim` launches and the robot stands in Gazebo.
- [ ] `ros2 topic echo /imu/data` shows a **changing `orientation`** (not identity
      `0,0,0,1`) — if it's static, the IMU isn't bridging and nothing else matters.
- [ ] `teleop_twist_keyboard` leans/drives it the expected way.
- [ ] If it drives *into* its fall, set `invert_pitch:=true` (the sim value is
      **independent** of the hardware one — different IMU math).

### 2. Hardware bring-up (in order)
- [ ] **Wheels off the ground** (robot on a stand/box) for every first test.
- [ ] Power from a **current-limited bench supply** first, if you have one.
- [ ] **100 µF cap** across the TB6612 `VM`/`GND` (prevents motor-inrush brown-outs).
- [ ] **Low-battery `LBO`** wired with its pull-up — or set
      `kHasLowBatteryPin = false` in `firmware/include/sbr_config.hpp`.
- [ ] GPIO map in `sbr_config.hpp` matches **your exact ESP32-S3 board**
      (watch the strap pins — see `docs/hardware.md`).

### 3. First firmware run (still on the stand)
- [ ] Flash, open the serial monitor, confirm a clean boot (no reset loop).
- [ ] **Tip-kill test:** hold it upright (motors engage), then tilt past ~45° —
      the motors **must cut**. If not, stop and check `fall_threshold` / pitch sign.
- [ ] **Direction test:** tiny `/cmd_vel`, confirm each wheel spins the right way;
      flip `kInvertLeft` / `kInvertRight` if not.
- [ ] **Pitch sign:** at low gain, confirm it corrects *toward* upright; set
      `kInvertPitch` if it accelerates its own fall. Raise gains only after this.
- [ ] **Comms-independence:** pull the USB / kill the agent → it should **keep
      balancing**. Velocity just falls back to "hold".

### 4. Only then
- [ ] Off the stand, on a soft, clear floor, hands clear of the wheels.
- [ ] Tune gains incrementally — see `docs/control_tuning.md`.
- [ ] Keep `firmware/include/sbr_config.hpp` gains in sync with
      `sbr_bringup/config/sbr_params.yaml` (the one manual sync point).
