# Build log

A diary of what didn't work the first time, why, and how each fix nudged the
project closer to a free-standing two-wheeled robot. Condensed from the original
bring-up notes (v0.5 / v0.6).

> Why a build log? Half of this project was electronics and power debugging, not
> control theory. Writing it down stopped me repeating the same mistakes — and
> might save you a couple of hours.

## Stage 1 — Minimal power test (PowerBoost 1000C)

**Goal:** confirm a single Li-ion can deliver a stable 5 V before adding any load.

- **EN chicken-and-egg.** Tied EN to VOUT ("that's the 5 V rail, right?") — but
  VOUT doesn't exist until EN is already HIGH. Result: blue LED blinked once and
  died, on repeat. **Fix:** pull EN HIGH from the **battery** rail through the
  rocker switch, with a 10 kΩ pull-down so it stays off by default.
- **Phantom low-battery (LBO) LED.** The red LBO lit at 3.9 V, well above the
  ~3.2 V threshold. Cause: **flux residue** bridging the BAT+ and LBO sense pad.
  **Fix:** scrub with IPA + a toothbrush. Lesson: clean every joint carrying
  µA-level signals.
- **Stuck JST.** The battery connector latched under a PCB cut-out; freed it with
  fine tweezers, later switched to a right-angle header.

**Result:** 5.18 V at 200 mA, rail drop < 80 mV. ✅

## Stage 2 — Motor driver + one motor

**Goal:** prove the boost survives a motor stall.

- **Why TB6612FNG over L298N / MX1508:** ~0.5 V drop vs ~2 V (more torque, less
  heat), native 3.3 V logic, and thermal protection. The MX1508 also throws
  nasty spikes on the power line when reversing direction.
- **Inrush brown-out.** With no bulk cap, free-spinning motors were fine, but
  loading the shaft flickered the rail and reset the OLED. **Fix:** 100 µF
  low-ESR cap across VM/GND.
- **Wrong wheel direction.** Fixed in software by swapping IN1/IN2 logic instead
  of re-soldering.

## Stage 3 — ESP32 power & flashing

- **90% of "ESP32 not detected" is a bad USB cable** — two of three micro-USB
  cables were power-only. The third enumerated as a CP2102 bridge immediately.
- Verified VIN = 5.18 V from the PowerBoost; ~80 mA idle. Flash at 921 600 baud.

## Stage 4 — OLED + dual I2C

- Display stayed blank until `Wire.begin()` was called **before**
  `display.begin()`.
- Ran the **MPU-6050 and SSD1306 on separate I2C buses** so fast IMU sampling
  (~100 Hz) isn't blocked by the slower OLED refresh (~10 Hz). Bus utilisation
  stayed under a couple of percent at 400 kHz.

## Stage 5 — IMU, P-only control, silent PWM

- **Pick one axis.** The robot only cares about tilt in the driving plane —
  settled on **pitch** (Y forward), sensor mounted flat.
- **Don't auto-zero at boot** unless you're sure it's upright, or the controller
  chases a phantom error. Manual calibration is the plan.
- **Kill the motor whine.** Default 1 kHz PWM screeched; moving to **20 kHz**
  (ESP32 LEDC) made it silent.
- **First P-controller:** `control = constrain(Kp * pitch, -255, 255)`. `Kp ≈ 5`
  gave a gentle response with no overshoot; `Kp ≈ 15` was aggressive.

## Stage 6 — final wiring + safety (v0.6)

- Settled GPIO map (see [hardware.md](hardware.md)); kept PWM off strap/input-only
  pins so the board always boots.
- **Low-battery fail-safe:** LBO → GPIO with a 10 kΩ pull-up; on LOW, cut the
  motors (STBY LOW) and warn on the OLED.
- Added a 60 mm 5 V fan across VOUT for stall testing (~110 mA — negligible).

## Big lessons so far

1. **Debug the power tree first** — digital "bugs" are often power problems.
2. **Run an I2C scanner** every time you add a device.
3. **Separate noisy loads (motors) from precision sensors (IMU)** with caps and
   layout, and split the I2C buses.
4. **High-frequency PWM** removes the whine and reduces torque ripple.
5. **Keep a log.** This document has already saved repeating three mistakes.

## How this maps onto the ROS 2 stack

The bring-up firmware proved the hardware and a P-controller. This repo
generalises that work:

- the complementary filter → `imu_node` (and the ESP32 firmware);
- the PID + tip-kill → the portable `BalanceController` core;
- gain tuning that used to risk the real robot → the **Gazebo simulation**.

## References

- ELEGOO UNO R3 Starter Kit tutorial — initial single-motor wiring/code.
- E. (2019), *MX1508 vs L9110S vs TB6612 vs L293D motor driver boards* —
  arduinodiy.wordpress.com.
- Van Hunter Adams, *Complementary filters* —
  vanhunteradams.com/Pico/ReactionWheel/Complementary_Filters.html.
- IJREISS report on complementary vs Kalman filtering for balancing robots.
- Adafruit DC Gearbox "TT" motor; CamJam EduKit 3 (motors); Adafruit PowerBoost
  1000C documentation.
