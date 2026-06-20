# Hardware

## Bill of materials

| Part | Choice | Why / notes |
|------|--------|-------------|
| Microcontroller | **ESP32-S3** | Dual-core, plenty of GPIO, Wi-Fi; runs the loop via micro-ROS. (Earlier bring-up used an ESP32 DevKit-v1.) |
| SBC (alternative) | **Raspberry Pi 3B** | Can run the whole ROS 2 stack in Linux instead of an MCU |
| IMU | **MPU-6050** (GY-521 breakout) | 3-axis accel + 3-axis gyro over I2C, addr `0x68` |
| Motors | 2× **DC "TT" gearmotor** (Adafruit / CamJam EduKit 3) | Cheap, easy; **no encoders** so balance is IMU-only. May upgrade to steppers later. |
| Motor driver | **TB6612FNG** | ~0.5 V drop @ 1 A, thermal protection, 2.7–5.5 V logic — much better than the L298N/MX1508 tried earlier |
| Power | **Adafruit PowerBoost 1000C** | Boosts a single Li-ion to 5 V (2 A) and charges it |
| Battery | 2000 mAh 3.7 V Li-ion (JST-PH) | ~1 A boost budget is enough for these motors |
| Display | 0.96" **SSD1306 OLED** (I2C) | Live pitch / duty / battery read-out |
| Misc | rocker switch, 100 µF cap, 10 kΩ, breadboard, silicone wire | See lessons below |

## Power tree

```
  Li-ion (3.7 V) --[JST]--> PowerBoost 1000C --(5 V VOUT)--> ESP32-S3 (VIN)
                                |  |                          TB6612 VM
                                |  +--(LBO, open-drain)-----> MCU GPIO (low-batt cut-off)
                                +--(EN)--[rocker]--> battery rail (boost enable)
```

- **EN is the boost-enable pin, not a power line.** Pull it HIGH from a rail
  that exists at startup (the **battery** or USB) — **not** VOUT, which doesn't
  exist until the boost is already running (the "chicken-and-egg" trap). A 10 kΩ
  pull-down keeps it off until the rocker ties EN to the battery rail.
- **LBO** (low-battery output) is open-drain: add a pull-up to 3V3 and read it
  on a GPIO. It pulls LOW around 3.2 V → the firmware should cut the motors
  (drive TB6612 `STBY` LOW), warn on the OLED, and stop.
- Put a **100 µF low-ESR cap across the TB6612 VM/GND** to absorb motor inrush;
  without it, motor stalls brown-out the 5 V rail and reset the MCU/OLED.

## GPIO map

### ESP32-S3 (primary — firmware)

Derived from the bring-up wiring (ESP32 DevKit-v1). Re-confirm against your
exact S3 board's strap/USB pins before flashing.

| Function | Pin | Function | Pin |
|----------|-----|----------|-----|
| Motor A PWM (PWMA) | 16 | Motor B PWM (PWMB) | 23 |
| Motor A IN1 (AIN1) | 18 | Motor B IN1 (BIN1) | 21 |
| Motor A IN2 (AIN2) | 17 | Motor B IN2 (BIN2) | 22 |
| TB6612 STBY | 19 | Low-batt (LBO) | 34 (input-only, ext. pull-up) |
| IMU I2C SDA / SCL | 32 / 33 | OLED I2C SDA / SCL | 26 / 27 |

- **Use two I2C buses** (the ESP32 has two): keep the fast MPU-6050 sampling
  independent of the slower OLED refresh. Both run happily at 400 kHz.
- **Drive the motor PWM at ~20 kHz** (above hearing) to silence the whine the
  default 1 kHz produces. On ESP32 use the LEDC peripheral.
- **Avoid strap / input-only pins for outputs.** On the original ESP32, GPIO
  34–39 are input-only; putting PWM on a strap pin can stop the board booting.
  The S3 has a different strap map — check your board's datasheet.

### Raspberry Pi 3B (alternative — `motor_node` defaults, BCM numbering)

| Function | BCM | Function | BCM |
|----------|-----|----------|-----|
| Left IN1 | 17 | Right IN1 | 22 |
| Left IN2 | 27 | Right IN2 | 23 |
| Left PWM | 18 | Right PWM | 13 |
| TB6612 STBY | 24 | IMU I2C | SDA=2, SCL=3 (bus 1) |

Change these in `src/sbr_bringup/config/sbr_params.yaml`.

## Wiring: TB6612FNG ↔ MCU

```
TB6612            ->  MCU / power
VM   -> 5 V (VOUT)     VCC  -> 3V3 logic
GND  -> GND bus        STBY -> GPIO (HIGH = enabled)
AIN1 -> GPIO           AIN2 -> GPIO
BIN1 -> GPIO           BIN2 -> GPIO
PWMA -> GPIO (20 kHz)  PWMB -> GPIO (20 kHz)
A01/A02 -> Left motor  B01/B02 -> Right motor
100 µF cap across VM–GND
```

If a wheel spins the wrong way, swap that motor's IN1/IN2 in software
(`invert_left` / `invert_right`) rather than re-soldering.

## IMU mounting

Mount the MPU-6050 flat with a consistent forward axis. This project balances on
**pitch** (tilt in the driving plane). If your tilt shows up as roll, rotate the
sensor 90° or remap axes; if the sign is backwards, set `invert_pitch: true`.
Don't auto-zero the angle at boot unless the robot is guaranteed upright — a
bad boot offset makes the controller chase a phantom error.

## Safety checklist

- [ ] Tip-kill: motors cut when `|pitch| > ~45°` (`fall_threshold`).
- [ ] Low-battery: LBO LOW → motors off, OLED warning.
- [ ] 100 µF bulk cap fitted across VM/GND.
- [ ] Wheels clear / robot on a stand when first testing new gains.
- [ ] Clean flux off fine-signal pads (LBO, I2C) — residue caused phantom
      low-battery trips during bring-up.
