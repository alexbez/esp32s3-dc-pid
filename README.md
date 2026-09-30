# ESP32-S3 closed-loop DC motor speed control

PlatformIO / Arduino firmware that holds a DC motor at a speed set by a potentiometer.

- **MCU:** ESP32-S3 (tested config: `esp32-s3-devkitc-1`)
- **Motor:** NFP-GM37-520-PEN, 12 V, 107 rpm (output shaft), with Hall quadrature encoder
- **Driver:** TB6612FNG (channel A)
- **Setpoint:** potentiometer, 0 to 95 rpm
- **Feedback:** speed from encoder counts over a fixed 20 ms sample period
- **Controller:** PI with feed-forward and anti-windup, gains adjustable over serial

## How it works

1. Both encoder channels are decoded at 4x resolution in interrupts.
2. Every 20 ms the firmware takes the change in counts and converts it to rpm (then smooths it).
3. The potentiometer value is oversampled, filtered and rate-limited to give the target rpm.
4. The PI controller computes a PWM duty: `duty = target/107 (feed-forward) + Kp*error + integral`.
   The integrator freezes while the output is saturated (anti-windup).
5. The duty drives the TB6612FNG at 20 kHz PWM. Below 3 rpm the motor is stopped and braked.

## Hardware

| Part | Notes |
|---|---|
| ESP32-S3 dev board | Powered over USB |
| TB6612FNG breakout | Motor driver, 1.2 A continuous / 3.2 A peak per channel |
| NFP-GM37-520-PEN motor, 12 V | With encoder, 6-wire |
| 12 V DC supply | Must cover the motor stall current, see the warning below |
| 10 kOhm potentiometer | Linear (B10K) |
| 100 uF electrolytic capacitor (recommended) | Across the TB6612FNG VM and GND |

## Wiring

All grounds (ESP32-S3, TB6612FNG, 12 V supply, encoder, potentiometer) must be connected together.

### ESP32-S3 to TB6612FNG

| TB6612FNG pin | Connect to | Purpose |
|---|---|---|
| PWMA | GPIO6 | Speed (PWM, 20 kHz) |
| AIN1 | GPIO4 | Direction |
| AIN2 | GPIO5 | Direction |
| STBY | GPIO7 | Driver enable (HIGH = on) |
| VCC | ESP32-S3 3V3 | Logic supply |
| GND | Common GND | |
| VM | +12 V supply | Motor supply |
| AO1 | Motor M+ terminal | |
| AO2 | Motor M- terminal | |
| PWMB, BIN1, BIN2, BO1, BO2 | Not connected | Channel B is unused |

If the motor turns the wrong way, swap the two motor wires on AO1/AO2.

### Motor and encoder

The GM37-520 has six wires. Colors vary between manufacturers, so check the datasheet of your motor before powering it. A typical layout:

| Motor wire (typical color) | Function | Connect to |
|---|---|---|
| Red | Motor + | TB6612FNG AO1 |
| White | Motor - | TB6612FNG AO2 |
| Blue | Encoder VCC | ESP32-S3 3V3 |
| Black | Encoder GND | Common GND |
| Yellow | Encoder channel A | GPIO15 |
| Green | Encoder channel B | GPIO16 |

Power the encoder from 3.3 V, not 5 V. The encoder outputs then swing to 3.3 V, which is safe for the ESP32-S3 inputs. The firmware enables internal pull-ups on GPIO15 and GPIO16.

### Potentiometer

| Potentiometer pin | Connect to |
|---|---|
| One outer pin | ESP32-S3 3V3 |
| Wiper (middle) | GPIO1 (ADC1) |
| Other outer pin | GND |

### Wiring diagram

```
                      +12 V supply
                        |      |
                  [100uF]      |
                        |      |
   ESP32-S3         TB6612FNG  |
  ----------        ---------- |
   3V3 -----------> VCC    VM -+
   GND -----------> GND
   GPIO6 ---------> PWMA  AO1 ------> Motor +
   GPIO4 ---------> AIN1  AO2 ------> Motor -
   GPIO5 ---------> AIN2
   GPIO7 ---------> STBY

   GPIO15 <-------- Encoder A     (encoder VCC -> 3V3, GND -> GND)
   GPIO16 <-------- Encoder B

   3V3 --[ 10k pot ]-- GND
            |
   GPIO1 <--+ (wiper)

   12 V supply GND ---- TB6612FNG GND ---- ESP32-S3 GND  (common ground)
```

### Power warnings

- The motor draws roughly 0.2 A with no load, but the stall current of a GM37-520 is typically 2 A or more. That is above the 1.2 A continuous rating of the TB6612FNG. Do not hold the shaft for long, and use a current-limited supply for first tests.
- Do not connect or disconnect the motor supply while it is powered.
- Connect the 100 uF capacitor close to the VM and GND pins, with the correct polarity.

## Pin changes

Pins are defined at the top of `src/main.cpp`. If you change them, avoid the ESP32-S3 strapping pins (GPIO0, 3, 45, 46), the USB pins (GPIO19, 20) and the flash/PSRAM pins (GPIO26 to 37). The potentiometer must be on an ADC1 pin (GPIO1 to 10).

## Build and upload

Install [PlatformIO](https://platformio.org/install), then from the project folder:

```bash
pio run                 # build
pio run -t upload       # flash the board
pio device monitor      # serial monitor, 115200 baud
```

If you use a different ESP32-S3 board, change `board` in `platformio.ini`.

## Before the first run

Check the encoder settings at the top of `src/main.cpp`:

- `ENCODER_PPR_MOTOR = 11` and `GEAR_RATIO = 90` are common values for the GM37-520 (3960 counts per output revolution). Confirm them against your motor's datasheet. Wrong values scale the measured and displayed rpm by the same factor.
- If the measured speed is negative when the motor runs forward, set `ENCODER_INVERT = true`.
- `MOTOR_NOLOAD_RPM = 107` and `MAX_SETPOINT_RPM = 95` set the feed-forward and the top of the potentiometer range. Keep the maximum setpoint below the no-load speed so the controller has headroom.

## Serial interface

Open the serial monitor at 115200 baud. The firmware prints one CSV line every 50 ms:

```
setpoint_rpm,measured_rpm,duty_percent
```

This format works with a serial plotter. Status and error replies start with `#`.

Type one command per line:

| Command | Action |
|---|---|
| `kp <value>` | Set the proportional gain (duty per rpm). With no value, prints the current gains. |
| `ki <value>` | Set the integral gain (duty per rpm and second). With no value, prints the current gains. |
| `get` | Print the current gains. |
| `save` | Store the gains in flash. They are restored on boot. |
| `default` | Restore the built-in gains (`Kp = 0.004`, `Ki = 0.030`). Run `save` to make it permanent. |
| `stream on` / `stream off` | Enable or disable the CSV output. |
| `help` | List the commands. |

Gains must be numbers from 0 to 10. Invalid values are rejected and the old gains stay active.

### Tuning procedure

1. Set `ki 0` and turn the potentiometer to a mid-range speed.
2. Raise `kp` until the speed reacts quickly to changes without oscillating.
3. Raise `ki` until the speed settles on the setpoint quickly after a load change, without overshoot.
4. Run `save` to keep the gains.

## Unit tests

The PI controller logic is in `lib/PiController/PiController.h` and has no hardware dependencies. The tests run on your computer:

```bash
pio test -e native
```

They cover the proportional and integral terms, gain updates (including rejection of invalid values), and anti-windup at both output limits.

## Project layout

```
platformio.ini                        PlatformIO environments (board and native tests)
src/main.cpp                          Pins, encoder, PWM, serial interface, control loop
lib/PiController/PiController.h       PI controller with feed-forward and anti-windup
test/test_pi_controller/test_main.cpp Unit tests
```

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| Motor does not move | STBY not high, VM not powered, or GND not common. Check the wiring table. |
| Motor runs at full speed and the measured rpm is 0 | Encoder not connected or not powered. Check GPIO15, GPIO16 and encoder VCC/GND. |
| Measured rpm is negative | Set `ENCODER_INVERT = true`. |
| Measured rpm is off by a constant factor | Wrong `ENCODER_PPR_MOTOR` or `GEAR_RATIO`. |
| Speed oscillates | Lower `kp` and `ki`. |
| Speed is slow to reach the setpoint | Raise `ki`, or check that the supply voltage is 12 V under load. |
| Setpoint jitters | Shorten the potentiometer wires, or lower `SETPOINT_FILTER_ALPHA`. |
