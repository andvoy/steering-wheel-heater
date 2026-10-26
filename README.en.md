# Steering wheel heater

[Download version 1.9](downloads/steering-wheel-heater-1.9.zip)

[Русский](README.md) · English

Arduino Nano V3 steering wheel heater controller. Firmware version **1.9**.

A single button switches heating on and off. The controller ramps up PWM, regulates temperature, checks the sensor and power switches, and remembers ON/OFF between trips.

## Hardware

| Item | Configuration |
|---|---|
| Controller | Arduino Nano V3, ATmega328P, 16 MHz |
| Heater supply | 42 V |
| Heater wire | Tungsten, Ø0.204 mm; 19.47 Ω at 21 °C |
| Sensor | Vishay NTCLE100E3103JB0, 10 kΩ, B = 3977 K |
| Power switches | Two IRL540N MOSFETs in series on the heater return |
| Target temperature | 32 °C |
| Emergency temperature | 40 °C |

The temperature refers to the NTC location. Surface temperature distribution depends on the winding, sensor placement and wheel covering.

## Connections

Power path: **+42 V → heater → Q1 Drain → Q1 Source → Q2 Drain → Q2 Source → GND**.

| Nano pin | Connection |
|---|---|
| D9 | Q1 Gate through R2, 220 Ω; R3, 10 kΩ, from Gate to GND |
| D8 | Q2 Gate through R4, 220 Ω; R5, 10 kΩ, from Gate to GND |
| A0 | NTC to GND; R1, 10 kΩ, to 5V; C1, 100 nF, to GND |
| A1 | Q1 Drain through R6, 220 kΩ; R7, 10 kΩ, and C2, 10 nF, to GND |
| D2 | Pushbutton to GND; INPUT_PULLUP |
| D5 / D6 | Green / red LED, each through its own current-limiting resistor |
| D13 | Built-in L LED |

The two-colour LED uses a common cathode connected to GND. Each colour has a series resistor selected for its operating current.

**C1: 104 = 100 nF. C2: 103 = 10 nF.** Both capacitors are non-polarized.

Power the Nano through VIN using a separate 42-to-10 V converter: +10 V to J1.1, GND to J1.2. In this build the Nano 5V pin supplies the NTC divider; the external converter is connected to VIN. Logic and power grounds are common; heater current has its own return conductor.

[Schematic and perfboard wiring](docs/steering-wheel-heater-1.9-wiring.pdf). Component-side J1 orientation: GND on the left, +10 V on the right. The orange J1 wire goes to VIN; a separate jumper from Nano 5V supplies R1.

### Supply and capacitors

- C1: 100 nF (104); C2: 10 nF (103), both non-polarized. Installing 100 nF at C2 increases A1 settling time and can cause a false self-test fault that blocks heating.
- The assembled prototype reset during PWM when powered by its external 5 V converter. Operation became stable after changing the supply to 10 V at VIN. This is the supply arrangement documented here.
- Select the 42 V boost converter for continuous output power with thermal headroom and adequate cooling. The cold load at full output is 42² / 19.47 ≈ 90.6 W and 42 / 19.47 ≈ 2.16 A. Check the converter's rating against its cooling conditions and allowed input current.

Converter used by the author: [Ozon listing](https://ozon.ru/t/EIjlFE6). Check the product documentation for its specifications.

Arduino Nano: [Ozon listing](https://ozon.ru/t/zLZbkiN). Tungsten wire: [Ozon listing](https://ozon.ru/t/hiCkYJc).

## Operation

- A short press toggles ON/OFF and stores the selection.
- Switching on starts one heating algorithm. Up to `NORMAL_FULL_POWER_C`, it requests `MAX_PWM`; above that threshold, output is regulated by temperature.
- The PI controller warms the wheel towards 32 °C and maintains the target. The transition follows the current temperature.
- The PWM ceiling decreases linearly from 100% at 0 °C to approximately 39% at 30 °C. The controller selects output within that limit.
- PWM ramps up on activation. Power reductions and emergency shutdown take effect immediately.
- The display shows «НАГРЕВ» (heating), then «ПОДДЕРЖАНИЕ» (maintaining) near the target, starting at 31 °C by default. This label follows temperature while the control algorithm stays the same.
- At 40 °C both switches turn off and fault 2 latches. Further power-stage tests become available after cooling and successful recovery.

Faults are held in RAM. A fault stores OFF in EEPROM. Holding the button for five seconds runs a recovery test; success leaves the module OFF. A new short press starts heating.

On restart the sensor and power stage are tested before restoring the saved ON/OFF request. Fault 3 also monitors supply recovery: one second of a stable high A1 signal triggers a fresh test. Successful recovery leaves the module OFF.

## Indicators and serial output

Green indicates OFF; red indicates an ON request. The built-in L LED gives a short heartbeat in OFF and stays lit during ON.

During a fault, red and L flash the fault number: 300 ms on, 300 ms between flashes, and two seconds between groups.

| Code | Cause |
|---|---|
| 2 | Overtemperature |
| 3 | Heater supply or an open heater circuit |
| 4 | Temperature sensor |
| 5 | Q1 turn-off test |
| 6 | Q2 turn-off test |
| 7 | Power-stage conduction test |

Serial Monitor: **115200 baud**. Russian-language status is printed once per second, with a blank line after each record:

```text
Режим: НАГРЕВ | Температура: 29.3 °C | Нагрев: 39% | Питание: ~42.0 В | Мощность расч.: ~35.5 Вт | Ошибок нет
```

The fields are mode, temperature, heating duty, estimated supply voltage, calculated power and fault status.

Supply voltage is estimated from A1 during the existing both-switches-off test. Samples remain current for two seconds. Power is calculated as `U² / R × PWM / 255`, using cold heater resistance. Tungsten resistance rises with temperature, so actual warm-heater power becomes lower than this estimate. The calculation uses raw PWM; the displayed percentage is rounded.

## Configuration and upload

Open [steering_wheel_heater_v1_9.ino](firmware/steering_wheel_heater_v1_9/steering_wheel_heater_v1_9.ino) in Arduino IDE. Select Arduino Nano, the ATmega328P option matching the installed bootloader, and the board port.

Parameter comments explain units and purpose. Main settings:

| Parameter | Purpose |
|---|---|
| `SUPPLY_V` | Heater voltage and power-stage test thresholds |
| `HEATER_R_REF_OHM` | Cold resistance used for calculated power |
| `TARGET_C` | Temperature target |
| `EMERGENCY_C` | Emergency temperature |
| `NORMAL_FULL_POWER_C`, `NORMAL_GENTLE_C` | Temperature range of the PWM ceiling curve |
| `MAX_PWM`, `WARM_PWM` | Cold and warm PWM ceilings |
| `HOLD_DISPLAY_BAND_C` | Display band for the maintaining label near the target |
| `PWM_RISE_PER_SECOND` | Output ramp rate |
| `ADC_REF_V` | Measured voltage between Nano 5V and GND |
| `DETAILED_DIAGNOSTICS` | Extended ADC and test output |

## First start

1. Check wiring and polarity with all sources disconnected.
2. Check the 5V rail, NTC reading and OFF startup.
3. Connect the heater and start heating with the button. Observe supply current, switch temperature and wheel surface temperature.
4. Check button OFF/ON and ON/OFF persistence after restart.
5. Check heat distribution, cable retention and electrical insulation before installation.

## Firmware checks

Tests execute the compiled firmware in an AVR8js ATmega328P model. Coverage includes PWM, ON/OFF storage, button handling, overtemperature, sensor faults, power switches and telemetry. The model uses ideal switches and settled ADC inputs; supply integrity, transients and thermal behaviour are checked on the assembled module.

Build:

```sh
arduino-cli compile --fqbn arduino:avr:nano:cpu=atmega328 --warnings all --build-path build firmware/steering_wheel_heater_v1_9
```

Install Node.js and dependencies with `npm install`. Set `AVR_BIN` to the Arduino AVR toolchain directory containing `avr-nm`, then run:

```sh
npm test -- build
```

Included build: [test report](verification/test-report.json), [build details](verification/build.txt).
