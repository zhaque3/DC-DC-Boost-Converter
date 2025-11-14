# DC Boost Board

**Project:** DC Boost Board — programmable boost converter interface for hydrogen fuel cell vehicle

**Purpose:** This board takes the vehicle's boostable DC bus (nominally 24 V from a hydrogen fuel cell) and provides a controllable boosted output (typical targets: 48 V or 57 V) for the motor controller and traction motor. It includes measurement (voltage/current sensing), safety/protection (eFuses and monitoring), MCUs for control and CAN connectivity, and a simple local UI (OLED + single push button) to let technicians or drivers adjust the boost voltage on-the-fly.

---

## Table of contents

1. [Key features](#key-features)
2. [Block diagram & functional overview](#block-diagram--functional-overview)
3. [Hardware details](#hardware-details)

   * [Synchronous boost controller (LM5123)](#synchronous-boost-controller-lm5123)
   * [Input / output sensors](#input--output-sensors)
   * [Power protection — TPS2623 eFuses and ILM pin](#power-protection---tps2623-efuses-and-ilm-pin)
   * [MCU & firmware interface](#mcu--firmware-interface)
   * [OLED + single-button UI behavior](#oled--single-button-ui-behavior)
   * [Status LEDs](#status-leds)
   * [Connectors and pinout](#connectors-and-pinout)
4. [Electrical design notes & part selection guidance](#electrical-design-notes--part-selection-guidance)
5. [CAN interface specification](#can-interface-specification)
6. [Firmware architecture & APIs](#firmware-architecture--apis)
7. [Calibration, testing and validation procedures](#calibration-testing-and-validation-procedures)
8. [Safety & handling](#safety--handling)
9. [Assembly & building the firmware](#assembly--building-the-firmware)
10. [Troubleshooting guide](#troubleshooting-guide)

---

## Key features

* **LM5123** synchronous boost controller driving external MOSFET(s) for high-efficiency boost operation.
* Programmable **dynamic voltage tuning** (default boost targets: 48 V and 57 V) controlled by MCU.
* Input: nominal **24 V hydrogen fuel cell**. Output adjustable, with option to reduce output to **25 V or 30 V** or fully disable boost.
* **Input / output voltage sensing** and **current sensing** on both input and output rails.
* **TPS2623 eFuses** for controlled power delivery and overcurrent protection.
* **ILM (current limit) monitoring** line available to MCU for fast shutdown or derating.
* **CAN bus** connectivity — reads vehicle RUN state and other telemetry; publishes board status and telemetry.
* **OLED + single push-button** local UI to change voltage setpoint with single-button UX.
* **5 MCU-controlled LEDs** for status indicators (user-configurable meanings).

---

## Block diagram & functional overview

(Place a schematic / block diagram image in the repository `docs/` and link here.)

High-level flow:

1. Fuel cell (24 V nominal) → input bulk filtering → TPS2623 eFuses → inrush / precharge path → LM5123 boost stage → output filter → motor controller / traction inverter.
2. MCU monitors input V/I, output V/I, ILM and other flags. MCU adjusts LM5123 via a digital interface (DAC or PWM-to-analog tuning) to set the target output.
3. MCU communicates over CAN to receive car state (RUN/COAST/BRAKE) and may change boost behaviour accordingly (e.g., lower setpoint or disable boost while coasting to save hydrogen).
4. Local UI (OLED + single-button) allows technicians to manually adjust output voltage when allowed.

---

## Hardware details

### Synchronous boost controller — LM5123

The board uses an LM5123 controller as the main switching regulator. Important design notes:

* LM5123 is configured as a synchronous boost (external high-side/low-side MOSFETs). Ensure MOSFET selection follows the LM5123 gate drive and Vds ratings for the expected VIN (24 V) and VOUT (up to 57 V plus margin).
* Compensation network, inductor selection, and output capacitors are chosen to ensure stability at the switching frequency used. Typical switching frequency and inductor values were selected during design verification — see `docs/power_design_notes.md` for the full loop-analysis and component choices.
* The LM5123's EN/SHUTDOWN pin is connected to the MCU for enable/disable control.
* The LM5123 can be tuned dynamically: the MCU changes the control reference (via DAC or filtered PWM) to change the target output voltage.

### Input / output sensors

* **Voltage sensing:** resistor divider networks on VIN and VOUT feed ADC channels on the MCU. Divider values chosen to keep ADC inputs within range (documented in `hardware/` schematic).
* **Current sensing:** differential current sensors (shunt + amplifier or dedicated IC) measure input and output currents. The ILM pin reports fast overcurrent events (e.g., from a dedicated comparator or the eFuse) to the MCU.
* ADC channels for sensors must be calibrated. See [Calibration](#calibration-testing-and-validation-procedures).

### Power protection — TPS2623 eFuses and ILM pin

* TPS2623 devices provide programmable current limits and fault reporting. They protect against sustained overcurrent and latch or auto-retry depending on config.
* The ILM pin is available to the MCU for rapid reaction to overcurrent or over-temperature. ILM is routed such that the MCU can immediately disable the LM5123 via the EN pin.

### MCU & firmware interface

* MCU handles: dynamic voltage tuning, reading sensors via ADC, safety monitoring, CAN communications, local UI handling, LED status control, and eFuse interactions.
* The firmware exposes a clean module separation:`

  * `power_control` (LM5123 interactions, setpoint handling)
  * `sensing` (ADC reads, filtering, calibration)
  * `safety` (fault handling, eFuse control, ILM response)
  * `can_comm` (message packing/parsing, heartbeat)
  * `ui` (OLED + button handling and menu logic)

**Important:** Boot behavior: MCU should set LM5123 to a safe default (output disabled) until sensors and CAN states are validated.

### OLED + single-button UI behavior

This board uses a single push button and a small OLED to make adjustments. The UI behaviour implemented is:

1. **Short press** — move cursor between options.
2. **Long press (3 seconds)** on the currently selected option — select that option.
3. **Menu flow**:

   * Press once to bring up menu with two options: `UP` and `DOWN`.
   * If `UP` is selected and long-pressed (3s), the board enters increase-voltage mode: each short press increments the setpoint by `+1 V`.
   * If `DOWN` is selected and long-pressed (3s), the board enters decrease-voltage mode: each short press decrements the setpoint by `-1 V`.
   * **Timeout:** if no button press for **10 seconds**, the menu fades/disappears and the UI returns to normal telemetry display.

This UX is intentionally minimal to let a single button do both selection and adjustment.

### Status LEDs

* **5 MCU-controlled LEDs** are exposed on the board. Default meanings we used in development:

  1. Power present (input ok)
  2. Boost enabled
  3. Fault (latched)
  4. CAN activity / heartbeat
  5. User-defined (e.g., charge mode)

You can remap the LEDs in firmware easily.

### Connectors and pinout

* **Main power input** — high-current connector (24 V fuel cell in). See `hardware/schematic.pdf` for footprint and pin mapping.
* **Boost output** — high-current connector to motor controller / load.
* **CAN_H / CAN_L** — isolated CAN bus connector to the vehicle network.
* **I/O header** — includes: MCU debug (SWD), ILM, eFuse fault signals, GPIOs, and I2C/SPI (for expansion).
* **OLED header** — I2C or SPI header for OLED (depending on chosen display)
* **Button** — mechanical pushbutton routed to MCU GPIO with external pull-down/pull-up as in schematics.

(Exact connector pinout is documented in `hardware/` directory and in the schematic PDF.)

---

## Electrical design notes & part selection guidance

* **MOSFETs** should be chosen with Vds rating comfortably above the maximum expected Vout (recommend ≥ 80 V to provide margin). Rds(on) should be chosen for low conduction losses while considering gate charge for switching losses.
* **Inductor** selection: the LM5123 design guide recommends a specific inductance for a given switching frequency. Keep the inductor saturation current rating higher than the maximum expected peak inductor current (account for duty cycle and ripple). See `docs/power_design_notes.md` for the loop analysis and suggested part numbers.
* **Output caps**: low ESR electrolytic and ceramic bulk caps sized for ripple and energy storage. Place ceramics close to MOSFETs for switching transients.
* **Gate driving**: ensure gate resistors and snubbers limit ringing; follow layout best practices.
* **Layout**: keep the power loop (VIN → MOSFETs → inductor → output cap → return) short and with wide copper. Place current shunt and sensing amplifiers away from high dv/dt nodes, and use Kelvin sense where needed.

---

## CAN interface specification

The CAN interface is the primary vehicle integration point. We use a simple message set for telemetry and control. The IDs and payloads below are recommended — they should be configurable in firmware.

**Example messages**

* `0x180` — **BoostControlCmd** (Tx: vehicle → Boost board)

  * Byte 0: Command type (0x01 = set voltage, 0x02 = enable, 0x03 = disable)
  * Byte 1: Voltage high byte (signed or unsigned depending on encoding)
  * Byte 2: Voltage low byte (LSB, 0.1 V units recommended)
  * Bytes 3-7: reserved / checksum

* `0x280` — **BoostStatus** (Tx: Boost board → vehicle)

  * Byte 0: State flags (bit0: enabled, bit1: fault, bit2: can_ok, ...)
  * Byte 1: Output voltage MSB (0.1 V units)
  * Byte 2: Output voltage LSB
  * Byte 3: Output current MSB (0.1 A units)
  * Byte 4: Output current LSB
  * Bytes 5-7: reserved

* `0x2A0` — **Telemetry** (periodic)

  * Packed VIN, IIN, VOUT, IOUT, temperature, fault codes.

**Heartbeat & safety**

* MCU must monitor CAN heartbeat from vehicle's master at a configured timeout (e.g., 250 ms). If heartbeat lost for a safety timeout (e.g., 500 ms–2000 ms), the board should move to a safe state (reduce setpoint to a safe limit or disable boost entirely), depending on vehicle integration rules.

---

## Firmware architecture & APIs

Suggested modular layout:

```
/src
  /boot
  /drivers
    adc.c / adc.h
    can.c / can.h
    gpio.c / gpio.h
    lm5123.c / lm5123.h
    eFuse.c / eFuse.h
    oled.c / oled.h
    button.c / button.h
  /app
    power_control.c
    sensing.c
    safety.c
    ui.c
  main.c

/docs
  firmware_architecture.md
```

Key algorithms & behaviors:

* **Voltage setpoint smoothing**: when switching setpoints (CAN or UI), ramp the reference smoothly to avoid step changes in duty cycle and large inrush currents.
* **Current limiting**: perform closed-loop current limiting using ILM and the current sensors. If a fast overcurrent event occurs, immediately disable the LM5123 and report fault over CAN.
* **Watchdog & fault escalation**: use hardware watchdog. On repeated faults, latch the board until cleared by CAN or a power-cycle.

Example API calls (pseudo-C):

```c
// set voltage target in volts (integer)
void power_control_set_voltage(int volts);

// enable / disable boost
void power_control_enable(bool en);

// read telemetry struct
telemetry_t telemetry = sensing_read_all();

// send status
can_send_boost_status(&telemetry);
```

---

## Calibration, testing and validation procedures

### ADC & sensor calibration

1. Connect a calibrated bench supply and precision multimeter to VIN. Apply 12 V, 24 V and any other expected operating points.
2. Record ADC raw codes vs measured voltages. Fit a linear calibration (slope & offset) and store in non-volatile memory.
3. Repeat for current sensors using a programmable electronic load and accurate ammeter.

### Functional tests

* **Power-on smoke test**: With no load, enable the board and verify Vout ramps to the default safe setpoint (or stays disabled per config). Watch for smoke, overheating, or abnormal currents.
* **Step-load test**: Apply increasing loads up to expected maximum. Verify current sensor responses, eFuse trips, and ILM behaviour.
* **Transient & loop stability**: Apply step changes in load and verify voltage regulation and loop stability (oscillations, overshoot).

### Integration tests with vehicle CAN

* Verify CAN message reception and correct interpretation at multiple setpoints.
* Validate safety timeouts for lost heartbeat.

---

## Safety & handling

* This board handles potentially hazardous voltages (24 V → up to 57 V and high currents). Work only in accordance with safe lab practices.
* When testing: use proper PPE, fuses, and current-limited bench supplies where possible.
* Ensure electrolytic capacitors are rated for the maximum Vout with margin.

---

## Assembly & building the firmware

1. Populate board according to `hardware/BOM.csv` and `hardware/schematic.pdf`.
2. Connect SWD and flash the firmware via your chosen tool (e.g., STLink, or Atmel-ICE if the MCU is AVR). Bootloader / programmer steps are documented in `docs/flash_instructions.md`.
3. Build system: use `Makefile` (or PlatformIO/CMake depending on repo). Example:

```bash
# from repo root
make all
make flash
make monitor
```

(Provide concrete toolchain & CPU target in `docs/`.)

---

## Troubleshooting guide

* **No output voltage**: Check EN pin state, check eFuse status, verify MOSFET gate driver supply, check bootloader sets safe default.
* **Output voltage present but unstable**: Check compensation network, inductor saturation, output capacitor ESR, layout.
* **eFuse tripping repeatedly**: Check for short on output, measure current spikes with oscilloscope, review inrush paths.

---

## Contributing

Contributions are welcome. Please follow these guidelines:

1. Open an issue describing the feature or bug.
2. Fork the repo and create a branch for your change.
3. Add clear commit messages and unit tests for firmware where possible.
4. Submit a pull request with a clear description and tests.

Please follow the repository's coding style and document any hardware changes in the `hardware/` directory.

---

## Files & folders (suggested repository layout)

```
/ hardware/         # schematics, PCB, BOM
/ docs/             # design notes, power calculations, testing procedures
/ firmware/         # firmware sources and build system
/ examples/         # sample CAN scripts and test logs
/ tools/            # helper scripts (BOM parsers, calibration tools)
/ README.md         # this file
```

---


---

*Last updated: November 14, 2025*
