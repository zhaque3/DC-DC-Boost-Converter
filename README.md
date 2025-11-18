# DC Boost Board

**Project:** DC Boost Board — programmable boost converter interface for a hydrogen fuel cell vehicle

**Purpose:** This board takes the vehicle's boostable DC bus (nominally 24 V from a hydrogen fuel cell) and provides a controllable boosted output (typical target: 48 V) for the motor controller and traction motor. It includes measurement (voltage/current sensing), safety/protection (eFuses and monitoring), an MCU running FreeRTOS for control and telemetry, and a simple local UI (OLED + single push button) to let technicians or drivers adjust or observe the boost behaviour.

---

## Table of contents

1. [Key features](#key-features)
2. [Functional overview](#block-diagram--functional-overview)
3. [Hardware details](#hardware-details)
4. [FreeRTOS firmware overview (what is actually running on-board)](#freertos-firmware-overview-what-is-actually-running-on-board)
5. [Electrical design notes & part selection guidance](#electrical-design-notes--part-selection-guidance)
6. [Calibration, testing and validation procedures](#calibration-testing-and-validation-procedures)
7. [Safety & handling](#safety--handling)
8. [Assembly & building the firmware](#assembly--building-the-firmware)
9. [Troubleshooting guide](#troubleshooting-guide)

---

## Key features

* **LM5123** synchronous boost controller driving external MOSFET(s) for high-efficiency boost operation.
* Programmable **dynamic voltage tuning** (default target used in firmware: 48 V) controlled by MCU and exposed on a local UI.
* Input: nominal **24 V hydrogen fuel cell**. Output adjustable; board supports disabling the boost stage when input is below a safe threshold.
* **Input / output voltage sensing** and **current sensing** using ADCs and filtering. Firmware computes averaged values before publishing them to the display and logs.
* **TPS2623 eFuses** (or equivalent) for controlled power delivery and overcurrent protection; an ILM/fault signal is wired to the MCU to allow fast disable of the boost controller.
* **Local UI**: small OLED display (SSD1306) + a single push button for interaction. The display shows input/output voltages, currents and a setpoint readout.
* **FreeRTOS-based firmware** with multiple tasks handling ADC acquisition, DAC control for the tracking/virtual voltage (TRK) pin, LED/status handling, screen updates, and boot/default behaviour.

---

## Functional overview



High-level flow:

1. Fuel cell (24 V nominal) → input bulk filtering → eFuse(s) → inrush / precharge path → LM5123 boost stage → output filter → motor controller / traction inverter.
2. MCU monitors VIN, VOUT, input/output currents, and ILM/fault inputs. MCU controls the LM5123 enable and a DAC-tracked reference (the TRK pin) that sets the controller target.
3. Local UI (OLED + button) renders telemetry and allows simple interactions.

---

## Hardware details

### Synchronous boost controller — LM5123

* LM5123 is used as the switching controller. It drives external MOSFETs; MOSFET choice must respect Vds (recommend ≥ 80 V) and have low Rds(on) while balancing gate-charge losses.
* The EN/ENABLE pin from the boost controller is driven by the MCU to allow safe startup/shutdown sequences (the code waits until input voltage is above a small threshold, then toggles ENABLE with a startup delay).
* The TRK (tracking) reference to the controller is driven by the MCU DAC to set the desired output indirectly — firmware converts the `SET_VOLT` value to an appropriate DAC code.

### Sensing

* Voltage sensing is implemented with resistor dividers feeding ADCs. `VOLT_TRANSFER_IN` and `VOLT_TRANSFER_OUT` constants in firmware indicate divider ratios.
* Current sensing is performed by shunt-based sensors (ADC + conditioning). Firmware performs an averaging buffer (50 samples) for both input and output current measurements before using or printing the values.
* ADC sampling is done via DMA for efficiency (see `HAL_ADC_Start_DMA` usage in firmware).

### Protection & power path

* eFuses control inrush and protect the input path. An ILM/fault signal is connected to the MCU so it can immediately disable the LM5123 if a fast fault is detected.
* Output capacitors and layout should be sized for ripple and energy storage. Place low-ESR ceramics close to the MOSFETs and bulk electrolytics for energy.

---

## FreeRTOS firmware overview (what is actually running on-board)

This repository's firmware is FreeRTOS-based — the current, tested application uses a set of cooperative/independent tasks implemented in `app_freertos.c`. The important runtime behaviours implemented in firmware are **actual code** (do not treat these as pseudocode):

### Tasks (as present in `app_freertos.c`)

* **DefaultTask** (`StartDefaultTask`)

  * Initializes the USB device stack (CDC/USB) on startup.
  * Implements the boot enable logic for the boost controller: it monitors the input voltage and only toggles `ENABLE` (GPIO) after input voltage is present and a startup delay has passed. It also includes a simple button-based increment of `SET_VOLT` (note: the hardware button is connected to the BOOT0 pin in the dev build as a convenience during flashing).

* **adcConvTask** (`StartAdcConv`)

  * Starts ADC DMA channels and continuously computes averaged current measurements (50-sample moving buffer), computes VIN and VOUT from ADC readings, and computes a simple efficiency estimate.
  * Prints telemetry via `printf` for debugging (USB CDC or semihosting depending on build).

* **ScreenPrintTask** (`startScreenPrint`)

  * Drives an SSD1306 OLED (driver code `ssd1306.c`) and draws telemetry on-screen at ~10 Hz. When a `lock_state` flag is active the screen shows an alarm state; otherwise it shows SET_VOLT, IN/OUT voltages, currents and a simple efficiency value.

* **TRKPinTask** (`StartTRKPin`)

  * Converts `SET_VOLT` to a DAC value and writes it to a DAC channel to act as the boost controller's tracking reference. Runs at 1 ms intervals for smooth tracking.

* **StatusLED** (`StartStatusLED`)

  * Handles multiple PWM-driven LEDs via TIM peripherals. The task updates LED PWM compare registers to indicate enable state, regulation state (within ±1 V of setpoint), and other status indicators. Also reads the ENABLE GPIO pin to update LED states.

> The code uses HAL timers for PWM generation and starts them at boot (`HAL_TIM_PWM_Start` calls in `MX_FREERTOS_Init`).

### Important variables & behaviour

* `SET_VOLT` (float) — the present voltage setpoint (default 48 V in code sample).
* `voltage_reached` (uint8_t) — flag used to avoid repeating the startup delay sequence when input voltage falls and rises.
* ADC buffering constants and conversion factors (e.g. `VOLT_MCU`, `VOLT_TRANSFER_IN`, `VOLT_TRANSFER_OUT`, `VOLT_TO_CURR_UNI`) are defined in `app_freertos.c` and must match hardware design.
* `boost_data`, `boost_data2`, `boost_data3` structs — hold telemetry numbers formatted to your FDCAN packing scheme in your codebase. The ADC task fills these fields for display/logging.

---

## Electrical design notes & part selection guidance

* **MOSFETs**: Choose MOSFETs with a comfortable voltage margin (≥ 80 V) and low Rds(on). Gate-charge and switching losses become relevant at higher switching frequencies; balance on-resistance vs gate-charge according to your switching frequency.
* **Inductor**: Follow LM5123 recommendations. Choose an inductor with a saturation current above expected peak inductor current, and appropriate DCR/inductance to meet ripple and thermal requirements.
* **Capacitors**: Use a mix of low-ESR ceramics for decoupling and electrolytic/tantalum for bulk energy. Make sure voltage ratings exceed maximum output voltage with margin.
* **Layout**: Keep the high-current power loop short and wide. Place sensing traces away from the switching node. Use Kelvin connections for current sensing if possible.

---

## Calibration, testing and validation procedures

### ADC & sensor calibration

1. Use a calibrated bench supply and precision multimeter. Apply known voltages to VIN/VOUT sense inputs to build a slope/offset calibration table for the ADC channels.
2. For current sensors, use a programmable electronic load or a current source and an accurate ammeter to collect ADC codes vs measured current. Fit and store linear calibration values in NVM.
3. Copy the calibration constants into `app_freertos.c` or a `calibration.h` used by the `sensing` routines.

### Functional tests

* **Power-on smoke test**: With no load, energize the input and observe that the board either holds the boost disabled or ramps to a safe default. Observe temperatures and any unusual currents.
* **Step-load test**: Apply increasing loads and verify that current sensing, eFuse behaviour and ENABLE gating behave correctly.
* **UI tests**: Verify OLED updates, the SET_VOLT increments via the designated button, and DAC-tracking behavior.

---

## Safety & handling

* The board manages potentially hazardous voltages and currents. Use PPE and current-limited supplies when bench-testing.
* Ensure electrolytic capacitors are rated above maximum expected voltage.
* When bringing up the power stage for the first time, use a series test resistor or electronic load with current limit to prevent destructive faults.

---

## Assembly & building the firmware

1. Populate the board according to `hardware/BOM.csv` and `hardware/schematic.pdf`.
2. The firmware included with the repo is FreeRTOS + STM32 HAL based (the provided `app_freertos.c` is authored for the STM32 HAL). Use your standard STM32 toolchain (STM32CubeIDE, Makefile, or your CI) to build.
3. Typical workflow (example using STM32CubeIDE or a Makefile):

```bash
# build (example)
make all
# flash with ST-Link (example command)
st-flash write build/firmware.bin 0x8000000
```

4. USB CDC is initialized by the `StartDefaultTask` — connect a serial terminal to observe `printf` telemetry if built with CDC support.

5. The OLED driver used in firmware is the SSD1306 library (repository includes `ssd1306.c`/`.h`). Ensure the display wiring matches the configuration (I2C or SPI) used in `ssd1306_cfg.h`.

---

## Troubleshooting guide

* **No display output**: Verify I2C/SPI wiring and that `ssd1306_Init()` succeeds. Check `ssd1306_UpdateScreen()` calls and that the display has power.
* **No DAC output / TRK pin not changing**: Verify the DAC peripheral (check `HAL_DAC_Start` and `HAL_DAC_SetValue`), and ensure `VOLT_MCU` constant matches the MCU reference.
* **ENABLE not toggling / boost not starting**: Check `boost_data.in_volt` and the logic in `StartDefaultTask` that sets `ENABLE`. Use a multimeter to confirm the ENABLE GPIO state.
* **Unexpected current readings**: Re-check shunt wiring, ADC input ranges, and conversion constants (`VOLT_TO_CURR_UNI`, `CURR_TRANSFERx`). Compare raw ADC codes to expected voltages.

---



