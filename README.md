# DC Boost Board

**Project:** DC Boost Board — a programmable boost converter for the University of Alberta EcoCar 2025 Prototype Vehicle. The vehicle (Sally) won first place at the Shell Eco Marathon 2025 in the Hydrogen Fuel Cell Prototype Category. 

**Purpose:** This board takes the vehicle's hydrogen fuel cell output voltage (nominally 24 V) and provides a controllable boosted output (typical target: 48 V) for the motor controller and motor. It includes measurement (voltage/current sensing), safety/protection (eFuses and monitoring), an MCU running FreeRTOS for control and telemetry, and a simple local UI (OLED + push buttons), and CAN bus communication with other boards on the vehicle and wireless base station.

---

## Table of contents

1. [Key features](#key-features)
2. [Hardware details](#hardware-details)
3. [FreeRTOS firmware overview](#freertos-firmware-overview-what-is-actually-running-on-board)

---

## Key features

* **LM5123** synchronous boost controller driving external mosfets for high-efficiency boost operation.
* Programmable dynamic voltage tuning controlled by MCU and local UI.
* Input/Output: nominal 24 V. The board is designed to output 48V but is capable of outputting 57. 
* **Input / output voltage sensing** and **current sensing** using ADCs and filtering. Firmware computes averaged values before publishing them to the display and logs.
* **TPS2623 eFuses** for controlled power delivery and overcurrent protection.
* **Local UI**: small OLED display (SSD1306) + push buttons for interaction. The display shows input/output voltages, currents, and an effeciency score.
* **FreeRTOS-based firmware** with multiple tasks handling ADC acquisition, DAC control for the tracking/virtual voltage (TRK) pin, LED/status handling, screen updates, and CAN bus communication.

---


## Hardware details

### Synchronous boost controller

* The Texas Instruments LM5123 is used as the switching controller. It drives external mosfets located at the back of the board. The 4 high and low side mosfets can be place on this board. 
* The ENABLE pin from the boost controller is driven by the MCU to allow safe startup/shutdown sequences (the code waits until input voltage is above a small threshold, then toggles ENABLE with a startup delay).
* The TRK (tracking) reference to the controller is driven by the MCU DAC to set the desired output. The firmware converts the `SET_VOLT` value to an appropriate DAC code.

### Sensing

* Voltage sensing is implemented with resistor dividers feeding ADCs.
* Current sensing is performed by hall-effect current sensors that outputs an ADC value to the MCU. 
* ADC sampling is done via DMA for efficiency

### Microcontroller

* The board is built around an STM32G91KUE6 microcontroller, which serves as the central control, sensing, and communication unit for the boost system.
* The MCU runs a FreeRTOS-based firmware architecture, allowing concurrent handling of power control, sensing, UI updates, and CAN communication without blocking critical control paths.

---

## FreeRTOS firmware overview

The firmware running on the DC Boost Board is FreeRTOS based and implemented entirely in `app_freertos.c`.  
The system is structured as a set of independent FreeRTOS tasks, each responsible for a specific functional domain: power control, sensing, user interface, and vehicle communication.

## Tasks (as implemented in `app_freertos.c`)

### DefaultTask (`StartDefaultTask`)

- Initializes the USB CDC device at startup for debug telemetry.
- Implements ENABLE pin sequencing for the LM5123 boost controller:
  - Continuously monitors input voltage.
  - Applies a one-time startup delay before asserting ENABLE once sufficient input voltage is detected.
  - Forces ENABLE low if input voltage falls below threshold and resets the startup sequence.
- Handles a push-button input to increment the output voltage setpoint (`SET_VOLT`) with bounds checking based on input voltage.

---

### ADC Conversion Task (`StartAdcConv`)

- Starts **ADC peripherals with DMA** for continuous sampling.
- Measures:
  - Input voltage
  - Output voltage
  - Input current
  - Output current
- Applies a 50-sample moving average to current measurements for noise reduction.
- Computes a real-time efficiency estimate using measured voltage and current values.
- Publishes computed values into CAN data structures and prints debug telemetry over USB CDC.

---

### Screen Print Task (`startScreenPrint`)

- Drives an **SSD1306 OLED display**.
- Displays:
  - Voltage setpoint
  - Input and output voltages
  - Input and output currents
  - Computed efficiency
- When a fault or lock condition is received over CAN, the display switches to an alarm state.
  
---

### TRK Pin Control Task (`StartTRKPin`)

- Controls the LM5123 TRK (tracking) pin using the on-chip DAC.
- Converts `SET_VOLT` into a proportional DAC output corresponding to the desired boost output voltage.

---

### Status LED Task (`StartStatusLED`)

- Controls multiple PWM-driven status LEDs using STM32 timer peripherals.
- LED indications include:
  - Boost ENABLE state
  - General system activity
- Continuously monitors the ENABLE GPIO to synchronize LED states with actual hardware behavior.

---

## CAN Bus Communication

- Uses the STM32 FDCAN peripheral for vehicle network communication.
- CAN reception is interrupt-driven and handled via the RX FIFO callback, with received frames pushed into FreeRTOS message queues.
- A dedicated receive task parses incoming CAN messages to:
  - Detect fault or lock conditions (e.g. hydrogen alarm states).
  - Update system behavior and UI state.
- A dedicated transmit task periodically publishes:
  - Input and output voltages
  - Input and output currents
  - Efficiency
  - System status flags
- CAN messaging enables real-time monitoring by other PCB's on the vehicle and the wireless telemetry system.

---
