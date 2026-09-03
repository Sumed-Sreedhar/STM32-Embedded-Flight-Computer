# STM32 Real-Time Flight Computer

A multi-sensor flight computer built on the STM32F446RE. Currently in active bring-up: SPI sensor drivers, a self-test harness, and a dual state-machine (system state + health state) core are running on bare-metal HAL. Sensor fusion, GPS, telemetry, and RTOS migration are still ahead — see [Roadmap](#roadmap--current-status).

> **Status: Active development / bring-up.** What's described below reflects what's actually implemented in the current source, not the target end-state.

---

## Overview

The firmware brings up SPI-connected IMU and barometer sensors, validates them through a self-test sequence, and tracks two independent state machines: an overall **system state** (boot/test/active, currently cycled manually via a debug button for bench validation) and a **health state** (normal/fault/critical-fault) that reflects sensor self-test and runtime status through dedicated LEDs. This bench-validation core is the foundation the rest of the flight computer (sensor fusion, GPS, logging, telemetry, RTOS tasks) is being built on top of.

## Hardware (currently integrated)

| Component | Interface | Role |
|---|---|---|
| STM32F446RE (Nucleo) | — | Main controller |
| MPU9250/MPU6500 IMU | SPI2 | Accelerometer + gyroscope (raw read) |
| BMP280 | SPI2 | Barometric temperature + pressure |
| 2x push buttons (STATE_BT, FAULT_BT) | GPIO/EXTI | Manual state-machine test harness |
| Onboard user button (B1) | GPIO/EXTI | Reserved |
| Status/Normal/Fault LEDs | GPIO | Health + state indication |

**Not yet integrated:** magnetometer, GPS module, microSD/FatFS logging, OLED display, ESP32-S3 telemetry bridge.

## Software Stack

- STM32 HAL (CubeMX-generated peripheral init)
- Polled SPI drivers for BMP280 and MPU9250/MPU6500
- BMP280 factory calibration loading + Bosch compensation formulas (temperature, pressure)
- Self-test framework gating health state on sensor WHO_AM_I / init / read success
- Superloop main() — no RTOS yet

## Architecture

### System State Machine

Implemented in `system_state.h` / `button.c`, currently advanced manually via the `STATE_BT` button for bench testing (`GPIO_State_Button_Callback`):

```
BOOTING → SELF_TEST → ACTIVE → BOOTING → ...   (cycles on button press)
```

Any unhandled state transition forces `current_health_state = CRITICAL_FAULT`.

### Health State Machine

Implemented in `system_health.h` / `system_health.c`, driven both automatically (self-test results) and manually via the `FAULT_BT` button (`GPIO_Fault_Button_Callback`):

```
NORMAL ⇄ FAULT ⇄ CRITICAL_FAULT ⇄ NORMAL   (button-cycled; also set by self-test outcome)
```

LED behavior per state (`handle_system_health`):

| State | Normal LED | Fault LED |
|---|---|---|
| NORMAL | On | Off |
| FAULT | Off | On (steady) |
| CRITICAL_FAULT | Off | Blinking |

### Self-Test Sequence

On boot, `BMP280_SelfTest()` and `MPU9250_SelfTest()` each independently gate `current_health_state`:

- Chip ID / WHO_AM_I mismatch or SPI comms failure → `CRITICAL_FAULT`
- Init, calibration-load, or first-read failure → `FAULT`
- All checks pass → `NORMAL`

### Sensor Drivers

- **BMP280:** chip ID read, config/ctrl_meas register setup, 24-byte factory calibration load, raw pressure/temperature read, full Bosch compensation math (`BMP280_CompensateTemp`, `BMP280_CompensatePressure`) — output in real engineering units (°C, Pa).
- **MPU9250/MPU6500:** WHO_AM_I check (expects `0x71`), power management + gyro/accel full-scale + DLPF config (41 Hz DLPF tuned for a future 100 Hz sensor task), raw 14-byte burst read (accel, temp, gyro). Currently reports scaled but uncalibrated accel/gyro values over UART for debug — no filtering or fusion applied yet.

Both sensors' raw + processed values are logged out over UART (`USART2`) once per second for bench debugging.

## Roadmap / Current Status

- [x] SPI drivers for BMP280 and MPU9250/MPU6500 (raw read)
- [x] BMP280 factory calibration + full compensation math
- [x] Self-test harness gating a health state machine
- [x] Manual debug-button-driven system/health state machines
- [x] LED-based health/state indication
- [x] UART debug output of live sensor data
- [ ] Standalone magnetometer integration (yaw correction)
- [ ] GPS (NEO-6M) driver + NMEA parsing
- [ ] microSD/FatFS telemetry logging
- [ ] OLED status display
- [ ] Mahony sensor fusion filter
- [ ] Autonomous flight state machine (BOOT → ARMED → FLIGHT → LANDING, replacing the current manual test-harness state machine)
- [ ] FreeRTOS task migration
- [ ] ESP32-S3 telemetry bridge

## Repository Structure

```
/Core        — application source (state machines, self-test, sensor drivers, main loop)
/Drivers     — HAL + custom SPI/I2C peripheral code
```

## Build

Developed in STM32CubeIDE / STM32CubeMX. Flash to an STM32F446RE Nucleo board with the SPI2 sensors and debug buttons wired per the hardware table above. Sensor readings are streamed over `USART2` at the configured baud rate for bench monitoring.

---

*Part of a broader embedded systems portfolio — see [github.com/Sumed-Sreedhar](https://github.com/Sumed-Sreedhar) for related STM32 firmware projects.*
