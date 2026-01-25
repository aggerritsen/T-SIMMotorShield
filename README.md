# T-SIM Relay shield (ESP32 / ESP32-S3)

This repository contains a **clean, deterministic GPIO + I2C baseline** used to validate pinout consistency and external bus behavior across **LilyGO T-SIM boards** when used with a **generic T-SIM Relay Shield**.

The shield also provides a male header connector for the XIAO slot of the Seeed Grove Vision AI V2 board.

The focus of this development is **hardware correctness and repeatability**, not application logic.
It provides a proven foundation for later integration of actuators (relays, DC or stepper motors) and external I2C peripherals (sensors, expanders, AI modules).

This baseline deliberately **excludes UART and transport logic**. USB Serial is used **only for logging**.

---

## Printer Circuit Board (PCB)

A Fritzing file (`.fzz`) is available to order the PCB at a manufacturer of choice.

---

## Goals of This Development

This project exists to answer these questions conclusively:

* Are GPIO pins wired correctly across all supported boards?
* Is GPIO output behavior consistent (timing, drive, polarity)?
* Is external **I2C-1** stable at **400 kHz** on all boards?
* Can the same Grove / relay / motor / sensor shields be reused across boards?

Example shield:

![PCB Relay Shield](pcb/PCB-RLY.jpg)


---

## Design Principles

* **GPIO output test for D0 and D1**
* **I2C-1 only** (I2C-0 remains free for internal / onboard use)
* **Non-blocking design** (watchdog-safe)
* **Identical firmware across all boards**
* Board differences handled **only via PlatformIO build flags**

---

## What the Firmware Does

### GPIO Test (D0 / D1 only)

* Uses **two output pins**: `D0` and `D1`
* Alternating pattern ("walking 1"):

  * D0 HIGH, D1 LOW
  * D0 LOW, D1 HIGH
* Interval configurable via `GPIO_TEST_INTERVAL_MS_CFG` (default: 200 ms)
* Non-blocking (`millis()` based)

This validates:

* Correct GPIO mapping
* Output drive capability
* Timing stability

### I2C-1 Scan (external bus)

* Uses **I2C bus 1** (`TwoWire(1)`)
* SDA / SCL pins defined per board via build flags
* Frequency: **400 kHz** (configurable)
* Scan starts **after each full D0 → D1 GPIO cycle**
* Scan is **incremental**:

  * Exactly **one I2C address per loop iteration**
  * Fully watchdog-safe

Scan output is printed to USB Serial.

---

## Supported Boards (current & verified)

| Board         | MCU        | Status     |
| ------------- | ---------- | ---------- |
| XIAO ESP32-S3 | ESP32-S3   | ✅ Verified |
| T-SIM7080G-S3 | ESP32-S3   | ✅ Verified |
| T-SIM7070     | ESP32      | ✅ Verified |
| T-SIM7000G    | ESP32      | ✅ Verified |
| T-SIM7600     | ESP32 / S3 | 🔜 Planned |
| T-SIM7670G-S3 | ESP32-S3   | 🔜 Planned |

All verified boards run **the exact same firmware source**, differing only by PlatformIO environment configuration.

---

## Pin Configuration

### T-SIM7080G-S3 (ESP32-S3)

| Signal | GPIO   | Notes            |
| ------ | ------ | ---------------- |
| 3V3    | —      | (P1.1)           |
| GND    | —      | (P1.2)           |
| SDA    | GPIO16 | I2C-1 (P1.3)     |
| SCL    | GPIO17 | I2C-1 (P1.4)     |
| D0     | GPIO03 | GPIO test output |
| D1     | GPIO46 | GPIO test output |

### T-SIM7000G / T-SIM7070 / T-SIM7600 / T-SIM7670G (ESP32 classic / S3)

| Signal | GPIO   | Notes            |
| ------ | ------ | ---------------- |
| 3V3    | —      | (P1.1)           |
| GND    | —      | (P1.2)           |
| SDA    | GPIO32 | I2C-1 (P1.3)     |
| SCL    | GPIO33 | I2C-1 (P1.4)     |
| D0     | GPIO26 | GPIO test output |
| D1     | GPIO25 | GPIO test output |

> Pins D2–D7 are intentionally **unused** in this baseline.

---

## Architecture Overview

```
+---------------------------+
| ESP32 / ESP32-S3 Board    |
|                           |
|  D0 / D1  ---> GPIO test  |
|  I2C-1    ---> Grove bus  |
|                           |
|  USB Serial (logs only)   |
+---------------------------+
```

---

## Logging

All activity is logged via **USB Serial**:

```
[      4123] GPIO D0 HIGH
[      4323] GPIO D1 HIGH
=======================================
 I2C-1 SCAN START (after GPIO cycle)
=======================================
  ✓ I2C device found at 0x3C
  Total devices: 1
  Scan duration: 128 ms
```

Logging includes:

* GPIO state transitions
* I2C scan start / completion
* Detected I2C device addresses
* Scan duration

---

## PlatformIO Usage

### Upload (example)

```ini
upload_port  = COM3
monitor_port = COM3
monitor_speed = 115200
```

### Build & upload

```bash
pio run -e t-sim7080g-s3 -t upload
```

### Monitor

```bash
pio device monitor
```

---

## What This Code Is (and Is Not)

### ✅ This is

* A GPIO and I2C validation harness
* A cross-board hardware sanity check
* A deterministic, watchdog-safe baseline
* Safe to extend with actuators or sensors

### ❌ This is not

* A transport or UART test
* A communication protocol
* Optimized for throughput
* Tied to AI, camera, or application logic

---

## Status

✅ **GPIO and I2C behavior proven stable across multiple boards**

This repository represents a known-good, cross-board baseline for:

* GPIO output validation (D0 / D1)
* External I2C-1 scanning at 400 kHz
* Reusable Grove-based hardware development
