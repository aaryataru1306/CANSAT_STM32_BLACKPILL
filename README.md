# CanSat Avionics & Ground Control System

A complete STM32-based CanSat avionics, telemetry, power distribution, and Ground Control Station (GCS) system developed for real-time flight monitoring, sensor data acquisition, wireless telemetry communication, and autonomous payload deployment.

---

## Project Overview

This project was developed as part of a CanSat mission with the objective of designing a complete aerospace telemetry ecosystem capable of collecting, processing, transmitting, receiving, visualizing, and logging critical flight parameters in real time.

The system integrates avionics hardware, sensor interfaces, power management, telemetry communication, deployment mechanisms, and a Ground Control Station into a unified architecture.

The project consists of two major subsystems:

### 1. Avionics Unit (Flight Computer)

The avionics unit is responsible for:

- Sensor data acquisition
- Sensor calibration
- Data filtering
- Flight parameter estimation
- Telemetry packet generation
- LoRa transmission
- Autonomous deployment control

### 2. Ground Control Station (GCS)

The Ground Control Station is responsible for:

- Receiving LoRa telemetry packets
- Decoding transmitted data
- Displaying mission parameters
- Monitoring communication quality
- Logging telemetry data
- Supporting post-flight analysis

---

# System Architecture

```text
                           ┌─────────────────┐
                           │   GPS Module    │
                           └────────┬────────┘
                                    │ UART
                                    │
                           ┌────────▼────────┐
                           │ STM32 BlackPill│
                           │ Flight Computer│
                           └─────┬────┬─────┘
                                 │    │
                            I2C  │    │ SPI
                                 │    │
                    ┌────────────▼┐  ┌─────────────┐
                    │ MPU6050 IMU │  │ SX1278 LoRa │
                    └─────────────┘  └──────┬──────┘
                                            │
                                            │
                                      433 MHz Link
                                            │
                                            ▼
                                    ┌─────────────┐
                                    │ SX1278 LoRa │
                                    │  Receiver   │
                                    └──────┬──────┘
                                           │ SPI
                                           │
                                   ┌───────▼────────┐
                                   │ STM32 BlackPill│
                                   │ Ground Station │
                                   └───────┬────────┘
                                           │ USB
                                           │
                                           ▼
                                 Ground Control Software
```

---

# Hardware Overview

## Flight Computer

- STM32 BlackPill
- 84 MHz ARM Cortex-M4
- USB CDC support
- SPI
- I2C
- UART
- PWM

---

## Sensor Suite

### MPU6050 IMU

Provides:

- X-axis acceleration
- Y-axis acceleration
- Z-axis acceleration
- X-axis angular velocity
- Y-axis angular velocity
- Z-axis angular velocity

Applications:

- Motion sensing
- Flight dynamics monitoring
- Vertical acceleration estimation

---

### MS5611 Barometric Sensor

Provides:

- Atmospheric pressure
- Temperature
- Relative altitude

Applications:

- Altitude determination
- Vertical velocity estimation
- Flight profile analysis

---

### GPS Module

Provides:

- Latitude
- Longitude
- GPS Altitude
- Satellite Count
- GPS Fix Status

Applications:

- Position tracking
- Recovery assistance
- Mission visualization

---

## Telemetry System

### LoRa Module

Module:

SX1278

Operating Frequency:

433 MHz

Applications:

- Long-range communication
- Low power telemetry transmission
- Real-time mission monitoring

---

# LoRa Configuration

| Parameter | Value |
|------------|--------|
| Frequency | 433 MHz |
| Bandwidth | 125 kHz |
| Coding Rate | 4/5 |
| Spreading Factor | SF7 |
| CRC | Enabled |
| Sync Word | 0x34 |
| Preamble Length | 8 |
| Header Mode | Explicit |

---

# Power System

A dedicated power subsystem was designed to ensure reliable operation of all avionics components.

## Battery Pack

The system is powered using:

- 2 × 3.7V Li-Ion Cells

Connected in series:

```text
3.7V + 3.7V = 7.4V
```

Total Battery Voltage:

```text
7.4V Nominal
8.4V Fully Charged
```

---

## Power Distribution

### Buck Converter 1

Input:

```text
7.4V
```

Output:

```text
5V
```

Supplies:

- STM32 BlackPill Board
- Servo Motor

---

### Buck Converter 2

Input:

```text
5V
```

Output:

```text
3.3V
```

Supplies:

- MPU6050
- MS5611
- GPS Module
- SX1278 LoRa Module

---

## Advantages

- Stable power rails
- High efficiency
- Reduced noise
- Improved battery life
- Reliable operation during flight

---

# Custom Handcrafted PCB

A custom handcrafted PCB was designed and fabricated to integrate all avionics components into a compact and reliable flight computer.

The PCB includes:

- STM32 interface
- LoRa interface
- GPS interface
- Sensor interfaces
- Servo connector
- Battery connector
- Buck converter integration
- Power distribution network

Benefits:

- Reduced wiring complexity
- Improved reliability
- Easier maintenance
- Better packaging
- Lightweight construction

---

# Software Architecture

The avionics firmware follows a cyclic execution model.

Main tasks include:

1. Sensor Acquisition
2. Calibration
3. Data Filtering
4. State Estimation
5. Telemetry Packet Formation
6. LoRa Transmission
7. Deployment Logic

---

# Startup Sequence

## Step 1

Initialize peripherals:

- GPIO
- SPI
- I2C
- UART
- USB
- PWM

---

## Step 2

Initialize sensors:

- MPU6050
- MS5611
- GPS

---

## Step 3

Initialize LoRa Transceiver

SX1278 configuration registers are programmed.

---

## Step 4

Calibrate MPU6050

Thousands of samples are collected.

Calculated offsets:

- Accelerometer Offset
- Gyroscope Offset

Purpose:

- Remove sensor bias
- Improve measurement accuracy

---

## Step 5

Ground Pressure Calibration

Multiple pressure samples are collected.

Average pressure is stored as:

```text
Pressure Baseline
```

Used as altitude reference.

---

# Altitude Estimation

Altitude is calculated using the standard barometric equation:

```text
Altitude = 44330 × (1 − (Pressure / Baseline Pressure)^0.1903)
```

The calculated altitude is filtered before transmission.

---

# Kalman Filter

A Kalman Filter is implemented to estimate:

- Altitude
- Vertical Velocity

Inputs:

- Barometric altitude
- Vertical acceleration

Outputs:

- Filtered altitude
- Filtered velocity

Benefits:

- Reduced sensor noise
- Smooth flight profile
- Improved altitude estimation

---

# Telemetry Packet Structure

Each telemetry packet contains:

| Field | Size |
|---------|---------|
| Packet Number | 2 Bytes |
| Altitude | 2 Bytes |
| Velocity | 2 Bytes |
| Pressure | 2 Bytes |
| Temperature | 2 Bytes |
| Satellite Count | 1 Byte |
| GPS Fix | 1 Byte |
| Gyroscope X | 2 Bytes |
| Gyroscope Y | 2 Bytes |
| Gyroscope Z | 2 Bytes |
| Accelerometer X | 2 Bytes |
| Accelerometer Y | 2 Bytes |
| Accelerometer Z | 2 Bytes |
| Latitude | 4 Bytes |
| Longitude | 4 Bytes |

Total Packet Size:

```text
30 Bytes
```

---

# Autonomous Ejection Logic

## Objective

Deploy payload automatically after release.

---

## Monitoring Phase

The flight computer continuously monitors:

```text
Az_g
```

Vertical acceleration.

---

## Release Detection

Condition:

```c
Az_g <= 0
```

When detected:

```c
released = 1;
```

The system assumes free-fall has started.

---

## Deployment Phase

The servo deployment sequence begins:

```c
servo_open();
```

The payload deployment mechanism is activated.

---

## Safety Protection

Deployment is protected using:

```c
released
servo_done
```

flags.

This prevents:

- Multiple deployments
- False triggering
- Repeated servo movement

---

# Ground Control Station

The Ground Control Station receives and processes telemetry packets.

Functions:

- Packet reception
- Packet decoding
- Telemetry visualization
- RSSI monitoring
- Data logging
- Debugging support

---

# Displayed Telemetry

The GCS displays:

- Packet Number
- Altitude
- Vertical Velocity
- Pressure
- Temperature
- Latitude
- Longitude
- GPS Fix
- Satellite Count
- Accelerometer Data
- Gyroscope Data
- RSSI

---

# Communication Interfaces

## I2C

Used for:

- MPU6050
- MS5611

---

## SPI

Used for:

- SX1278

---

## UART

Used for:

- GPS Module

---

## USB CDC

Used for:

- Ground Station Output
- Debugging
- Telemetry Logging

---

# Hardware Connections

## SX1278 LoRa

| SX1278 | STM32 |
|----------|----------|
| NSS | PA4 |
| SCK | PA5 |
| MISO | PA6 |
| MOSI | PA7 |
| RESET | PB0 |
| DIO0 | PB1 |

---

## GPS Module

| GPS | STM32 |
|------|------|
| TX | USART1 RX |
| RX | USART1 TX |

---

## Servo

| Servo | STM32 |
|---------|---------|
| Signal | TIM1 CH1 |

---

# Mission Workflow

```text
Power ON
    ↓
Sensor Initialization
    ↓
Calibration
    ↓
Pressure Baseline Acquisition
    ↓
LoRa Initialization
    ↓
Telemetry Generation
    ↓
Packet Transmission
    ↓
Release Detection
    ↓
Servo Deployment
    ↓
Ground Station Monitoring
    ↓
Mission Completion
```

---

# Future Improvements

- SD Card Data Logging
- Battery Voltage Monitoring
- Flight Event Detection
- Redundant Sensor Architecture
- Real-Time Graphical Dashboard
- Telemetry Encryption
- Adaptive LoRa Data Rate
- Ground Station Mapping
- Flight State Machine
- Recovery Beacon System

---

# Applications

- CanSat Missions
- Aerospace Research
- Educational Satellites
- High Altitude Balloon Telemetry
- UAV Telemetry Systems
- Embedded Systems Research
- Wireless Sensor Networks

---

# Authors

**CanSat Avionics Team**

Developed as part of an aerospace engineering project focusing on:

- Avionics Design
- Embedded Systems
- Telemetry Systems
- Power Electronics
- Sensor Fusion
- Wireless Communication
- Ground Control Station Development
- Aerospace Electronics

---

## License

This project is intended for educational and research purposes.
