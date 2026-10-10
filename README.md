
# QNX-Based Autonomous Emergency Braking (AEB) Controller

A real-time Autonomous Emergency Braking (AEB) prototype developed using **QNX SDP 8.0, C, and Raspberry Pi 4**. The system detects obstacles, evaluates collision risk, and generates braking commands using ultrasonic sensors, an MPU6050 IMU, and an L298N motor driver.

## Table of Contents

- [Overview](#overview)
- [Objectives](#objectives)
- [Features](#features)
- [Hardware Requirements](#hardware-requirements)
- [Software Requirements](#software-requirements)
- [Project Structure](#project-structure)
- [System Architecture](#system-architecture)
- [Module Description](#module-description)
- [Working Principle](#working-principle)
- [AEB Decision States](#aeb-decision-states)
- [Hardware Connections](#hardware-connections)
- [Build and Execution](#build-and-execution)
- [Testing and Validation](#testing-and-validation)
- [Future Enhancements](#future-enhancements)
- [Applications](#applications)
- [Technologies Used](#technologies-used)
- [Author](#author)
- [Disclaimer](#disclaimer)

## Overview

Autonomous Emergency Braking (AEB) is an active safety technology designed to detect potential collisions and initiate braking when necessary.

This project implements an experimental AEB controller using the QNX real-time operating system. The controller continuously acquires sensor data, evaluates obstacle distances, identifies potential hazards, and generates motor-control commands based on predefined decision thresholds.

QNX provides a real-time execution environment for embedded processing and control. The prototype demonstrates sensor integration, GPIO-based motor control, fault detection, and real-time response monitoring.

## Objectives

- Develop an AEB prototype using QNX SDP 8.0.
- Detect obstacles using ultrasonic sensors.
- Acquire motion-related information from an MPU6050 IMU.
- Evaluate obstacle-related collision risks.
- Classify driving conditions into Safe, Warning, and Emergency states.
- Generate appropriate motor-control commands.
- Detect invalid sensor readings and handle faults.
- Evaluate controller response latency.
- Demonstrate real-time embedded system development on Raspberry Pi 4.

## Features

- **Obstacle Detection:** Measures obstacle distances using HC-SR04 ultrasonic sensors.
- **IMU Integration:** Acquires motion-related data using the MPU6050.
- **Collision-Risk Assessment:** Evaluates obstacle distances against configured thresholds.
- **AEB State Management:** Supports Safe, Warning, and Emergency states.
- **GPIO Motor Control:** Controls DC motors through an L298N motor driver.
- **Sensor Validation:** Identifies invalid or unavailable sensor readings.
- **Fault Handling:** Prevents normal motor operation when required sensor data is invalid.
- **Real-Time Processing:** Executes sensor acquisition and decision logic in a QNX environment.
- **Monitoring:** Displays sensor readings, controller states, and detected faults.
- **Modular Design:** Separates sensor drivers, GPIO operations, and AEB logic into individual C modules.

## Hardware Requirements

| Component | Purpose |
|---|---|
| Raspberry Pi 4 (8 GB) | Main processing platform |
| HC-SR04 ultrasonic sensors × 2 | Obstacle-distance measurement |
| MPU6050 IMU | Motion-related data acquisition |
| L298N motor driver | DC motor control |
| DC gear motors | Prototype vehicle movement |
| Vehicle chassis | Mechanical platform |
| Raspberry Pi power supply | Powers the Raspberry Pi |
| Motor power supply | Supplies power to the motors |
| Voltage-divider resistors | Reduces ultrasonic ECHO voltage |
| Jumper wires | Electrical connections |

## Software Requirements

- QNX SDP 8.0
- QNX Momentics IDE
- C programming language
- QNX compiler and build tools
- GPIO interface
- I2C interface
- QNX-compatible Raspberry Pi 4 target environment

## Project Structure

```text
AEB_Sensor_Test_Clean/
├── include/
│   ├── public/
│   │   └── rpi_gpio.h
│   └── sys/
│       ├── aeb_logic.h
│       ├── hcsr04.h
│       ├── mpu6050.h
│       └── rpi_gpio.h
├── src/
│   ├── aeb_logic.c
│   ├── hcsr04.c
│   ├── main.c
│   ├── mpu6050.c
│   └── rpi_gpio.c
├── Makefile
└── README.md
```

## System Architecture

The system is organized into three functional layers.

### 1. Sensor Layer

The sensor layer collects data from the hardware components.

- HC-SR04 sensors measure obstacle distances.
- The MPU6050 provides inertial measurement data.
- GPIO and I2C interfaces provide communication with the connected hardware.

### 2. Processing Layer

The processing layer validates sensor readings and evaluates the current obstacle-related risk.

- Process distance measurements.
- Validate sensor data.
- Detect sensor faults.
- Evaluate configured braking thresholds.
- Determine the appropriate AEB state.

### 3. Control Layer

The control layer generates motor-control signals according to the controller's decision.

- Operate the L298N motor driver through GPIO.
- Generate the configured braking response.
- Stop normal motor operation when required sensor data is invalid.
- Monitor and report controller states.

QNX provides the operating environment for the application and its real-time processing requirements.

## Module Description

| File | Description |
|---|---|
| `main.c` | Main application, initialization, and sensor-processing loop |
| `aeb_logic.c` | AEB state management and collision-risk decision logic |
| `hcsr04.c` | HC-SR04 ultrasonic distance measurement |
| `mpu6050.c` | MPU6050 communication and data acquisition |
| `rpi_gpio.c` | GPIO initialization, output control, and cleanup |
| `aeb_logic.h` | AEB logic declarations |
| `hcsr04.h` | Ultrasonic sensor declarations |
| `mpu6050.h` | MPU6050 interface declarations |
| `rpi_gpio.h` | GPIO interface declarations |
| `Makefile` | Project build configuration |

## Working Principle

1. Initialize the GPIO interface and connected sensors.
2. Read obstacle distances from the ultrasonic sensors.
3. Acquire available motion-related data from the MPU6050.
4. Validate sensor readings and identify communication or measurement faults.
5. Compare obstacle distances against configured thresholds.
6. Determine whether the current condition is Safe, Warning, or Emergency.
7. Generate motor-control commands according to the AEB decision.
8. Stop normal motor operation when a required sensor fault is detected.
9. Display sensor readings, AEB states, and diagnostic information.
10. Evaluate response latency during prototype testing.

## AEB Decision States

| State | Description |
|---|---|
| Safe | No obstacle condition requiring a warning or emergency response is detected. |
| Warning | An obstacle reaches the configured warning threshold. |
| Emergency | The emergency threshold is reached and the controller initiates its configured braking response. |
| Sensor Fault | Required sensor data is invalid or unavailable, requiring fail-safe handling. |

**Note:** The actual thresholds and state transitions are defined by the implementation in `aeb_logic.c`. A motor stop command is a prototype response and does not, by itself, represent a validated automotive braking system.

## Hardware Connections

### Ultrasonic Sensor 1 (HC-SR04)

| Signal | Raspberry Pi Connection |
|---|---|
| VCC | Physical pin 2 (5V) |
| TRIG | GPIO23, physical pin 16 |
| ECHO | GPIO24, physical pin 18, through a voltage divider |
| GND | Physical pin 6 |

### Ultrasonic Sensor 2 (HC-SR04)

| Signal | Raspberry Pi Connection |
|---|---|
| VCC | Physical pin 4 (5V) |
| TRIG | GPIO27, physical pin 13 |
| ECHO | GPIO22, physical pin 15, through a voltage divider |
| GND | Physical pin 14 |

### MPU6050 IMU

| Signal | Raspberry Pi Connection |
|---|---|
| VCC | Physical pin 1 (3.3V) |
| GND | Physical pin 9 |
| SDA | GPIO2, physical pin 3 |
| SCL | GPIO3, physical pin 5 |
| AD0 | GND for the default I2C address, 0x68 |

### L298N Motor Driver

| Signal | Raspberry Pi GPIO |
|---|---|
| IN1 | GPIO5 |
| IN2 | GPIO6 |
| IN3 | GPIO13 |
| IN4 | GPIO19 |
| ENA | Enable jumper fitted |
| ENB | Enable jumper fitted |

The motor supply must be appropriate for the motors and driver. Connect the grounds of the Raspberry Pi and motor driver together.

**Electrical safety:** Raspberry Pi GPIO pins are not 5V tolerant. Use a voltage divider or appropriate level shifter for each HC-SR04 ECHO signal. Disconnect motor power before changing wiring, and test with the wheels raised.

## Build and Execution

### Step 1: Open the Project

Import `AEB_Sensor_Test_Clean` into QNX Momentics IDE.

### Step 2: Configure the Target

Configure the build for the appropriate Raspberry Pi 4 AArch64 QNX environment.

### Step 3: Verify the Project Configuration

Check the include paths, source files, compiler settings, and Makefile configuration.

### Step 4: Build the Application

Build the project in Momentics and resolve any compilation or linking errors.

### Step 5: Connect the Hardware

Verify the ultrasonic sensor, MPU6050, GPIO, and motor-driver connections. Ensure that the required QNX GPIO and I2C interfaces are available.

### Step 6: Deploy the Application

Deploy the compiled executable to the Raspberry Pi 4 running QNX SDP 8.0.

### Step 7: Run and Monitor

Run the application and observe sensor measurements, AEB state transitions, motor-control outputs, and fault messages.

*Exact build commands, executable names, and deployment steps depend on the configured QNX project and Makefile.*

## Testing and Validation

### 1. Obstacle Detection Test

Place an obstacle at different distances and verify the ultrasonic sensor readings.

### 2. Sensor Fault Test

Verify that invalid or unavailable sensor readings are detected and handled appropriately.

### 3. AEB State Test

Confirm that the controller transitions between Safe, Warning, and Emergency states according to the configured thresholds.

### 4. Motor-Control Test

Verify that the L298N receives the expected GPIO signals and that the motors respond correctly.

### 5. IMU Communication Test

Confirm that the MPU6050 initializes successfully and provides readable sensor data.

### 6. Response-Latency Test

Measure the interval between sensor acquisition, collision-risk evaluation, and generation of the braking command.

### 7. System Stability Test

Observe the application for unexpected termination, sensor communication failures, and inconsistent state transitions during repeated tests.

## Future Enhancements

- Implement Time-to-Collision (TTC) estimation using reliable relative-motion and speed data.
- Improve obstacle tracking and sensor fusion.
- Integrate vehicle-speed estimation.
- Add watchdog monitoring and recovery mechanisms.
- Implement structured event logging.
- Measure worst-case execution time and response latency.
- Improve fault diagnosis and recovery.
- Add hardware-in-the-loop vehicle simulation.
- Evaluate controller performance under different obstacle scenarios.

## Applications

- Automotive safety prototyping
- Real-time embedded systems development
- Collision detection and avoidance research
- QNX RTOS application development
- Sensor integration and GPIO control
- Embedded hardware testing
- Hardware-in-the-loop experimentation

## Technologies Used

| Category | Technology |
|---|---|
| Programming Language | C |
| Real-Time Operating System | QNX SDP 8.0 |
| IDE | QNX Momentics |
| Processing Platform | Raspberry Pi 4 |
| Distance Sensors | HC-SR04 |
| Inertial Sensor | MPU6050 |
| Motor Driver | L298N |
| Communication | GPIO, I2C |
| Build System | Makefile |



## Disclaimer

This project is intended for educational and research purposes only. It is an experimental prototype and is not a certified automotive safety system. Do not use it as the sole means of braking or collision prevention on a real vehicle.
