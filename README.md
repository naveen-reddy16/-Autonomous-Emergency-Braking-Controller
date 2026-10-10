
# QNX-Based Autonomous Emergency Braking (AEB) Controller

A real-time Autonomous Emergency Braking (AEB) prototype developed using **QNX SDP 8.0, C, and Raspberry Pi 4**. The system detects obstacles, evaluates collision risk, and generates braking commands using ultrasonic sensors, an MPU6050 IMU, and an L298N motor driver.

## Features

- HC-SR04 ultrasonic obstacle detection
- MPU6050 IMU data acquisition
- Distance-based collision-risk assessment
- Safe, warning, and emergency states
- GPIO-based motor control
- Sensor validation and fault handling
- Real-time sensor monitoring and logging
- Modular C implementation

## Hardware Requirements

- Raspberry Pi 4 (8 GB)
- 2 × HC-SR04 ultrasonic sensors
- MPU6050 IMU sensor
- L298N motor driver
- DC gear motors and chassis
- Suitable power supplies
- Jumper wires and voltage-divider components

## Software Requirements

- QNX SDP 8.0
- QNX Momentics IDE
- C compiler and QNX build tools
- GPIO interface
- I2C interface

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

## Module Description

| File | Description |
|---|---|
| `main.c` | Main application and sensor-processing loop |
| `aeb_logic.c` | AEB state management and collision-risk logic |
| `hcsr04.c` | Ultrasonic sensor distance measurement |
| `mpu6050.c` | MPU6050 communication and data acquisition |
| `rpi_gpio.c` | GPIO initialization and control |
| `aeb_logic.h` | AEB logic declarations |
| `hcsr04.h` | Ultrasonic sensor declarations |
| `mpu6050.h` | MPU6050 interface declarations |
| `rpi_gpio.h` | GPIO interface declarations |
| `Makefile` | Build configuration |

## System Architecture

The system consists of three main layers:

1. **Sensor Layer:** Acquires obstacle-distance measurements from HC-SR04 sensors and motion-related data from the MPU6050.
2. **Processing Layer:** Validates sensor readings, processes inputs, and evaluates collision risk using the AEB decision logic.
3. **Control Layer:** Uses GPIO signals to control the L298N motor driver according to the current AEB state.

QNX provides the real-time operating environment for sensor processing, task execution, and control operations.

## Working Principle

1. Initialize GPIO, ultrasonic sensors, and the MPU6050.
2. Continuously acquire obstacle-distance measurements.
3. Validate sensor readings and detect sensor faults.
4. Process available IMU data.
5. Evaluate collision risk using configured distance thresholds.
6. Classify the current condition as Safe, Warning, or Emergency.
7. Generate appropriate motor-control commands based on the AEB state.
8. Monitor and log sensor readings, system states, and detected faults.

## AEB Decision States

| State | Description |
|---|---|
| Safe | No immediate obstacle-related braking condition is detected. |
| Warning | An obstacle is approaching the configured warning threshold. |
| Emergency | The emergency threshold is reached and the controller initiates its braking response. |
| Sensor Fault | Invalid or unavailable sensor data is detected, requiring appropriate fail-safe handling. |

*The actual thresholds and transitions depend on the implementation in `aeb_logic.c`.*

## Build and Execution

### 1. Open the Project

Import `AEB_Sensor_Test_Clean` into QNX Momentics IDE.

### 2. Configure the Target

Configure the project for the Raspberry Pi 4 running the appropriate QNX SDP 8.0 AArch64 environment.

### 3. Build the Project

Build the project using the configured Makefile and resolve any compilation or linking errors.

### 4. Connect the Hardware

Verify the connections for the ultrasonic sensors, MPU6050, GPIO pins, and L298N motor driver.

### 5. Deploy and Run

Deploy the generated executable to the Raspberry Pi 4 and run it with the required GPIO and I2C interfaces available.

### 6. Monitor the Output

Observe sensor readings, AEB state transitions, fault messages, and motor-control behavior.

*Exact build and execution commands depend on the project's QNX Momentics configuration.*

## Testing and Validation

The prototype can be evaluated using the following tests:

- **Obstacle Detection:** Verify ultrasonic distance measurements at different distances.
- **Sensor Validation:** Test the controller's response to invalid or unavailable sensor readings.
- **AEB State Transitions:** Verify Safe, Warning, and Emergency state changes.
- **Motor Control:** Check motor responses to the controller's GPIO commands.
- **Response Latency:** Measure the time between obstacle detection and the generation of the braking command.
- **IMU Communication:** Verify successful MPU6050 initialization and data acquisition.

## Future Enhancements

- Implement Time-to-Collision (TTC) estimation using relative-motion data.
- Improve sensor fusion and obstacle tracking.
- Add watchdog monitoring and recovery mechanisms.
- Measure worst-case execution time and response latency.
- Implement structured event logging and performance analysis.
- Integrate vehicle-speed estimation.
- Add additional sensors for improved obstacle detection.
- Evaluate the system through hardware-in-the-loop testing.

## Applications

- Automotive safety system prototyping
- Real-time embedded systems development
- Collision detection and avoidance research
- QNX RTOS application development
- Sensor integration and GPIO control
- Hardware-in-the-loop experimentation

## Technologies Used

- **Programming Language:** C
- **Operating System:** QNX SDP 8.0
- **Development Environment:** QNX Momentics IDE
- **Hardware Platform:** Raspberry Pi 4
- **Sensors:** HC-SR04, MPU6050
- **Motor Driver:** L298N
- **Interfaces:** GPIO, I2C


## Disclaimer

This project is intended for educational and research purposes only. It is an experimental prototype and is not a certified automotive safety system. Do not rely on it for real-world vehicle braking or collision prevention.
