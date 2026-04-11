# Maze Solving Robot — Robofest 4.0

An autonomous maze-solving robot built for **Robofest 4.0**, a national-level robotics competition. The robot navigates mazes using real-time sensor fusion, compass-based orientation, and two switchable traversal algorithms — with path optimization for the return run.

---

## Demo / Competition

> Built and competed at **Robofest 4.0** (National Level Competition)

---

## How It Works

The robot uses three Time-of-Flight (ToF) distance sensors to detect open paths (left, front, right) and a magnetometer for absolute heading. It solves the maze on the first run, records every turn, then optimizes the path and runs it back at full speed.

### Algorithms
- **LSRB** (Left-Straight-Right-Back) — Left-priority traversal
- **RSLB** (Right-Straight-Left-Back) — Right-priority traversal
- Switchable at runtime via physical buttons

### Path Optimization
After reaching the end, dead-ends (U-turns) are collapsed using string replacement rules:

```
LUL → S     RUR → S
LUR → U     RUL → U
SUR → L     RUS → L
SUL → R     LUS → R
SUS → U
```

### Wall Following
During forward movement, the robot dynamically adjusts motor speeds based on lateral wall distances, keeping itself centered between walls (or tracking a single wall when only one is present).

---

## Hardware

| Component | Description |
|---|---|
| **Microcontroller** | Teensy 4.1 |
| **Motors** | 2× N20 DC Gear Motors |
| **Motor Driver** | HW-121 (dual H-bridge) |
| **Distance Sensors** | 3× VL53L1X Time-of-Flight (Left, Front, Right) |
| **IMU / Compass** | BMX160 (magnetometer + gyro + accelerometer) |
| **Multiplexer** | TCA9548A I²C Mux (for multiple ToF sensors on same bus) |

---

## Software & Libraries

- `DFRobot_BMX160` — magnetometer/IMU readings
- `Adafruit_VL53L1X` — ToF sensor ranging
- `Wire.h` — I²C communication
- Arduino/Teensy framework (C++)

---

## Features

- Real-time wall-following with dynamic motor correction
- Absolute compass heading using BMX160 magnetometer
- Compass calibration via physical button (North → West → South → East)
- Switchable LSRB / RSLB maze algorithms at runtime
- Path recording and string-based optimization
- Optimized path replay after maze is solved
- System reset button to restart exploration

---

## Project Structure

```
maze-solving-robot/
├── maze_solver.ino     # Main Arduino sketch (all logic)
└── README.md
```

---

## Getting Started

### Prerequisites
- [Arduino IDE](https://www.arduino.cc/en/software) or [PlatformIO](https://platformio.org/)
- Teensyduino add-on for Teensy 4.1 support
- Install libraries via Library Manager:
  - `DFRobot BMX160`
  - `Adafruit VL53L1X`

### Upload
1. Open `maze_solver.ino` in Arduino IDE
2. Select **Board: Teensy 4.1**
3. Connect via USB and click **Upload**

### Calibration (First Run)
1. Place robot facing **North**
2. Press the **Calibration button** (pin 40)
3. Follow serial prompts — rotate to West → South → East, pressing the button at each

---

## Pin Reference

| Pin | Function |
|---|---|
| 5 | Right motor PWM (PWMA) |
| 6 / 7 | Right motor direction (AIN1/AIN2) |
| 2 | Left motor PWM (PWMB) |
| 3 / 4 | Left motor direction (BIN1/BIN2) |
| 35 | Reset button |
| 36 | Optimize path button |
| 37 | Start optimized path button |
| 38 | RSLB algorithm select |
| 39 | LSRB algorithm select |
| 40 | Calibration button |

---

## Built With

- C++ / Arduino framework
- Teensy 4.1 microcontroller
- I²C sensor bus with TCA9548A multiplexer

---

## Author

**Anshul Majmudar**
Competed at Robofest 4.0 — National Robotics Competition

---

## Patent

This project is protected under a published patent.
**Patent No.:** 202521042150
**Title:** Autonomous Robot for Real-Time Pathfinding and Obstacle Avoidance
**Filed:**  01 May 2025 | **Published:** 23 May 2025

---

## License

This project is open source under the [MIT License](LICENSE).
