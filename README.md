# Maze Solving Robot — Robofest 4.0

An autonomous maze-solving robot built for **Robofest 4.0**, a national-level robotics competition. The robot navigates mazes using real-time sensor fusion, compass-based orientation, and two switchable traversal algorithms — with path optimization for the return run.

---

## Robot Photos

<table>
  <tr>
    <td align="center"><img src="media/Perspective View.jpg" width="250"/><br/>Perspective view</td>
    <td align="center"><img src="media/Top View.jpg" width="250"/><br/>Top view</td>
    <td align="center"><img src="media/Front View.jpg" width="250"/><br/>Front view (ToF sensor)</td>
  </tr>
  <tr>
    <td align="center"><img src="media/Back View.jpg" width="250"/><br/>Back view</td>
    <td align="center"><img src="media/Left Hand View.jpg" width="250"/><br/>Left side</td>
    <td align="center"><img src="media/Right Hand View.jpg" width="250"/><br/>Right side</td>
  </tr>
  <tr>
    <td align="center"><img src="media/Bottom View.jpg" width="250"/><br/>Bottom PCB (NAVIX-4)</td>
    <td align="center"><img src="media/Test Maze.jpg" width="250"/><br/>Competition maze arena</td>
  </tr>
</table>

---

## Demo / Competition

Built and competed at **Robofest 4.0** (National Level Competition)

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

During forward movement, motor speeds are dynamically adjusted based on lateral wall distances to keep the robot centered.

---

## Hardware

| Component | Description |
|---|---|
| **Microcontroller** | Teensy 4.1 |
| **Motors** | 2x N20 DC Gear Motors |
| **Motor Driver** | HW-121 (dual H-bridge) |
| **Distance Sensors** | 3x VL53L1X Time-of-Flight (Left, Front, Right) |
| **IMU / Compass** | BMX160 (magnetometer + gyro + accelerometer) |
| **Multiplexer** | TCA9548A I2C Mux |

---

## PCB Schematics

| Top Layer | Bottom Layer |
|---|---|
| ![Top Layer](schematics/Upper%20Layer.jpg) | ![Bottom Layer](schematics/Bottom%20Layer.jpg) |

---

## Patent

This project is protected under a published Indian patent.

| Field | Details |
|---|---|
| **Application No.** | 202521042150 |
| **Title** | Autonomous Robot for Real-Time Pathfinding and Obstacle Avoidance |
| **Filed** | 01 May 2025 |
| **Published** | 23 May 2025 |
| **Institution** | Birla Vishvakarma Mahavidyalaya Engineering College, Gujarat |
| **Journal** | Patent Office Journal No. 21/2025 |

The full patent publication is available in the [Patent/](./Patent/) folder of this repository.

---

## Software and Libraries

- `DFRobot_BMX160` — magnetometer/IMU readings
- `Adafruit_VL53L1X` — ToF sensor ranging
- `Wire.h` — I2C communication
- Arduino/Teensy framework (C++)

---

## Features

- Real-time wall-following with dynamic motor correction
- Absolute compass heading using BMX160 magnetometer
- Compass calibration via physical button (North to West to South to East)
- Switchable LSRB / RSLB maze algorithms at runtime
- Path recording and string-based optimization
- Optimized path replay after maze is solved
- System reset button to restart exploration

---

## Getting Started

### Prerequisites

- Arduino IDE or PlatformIO
- Teensyduino add-on for Teensy 4.1 support
- Install libraries via Library Manager: DFRobot BMX160 and Adafruit VL53L1X

### Upload

1. Open `src/Project-final.ino` in Arduino IDE
2. Select Board: Teensy 4.1
3. Connect via USB and click Upload

---

## Pin Reference

| Pin | Function |
|---|---|
| 5 | Right motor PWM |
| 6 / 7 | Right motor direction |
| 2 | Left motor PWM |
| 3 / 4 | Left motor direction |
| 35 | Reset button |
| 36 | Optimize path button |
| 37 | Start optimized path button |
| 38 | RSLB algorithm select |
| 39 | LSRB algorithm select |
| 40 | Calibration button |

---

## Author

**Anshul Majmudar**
Competed at Robofest 4.0 — National Robotics Competition, Gujarat

---

## License

This project is open source under the [MIT License](LICENSE).
