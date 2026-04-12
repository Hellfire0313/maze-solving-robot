<p align="center">
  <strong>NAVIX-4</strong><br/>
  <em>Autonomous Maze-Solving Robot</em>
</p>

<p align="center">
  <a href="https://en.wikipedia.org/wiki/MIT_License"><img src="https://img.shields.io/badge/license-MIT-blue.svg" alt="License"/></a>
  <img src="https://img.shields.io/badge/platform-Teensy_4.1-orange.svg" alt="Platform"/>
  <img src="https://img.shields.io/badge/patent-IN_202521042150-green.svg" alt="Patent"/>
  <img src="https://img.shields.io/badge/competition-Robofest_4.0-red.svg" alt="Competition"/>
</p>

---

A 10×10 cm autonomous robot that **explores, learns, and optimizes** maze paths in real time. Built for [Robofest 4.0](https://robofest.co.in/) (National Level, Gujarat), it fuses three time-of-flight distance sensors with a 9-axis IMU to navigate unknown mazes, record every decision, eliminate dead ends via string-based path optimization, and replay the shortest route at full speed.

**[Indian Patent Published](https://iprsearch.ipindia.gov.in/) — Application No. 202521042150**

---

## How It Works

```
┌─────────────────────────────────────────────────────────┐
│                    SENSOR LAYER                         │
│  ┌──────────┐   ┌──────────┐   ┌──────────┐            │
│  │ VL53L1X  │   │ VL53L1X  │   │ VL53L1X  │            │
│  │  LEFT    │   │  FRONT   │   │  RIGHT   │            │
│  └────┬─────┘   └────┬─────┘   └────┬─────┘            │
│       └───────────┬───┴───┬──────────┘                  │
│             ┌─────┴─────┐                               │
│             │ TCA9548A  │  I²C Multiplexer               │
│             │  (0x70)   │                               │
│             └─────┬─────┘                               │
│                   │ I²C                                 │
│  ┌────────────────┼────────────────────┐                │
│  │          TEENSY 4.1                 │                │
│  │   ┌─────────────────────────────┐   │                │
│  │   │  Decision Engine            │   │                │
│  │   │  ┌────────┐  ┌──────────┐   │   │   ┌─────────┐ │
│  │   │  │ LSRB / │  │ Path     │   │   │   │ BMX160  │ │
│  │   │  │ RSLB   │  │ Optimize │   │   │◄──┤ 9-axis  │ │
│  │   │  └────────┘  └──────────┘   │   │   │ IMU     │ │
│  │   └─────────────────────────────┘   │   └─────────┘ │
│  └───────────┬─────────────┬───────────┘                │
│              │             │                            │
│         ┌────┴────┐   ┌────┴────┐                       │
│         │  HW-121 │   │  HW-121 │  Dual H-Bridge        │
│         │  Motor  │   │  Motor  │                       │
│         │ Driver  │   │ Driver  │                       │
│         └────┬────┘   └────┬────┘                       │
│         ┌────┴────┐   ┌────┴────┐                       │
│         │  N20 L  │   │  N20 R  │  DC Gear Motors        │
│         └─────────┘   └─────────┘                       │
└─────────────────────────────────────────────────────────┘
```

### Phase 1 — Explore

The robot enters the maze blind. At every junction it reads left/front/right distances and follows the selected priority rule:

| Algorithm | Priority Order | Use Case |
|-----------|---------------|----------|
| **LSRB** | Left → Straight → Right → Back | Left-wall-hugging traversal |
| **RSLB** | Right → Straight → Left → Back | Right-wall-hugging traversal |

Every turn is appended to a path string (`L`, `S`, `R`, `U`). During forward segments, the robot uses **wall-following PID** (when walls are visible) or **magnetometer heading lock** (in open corridors) to stay centered.

### Phase 2 — Optimize

Dead-end U-turns are collapsed using iterative string replacement:

```
LUL → S    RUR → S    SUS → U
LUR → U    RUL → U
SUR → L    RUS → L
SUL → R    LUS → R
```

This reduces a path like `LLULSURRS` into the shortest equivalent without dead-end detours.

### Phase 3 — Replay

The optimized path is executed turn-by-turn at full speed — no exploration, no hesitation.

---

## Hardware

| Component | Part | Role |
|-----------|------|------|
| **MCU** | Teensy 4.1 (ARM Cortex-M7, 600 MHz) | Core controller |
| **Distance** | 3× VL53L1X Time-of-Flight | Left / Front / Right ranging |
| **IMU** | BMX160 (mag + gyro + accel) | Absolute heading & compass calibration |
| **Mux** | TCA9548A I²C Multiplexer | Addresses 3 identical-address ToF sensors |
| **Drive** | 2× N20 DC gear motors + HW-121 H-bridge | Differential drive |
| **PCB** | Custom NAVIX-4 (stacked design) | Fits within 10×10 cm footprint |

### Pin Map

| Pin | Function | Pin | Function |
|-----|----------|-----|----------|
| 2 | Left motor PWM | 5 | Right motor PWM |
| 3, 4 | Left motor DIR | 6, 7 | Right motor DIR |
| 35 | Reset | 36 | Optimize path |
| 37 | Start optimized run | 38 | RSLB select |
| 39 | LSRB select | 40 | Calibration |

---

## Robot

<table>
  <tr>
    <td align="center"><img src="media/Perspective View.jpg" width="250"/><br/><sub>Perspective</sub></td>
    <td align="center"><img src="media/Top View.jpg" width="250"/><br/><sub>Top</sub></td>
    <td align="center"><img src="media/Front View.jpg" width="250"/><br/><sub>Front (ToF sensor array)</sub></td>
  </tr>
  <tr>
    <td align="center"><img src="media/Back View.jpg" width="250"/><br/><sub>Back</sub></td>
    <td align="center"><img src="media/Left Hand View.jpg" width="250"/><br/><sub>Left</sub></td>
    <td align="center"><img src="media/Right Hand View.jpg" width="250"/><br/><sub>Right</sub></td>
  </tr>
  <tr>
    <td align="center"><img src="media/Bottom View.jpg" width="250"/><br/><sub>Bottom — NAVIX-4 PCB</sub></td>
    <td align="center"><img src="media/Test Maze.jpg" width="250"/><br/><sub>Competition arena</sub></td>
  </tr>
</table>

### PCB Schematics

| Top Layer | Bottom Layer |
|-----------|-------------|
| ![Top](schematics/Upper%20Layer.jpg) | ![Bottom](schematics/Bottom%20Layer.jpg) |

---

## Patent

| | |
|---|---|
| **Application** | 202521042150 |
| **Title** | Autonomous Robot for Real-Time Pathfinding and Obstacle Avoidance |
| **Filed / Published** | 01 May 2025 / 23 May 2025 |
| **Institution** | Birla Vishvakarma Mahavidyalaya Engineering College, Gujarat |
| **Journal** | Patent Office Journal No. 21/2025 |

Full publication available in [`Patent/`](./Patent/).

---

## Quick Start

```bash
# 1. Clone
git clone https://github.com/<your-username>/maze-solving-robot.git

# 2. Open in Arduino IDE
#    File → Open → src/Project-final.ino

# 3. Install dependencies (Library Manager)
#    - DFRobot BMX160
#    - Adafruit VL53L1X

# 4. Select board: Teensy 4.1  →  Upload
```

**Calibration:** Power on → press calibration button facing North → rotate to West, South, East, pressing at each cardinal point.

---

## Project Structure

```
maze-solving-robot/
├── src/
│   └── Project-final.ino    # Complete firmware (sensor fusion, algorithms, motor control)
├── media/                    # Robot photos
├── schematics/               # PCB layer images
├── Patent/                   # Published patent document
├── docs/                     # GitHub Pages site
│   └── index.html            # Project landing page
│   
└── README.md
```

---

## Dependencies

| Library | Purpose |
|---------|---------|
| [`DFRobot_BMX160`](https://github.com/DFRobot/DFRobot_BMX160) | 9-axis IMU (magnetometer for heading) |
| [`Adafruit_VL53L1X`](https://github.com/adafruit/Adafruit_VL53L1X) | Time-of-flight distance sensing |
| `Wire.h` | I²C bus communication |
| Arduino / [Teensyduino](https://www.pjrc.com/teensy/teensyduino.html) | Framework + board support |

---

## Author

**[Anshul Majmudar](https://linkedin.com/in/Anshul-Majmudar)**
Electronics & Communication Engineering, BVM Gujarat
Competed at Robofest 4.0 — National Robotics Competition

---

## License

[MIT](LICENSE)
