# TeaMate - Automated Tea Steeping Robot 🫖🤖

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Arduino](https://img.shields.io/badge/Arduino-Compatible-blue.svg)](https://www.arduino.cc/)
[![Platform](https://img.shields.io/badge/Platform-MegaPi-green.svg)](https://www.makeblock.com/steam-kits/megapi)

An intelligent robotic arm system that automates the tea steeping process. TeaMate grabs a tea bag, steeps it in your cup for a precise duration, performs gentle agitation during steeping, and automatically disposes of the used tea bag.

**Authors:** Tristan Dobrowney & Evan Evaniuk  
**Date:** March 2025  
**Version:** 1.0

---

## 📋 Table of Contents

- [Features](#-features)
- [Hardware Requirements](#-hardware-requirements)
- [Software Dependencies](#-software-dependencies)
- [System Architecture](#-system-architecture)
- [Installation](#-installation)
- [Usage](#-usage)
- [Configuration](#-configuration)
- [Project Structure](#-project-structure)
- [State Machine](#-state-machine)
- [Calibration](#-calibration)
- [Troubleshooting](#-troubleshooting)
- [Future Enhancements](#-future-enhancements)
- [License](#-license)

---

## ✨ Features

- **Automated Tea Steeping**: Complete hands-free tea preparation from grab to disposal
- **Precision Timing**: User-adjustable steep duration up to 99:59 (mm:ss)
- **Intelligent Movement**: 3DOF robotic arm with coordinated joint control
- **Active Steeping**: Gentle in-and-out motion during steeping for optimal flavor extraction
- **Visual Feedback**: OLED display with real-time status and animated graphics
- **Audio Alerts**: Musical tones notify when tea is ready
- **Safety Features**: 
  - Automatic calibration with limit detection
  - Joint angle constraints to prevent collisions
  - Emergency stop capability
- **User-Friendly Interface**: Rotary encoder for easy time adjustment

---

## 🔧 Hardware Requirements

### Core Components

- **Controller**: Makeblock MegaPi (Arduino Mega 2560 compatible)
- **Motors**: 3x MeEncoderOnBoard motors (SLOT1, SLOT2, SLOT3)
- **End Effector**: MeMegaPiDCMotor claw gripper (PORT4A)
- **Sensors**:
  - MeLineFollower sensor (PORT_5) - Base calibration
  - MeUltrasonicSensor (PORT_8) - Cup detection
  - 3x Rotary encoders (integrated with motors)

### User Interface

- **Display**: SH1106 128x64 OLED (I2C)
- **Input**: Rotary encoder with push button
  - CLK: Pin 29
  - DT: Pin 26
  - SW: Pin 27
- **Indicators**:
  - Red LED (Pin 30) - Setup mode
  - Yellow LED (Pin 39) - Steeping/Calibration
  - Green LED (Pin 28) - Complete
- **Audio**: Passive buzzer (Pin 24)

### Mechanical Specifications

- **Base Rotation**: 360° continuous (detected via line follower)
- **Shoulder Joint**: ~180° range
- **Elbow Joint**: ~180° range
- **Gear Ratios**:
  - Base: 9:1 (72/8 teeth)
  - Arm joints: 7:1 (56/8 teeth)
- **Workspace**: Approximately 460mm reach

---

## 📚 Software Dependencies

### Required Libraries

Install via Arduino Library Manager:

```cpp
MeMegaPi         // Makeblock MegaPi library
U8g2lib          // OLED display driver (v2.35.x)
```

### Custom Headers (Included)

- `motor_structure.h` - Motor configuration and state structures
- `TeaMate_Tones.h` - Audio feedback system
- `MeMegaPiWrapper.h` - Library compatibility wrapper

---

## 🏗️ System Architecture

### Movement Sequences

The robot follows pre-programmed movement sequences defined as arrays of `MovementCommand` structures:

```cpp
struct MovementCommand {
  float base;      // Base rotation angle (degrees)
  float shoulder;  // Shoulder angle (degrees)
  float elbow;     // Elbow angle (degrees)
  float speed;     // Movement speed (RPM)
};
```

**Key Sequences:**
1. **NeutralToTeaBag**: 5 waypoints to approach and grasp tea bag
2. **TeaBagToCup**: 2 positions for lowering bag into cup
3. **CupToTrash**: 3 waypoints to dispose of used tea bag

### Control System

- **Motor Control**: Callback-based position control with PID
- **State Management**: Finite state machine with 7 states
- **Timing**: Non-blocking delays using `millis()` for responsive control
- **Safety**: Joint limit enforcement and stall detection

---

## 🚀 Installation

### 1. Clone Repository

```bash
git clone https://github.com/yourusername/teamate.git
cd teamate
```

### 2. Install Dependencies

Open Arduino IDE and install:
- **MeMegaPi** library from Library Manager
- **U8g2** library from Library Manager

### 3. Configure Hardware

- Connect all motors to correct slots (SLOT1-3)
- Attach sensors to specified ports
- Connect OLED via I2C (SDA/SCL)
- Wire LEDs and buzzer to designated pins

### 4. Upload Code

1. Open `TeaMate.ino` in Arduino IDE
2. Select Board: "Arduino Mega 2560"
3. Select correct COM port
4. Upload sketch
5. Power cycle the robot

### 5. Initial Calibration

On first boot, TeaMate will automatically:
1. Calibrate base rotation using line follower
2. Detect shoulder joint limits
3. Detect elbow joint limits
4. Move to neutral position
5. Open claw and enter setup mode

---

## 💡 Usage

### Basic Operation

1. **Setup Mode** (Red LED):
   - Rotate encoder to adjust steep time (0:00 to 99:59)
   - Press encoder button to confirm

2. **Place Cup** (Yellow LED):
   - Position cup on detection tray
   - System automatically detects cup presence

3. **Steeping** (Yellow LED):
   - Robot grabs tea bag and places in cup
   - Performs gentle agitation every few seconds
   - Countdown displays remaining time

4. **Complete** (Green LED):
   - Musical alert sounds
   - Tea bag automatically disposed in trash
   - Animated "Tea is served" display
   - Place cup to reset for next use

### Advanced Features

- **Emergency Stop**: Implemented via `emergency_stop()` function
- **Manual Home**: Returns all joints to neutral position
- **Debug Mode**: Set `DEBUG 1` in code for serial output

---

## ⚙️ Configuration

### Timing Adjustments

Edit in code:

```cpp
const byte DEFAULT_TIME = 10;              // Default steep time (seconds)
const int MAX_TIMER_VALUE = 99 * 60 + 59; // Maximum time (99:59)
const int COUNTDOWN_INTERVAL = 1000;       // Timer update rate (ms)
```

### Movement Speeds

```cpp
const float TEABAG_MOVE_SPEED = 35;  // Main movement speed (RPM)
```

Adjust individual sequence speeds in movement command arrays.

### Claw Settings

```cpp
claw claw = {
  MeMegaPiDCMotor(PORT4A),
  CLAW_CLOSED,
  200,    // Open speed (0-255)
  200,    // Close speed (0-255)
  2000    // Movement time (ms)
};
```

### PID Tuning

Motor PID parameters in configuration structs:

```cpp
const motor_config BASE_CONFIG = {
  // ... other params
  7.5f,   // position_Kp
  2.5f,   // position_Ki
  1.0f,   // position_Kd
  0.5f,   // speed_Kp
  0.0f,   // speed_Ki
  0.2f    // speed_Kd
};
```

---

## 📁 Project Structure

```
teamate/
├── TeaMate.ino              # Main program file
├── motor_structure.h        # Motor control structures
├── TeaMate_Tones.h         # Sound generation functions
├── MeMegaPiWrapper.h       # Library compatibility wrapper
├── README.md               # This file
└── LICENSE                 # MIT License
```

### Unused Development Files

The following files were part of earlier development but are not currently used:
- `inverse_kinematics.h/cpp` - Jacobian-based IK (future feature)
- `robot_kinematics.h/cpp` - DH parameter FK calculations
- `matrix_operations.h` - Matrix math utilities
- `path_planning.h` - Obstacle avoidance (future feature)
- `trajectory_generator.h` - Motion profiling
- `optimized_jacobian.h` - Efficient Jacobian calculations

---

## 🔄 State Machine

```
SETUP → PLACE_CUP → GET_TEABAG → TEA_IN_CUP → COUNTING_DOWN → TEA_IN_GARBAGE → FINISHED
  ↑                                                                                   ↓
  └───────────────────────────────────────────────────────────────────────────────────┘
```

**States:**
- `SETUP`: User adjusts steep time
- `PLACE_CUP`: Waiting for cup detection
- `GET_TEABAG`: Grabbing tea bag from holder
- `TEA_IN_CUP`: Initial bag placement
- `COUNTING_DOWN`: Active steeping with agitation
- `TEA_IN_GARBAGE`: Disposing used tea bag
- `FINISHED`: Notification and cleanup

---

## 🔧 Calibration

### Automatic Calibration Process

TeaMate performs automatic calibration on startup:

#### Base Motor (Line Follower Method)
1. Rotates to find first black line marker
2. Continues to find second black line marker
3. Calculates working range and neutral position

#### Arm Joints (Stall Detection Method)
1. Moves to lower limit until mechanical stop detected
2. Moves to upper limit until mechanical stop detected
3. Applies safety margin (50 encoder counts)
4. Calculates neutral position at midpoint

### Manual Recalibration

If robot behavior becomes erratic:
1. Power cycle the system
2. Ensure no obstructions in workspace
3. Allow full calibration sequence to complete
4. Check serial monitor for calibration values

### Calibration Parameters

```cpp
const calibration_params JOINT_CALIBRATION = {
  2520,   // travel_limit (max encoder counts)
  70.0f,  // calibration_speed (RPM)
  3,      // stall_threshold (position delta)
  10000,  // timeout (ms)
  1500,   // startup_delay (ms before stall detection)
  150,    // check_interval (ms between checks)
  90.0f   // safe_position (degrees)
};
```

---

## 🐛 Troubleshooting

### Common Issues

**Problem**: Robot doesn't move after startup  
**Solution**: Check that calibration completed successfully. Look for "System calibration complete!" in serial monitor.

**Problem**: Claw doesn't grip tea bag  
**Solution**: Adjust `claw.move_time` or `claw.close_speed` parameters. Verify mechanical alignment.

**Problem**: Tea bag misses cup  
**Solution**: Verify cup is centered on detection platform. Adjust movement commands if needed.

**Problem**: Motors jitter during movement  
**Solution**: Review PID parameters. Reduce movement speeds. Check for mechanical binding.

**Problem**: Display shows corrupted text  
**Solution**: Check I2C connections. Verify OLED display address (0x3C default).

**Problem**: Ultrasonic sensor doesn't detect cup  
**Solution**: Ensure cup is within 5cm range. Check sensor wiring and PORT_8 connection.

### Serial Debug Output

Enable verbose debugging:

```cpp
#define DEBUG 1
```

Monitor at 115200 baud for:
- Calibration progress
- Motor positions
- State transitions
- Error messages

---

## 🚀 Future Enhancements

### Planned Features

- [ ] **Cartesian Control**: Direct XYZ positioning using inverse kinematics
- [ ] **Obstacle Avoidance**: Path planning around cups and other objects
- [ ] **Multiple Tea Types**: Support for different steep times per tea variety
- [ ] **Water Temperature Monitoring**: Integration with temperature sensor
- [ ] **Cup Size Detection**: Automatic adjustment for different cup heights
- [ ] **Milk/Sugar Addition**: Extended end effector capabilities

### Contributing

Contributions welcome! Please:
1. Fork the repository
2. Create a feature branch
3. Commit changes with clear messages
4. Submit a pull request

---

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

```
MIT License

Copyright (c) 2025 Tristan Dobrowney & Evan Evaniuk

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

---

## 🙏 Acknowledgments

- Makeblock for the MegaPi robotics platform
- U8g2 library developers for excellent OLED support
- The Arduino community for inspiration and support

---

## 📞 Contact

For questions, suggestions, or collaboration:
- **Issues**: [GitHub Issues](https://github.com/yourusername/teamate/issues)
- **Email**: your.email@example.com

---

**Enjoy your perfectly steeped tea! ☕**
