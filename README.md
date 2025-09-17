# Self-Balancing Robot with ESP32 and MPU6050

A comprehensive self-balancing robot implementation using ESP32 NodeMCU-32S board with GY-521 MPU6050 module, featuring FreeRTOS multi-tasking architecture and DMP (Digital Motion Processor) sensor fusion.

## 🚀 Features

- **FreeRTOS Multi-tasking Architecture**: Separate tasks for sensor reading, control, motor control, and monitoring
- **MPU6050 DMP Mode**: Uses Digital Motion Processor for accurate attitude estimation
- **PID Control System**: Dual PID controllers for angle and speed control
- **Real-time Sensor Fusion**: Quaternion-based attitude estimation with gravity compensation
- **Safety Features**: Emergency stop and angle limits
- **Modular Design**: Easy to extend and modify

## 📋 Hardware Requirements

### Main Components
- **ESP32 NodeMCU-32S** development board
- **GY-521 MPU6050** IMU module
- **Motor Driver** (L298N, TB6612FNG, or similar)
- **2x DC Motors** with encoders (optional)
- **Battery Pack** (7.4V - 12V recommended)
- **Robot Chassis** (3D printed or custom)

### Pin Connections

| ESP32 Pin | MPU6050 Pin | Motor Driver |
|-----------|-------------|--------------|
| GPIO 21   | SDA         | -            |
| GPIO 22   | SCL         | -            |
| GPIO 18   | -           | Left PWM     |
| GPIO 19   | -           | Left DIR     |
| GPIO 16   | -           | Right PWM    |
| GPIO 17   | -           | Right DIR    |

## 🏗️ Software Architecture

### FreeRTOS Task Structure

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Sensor Task   │───▶│  Control Task   │───▶│   Motor Task    │
│   (Priority 5)  │    │  (Priority 4)   │    │  (Priority 3)   │
│  200Hz reading  │    │  100Hz control  │    │  100Hz update   │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                       │                       │
         ▼                       ▼                       ▼
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│  MPU6050 DMP    │    │  PID Controllers│    │  Motor Drivers  │
│  Sensor Fusion  │    │  Angle & Speed  │    │  PWM Control    │
└─────────────────┘    └─────────────────┘    └─────────────────┘
                                │
                                ▼
                       ┌─────────────────┐
                       │  Monitor Task   │
                       │  (Priority 2)   │
                       │   1Hz logging   │
                       └─────────────────┘
```

### Key Components

1. **Sensor Task**: Reads MPU6050 data using DMP mode
2. **Control Task**: Implements PID control algorithms
3. **Motor Task**: Controls motor speeds via PWM
4. **Monitor Task**: Logs system status and debugging info

## 🔧 Configuration

### PID Parameters (Tune for your robot)

```cpp
// Angle control (primary balancing)
#define KP_ANGLE 50.0f    // Proportional gain
#define KI_ANGLE 0.0f     // Integral gain  
#define KD_ANGLE 1.0f     // Derivative gain

// Speed control (optional, for forward/backward movement)
#define KP_SPEED 0.5f     // Proportional gain
#define KI_SPEED 0.0f     // Integral gain
#define KD_SPEED 0.0f     // Derivative gain
```

### Safety Limits

```cpp
#define MAX_MOTOR_SPEED 255           // Maximum PWM value
#define MAX_ANGLE_DEVIATION 30.0f     // Normal operation limit
#define EMERGENCY_STOP_ANGLE 45.0f    // Emergency stop threshold
```

## 🚀 Getting Started

### 1. Hardware Setup

1. **Connect MPU6050**:
   - VCC → 3.3V
   - GND → GND
   - SDA → GPIO 21
   - SCL → GPIO 22

2. **Connect Motor Driver**:
   - Connect motors to motor driver
   - Connect PWM and DIR pins to ESP32
   - Connect power supply

3. **Mount Components**:
   - Securely mount MPU6050 on robot chassis
   - Ensure MPU6050 is level when robot is balanced
   - Mount motors and wheels

### 2. Software Setup

1. **Install PlatformIO** (if not already installed)

2. **Clone/Download** this project

3. **Build and Upload**:
   ```bash
   pio run -t upload
   ```

4. **Monitor Output**:
   ```bash
   pio device monitor
   ```

### 3. Calibration

The system automatically calibrates the MPU6050 on startup. You'll see:
```
I (xxxx) MPU6050: Calibrating sensors...
I (xxxx) MPU6050: Calibration complete!
```

## 🎛️ Tuning Guide

### Step 1: Basic Angle Control
1. Start with **KP_ANGLE = 20.0f**
2. Gradually increase until robot shows response
3. Add **KD_ANGLE = 0.5f** to reduce oscillations
4. Fine-tune for stability

### Step 2: Speed Control (Optional)
1. Add encoders to measure wheel speed
2. Implement speed feedback in `balance_control()`
3. Tune **KP_SPEED** for smooth movement

### Step 3: Safety Tuning
1. Adjust **EMERGENCY_STOP_ANGLE** based on your robot's limits
2. Set **MAX_ANGLE_DEVIATION** for normal operation range

## 📊 Data Flow

### Sensor Data Structure
```cpp
typedef struct {
    float yaw, pitch, roll;        // Euler angles (degrees)
    float gyro_x, gyro_y, gyro_z;  // Gyroscope (deg/s)
    float accel_x, accel_y, accel_z; // Accelerometer (g)
    uint32_t timestamp;             // System timestamp
} sensor_data_t;
```

### Control Data Structure
```cpp
typedef struct {
    float angle_output;             // PID angle output
    float speed_output;             // PID speed output
    float left_motor_speed;         // Left motor PWM (-255 to 255)
    float right_motor_speed;        // Right motor PWM (-255 to 255)
    uint32_t timestamp;             // System timestamp
} control_data_t;
```

## 🔍 Debugging

### Monitor Output
The system provides detailed logging:
```
I (xxxx) MONITOR: Pitch: 2.34°, Roll: -1.23°, Left: 45.2, Right: 43.8
```

### Common Issues

1. **Robot falls immediately**:
   - Check MPU6050 orientation
   - Verify motor connections
   - Increase KP_ANGLE gradually

2. **Oscillations**:
   - Reduce KP_ANGLE
   - Increase KD_ANGLE
   - Check for mechanical issues

3. **No response**:
   - Verify I2C connection
   - Check motor driver power
   - Ensure proper calibration

## 🛠️ Customization

### Adding Features

1. **Bluetooth Control**:
   - Add Bluetooth task
   - Implement command parsing
   - Modify control task to accept external commands

2. **Web Interface**:
   - Add WiFi task
   - Create web server
   - Real-time parameter tuning

3. **Data Logging**:
   - Add SD card support
   - Log sensor data for analysis
   - Export for MATLAB/Python analysis

### Motor Driver Support

The code includes placeholder functions for motor control. Implement based on your motor driver:

```cpp
void set_motor_speed(int motor, float speed) {
    // Implement PWM control for your specific motor driver
    // Examples: L298N, TB6612FNG, DRV8833, etc.
}
```

## 📚 Technical Details

### MPU6050 DMP Mode
- **Sample Rate**: 200Hz sensor reading
- **Fusion Algorithm**: Quaternion-based sensor fusion
- **Output**: Direct Euler angles and quaternions
- **Calibration**: Automatic accelerometer and gyroscope calibration

### PID Control
- **Angle Control**: Primary balancing control
- **Speed Control**: Optional forward/backward movement
- **Anti-windup**: Integral term limiting
- **Derivative Filtering**: Prevents noise amplification

### FreeRTOS Features
- **Task Priorities**: Sensor > Control > Motor > Monitor
- **Inter-task Communication**: Queues and semaphores
- **Real-time Scheduling**: Deterministic timing
- **Memory Management**: Stack size optimization

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly
5. Submit a pull request

## 📄 License

This project is open source. Feel free to use, modify, and distribute.

## 🙏 Acknowledgments

- **Jeff Rowberg** for the excellent MPU6050 library
- **ESP-IDF** team for the robust FreeRTOS implementation
- **InvenSense** for the MPU6050 sensor and DMP technology

## 📞 Support

For questions and support:
- Check the debugging section above
- Review the code comments
- Open an issue on GitHub

---

**Happy Balancing! 🤖⚖️**
