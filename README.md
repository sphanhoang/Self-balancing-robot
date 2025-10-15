# Self-Balancing Robot with ESP32 and MPU6050

A comprehensive self-balancing robot implementation using ESP32 NodeMCU-32S board with GY-521 MPU6050 module, featuring FreeRTOS multi-tasking architecture, DMP (Digital Motion Processor) sensor fusion, and a real-time web interface for parameter tuning.

## 🚀 Features

- **FreeRTOS Multi-tasking Architecture**: Separate tasks for sensor reading, control, motor control, encoder reading, and web server
- **MPU6050 DMP Mode**: Uses Digital Motion Processor for accurate attitude estimation with quaternion-based sensor fusion
- **Dual PID Control System**: Roll (angle) and speed PID controllers with real-time parameter tuning
- **Real-time Web Interface**: Live parameter adjustment, data visualization, and robot control via WiFi
- **Encoder Feedback**: Quadrature encoder support for precise motor speed measurement
- **Safety Features**: Emergency stop, angle limits, and integral windup protection
- **Modular Design**: Easy to extend and modify with clear separation of concerns

## 📋 Hardware Requirements

### Main Components
- **ESP32 NodeMCU-32S** development board
- **GY-521 MPU6050** IMU module
- **Motor Driver** (L298N, TB6612FNG, or similar)
- **2x DC Motors** with quadrature encoders
- **Battery Pack** (7.4V - 12V recommended)
- **Robot Chassis** (3D printed or custom)

### Pin Connections

| ESP32 Pin | Component | Function |
|-----------|-----------|----------|
| GPIO 21   | MPU6050   | SDA (I2C) |
| GPIO 22   | MPU6050   | SCL (I2C) |
| GPIO 14   | Motor Driver | ENA (Right Motor PWM) |
| GPIO 32   | Motor Driver | ENB (Left Motor PWM) |
| GPIO 27   | Motor Driver | IN1 (Right Motor Direction) |
| GPIO 26   | Motor Driver | IN2 (Right Motor Direction) |
| GPIO 25   | Motor Driver | IN3 (Left Motor Direction) |
| GPIO 33   | Motor Driver | IN4 (Left Motor Direction) |
| GPIO 16   | Left Encoder | Channel A |
| GPIO 17   | Left Encoder | Channel B |
| GPIO 18   | Right Encoder | Channel A |
| GPIO 19   | Right Encoder | Channel B |

## 🏗️ Software Architecture

### FreeRTOS Task Structure

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Sensor Task   │───▶│  Control Task   │───▶│   Motor Task    │
│   (Priority 5)  │    │  (Priority 4)   │    │  (Priority 3)   │
│  100Hz reading  │    │  100Hz control  │    │  100Hz update   │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                       │                       │
         ▼                       ▼                       ▼
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│  MPU6050 DMP    │    │  PID Controllers│    │  Motor Drivers  │
│  Sensor Fusion  │    │  Roll & Speed   │    │  PWM Control    │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                       │                       │
         ▼                       ▼                       ▼
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│  Encoder Task   │    │  Web Server     │    │  Monitor Task   │
│  (Priority 3)   │    │  (Priority 2)   │    │  (Priority 1)   │
│  1Hz reading    │    │  WiFi + HTTP    │    │  0.5Hz logging  │
└─────────────────┘    └─────────────────┘    └─────────────────┘
```

### Key Components

1. **Sensor Task**: Reads MPU6050 data using DMP mode with quaternion fusion
2. **Control Task**: Implements dual PID control (roll angle + speed) with web parameter updates
3. **Motor Task**: Controls motor speeds via MCPWM with direction control
4. **Encoder Task**: Reads quadrature encoders and calculates motor speeds
5. **Web Server Task**: Provides real-time web interface for parameter tuning and monitoring
6. **Monitor Task**: Logs system status and debugging information

## 🔧 Configuration

### PID Parameters (Tune for your robot)

```cpp
// Roll control (primary balancing)
#define KP_ROLL 18.0f     // Proportional gain (best results: 18)
#define KI_ROLL 50.0f     // Integral gain (best results: 50)
#define KD_ROLL 0.4f      // Derivative gain (best results: 0.4)

// Speed control (for forward/backward movement)
#define KP_SPEED 0.01f    // Proportional gain
#define KI_SPEED 0.01f    // Integral gain
#define KD_SPEED 0.01f    // Derivative gain
#define TILT_ANGLE 0.6f   // Target tilt angle for speed control
```

### Safety Limits

```cpp
#define MAX_MOTOR_SPEED 255.0f        // Maximum PWM value
#define MIN_MOTOR_SPEED 30.0f         // Minimum PWM value (38.25)
#define MAX_ANGLE 15.0f               // Normal operation limit (degrees)
#define MIN_ANGLE -15.0f              // Normal operation limit (degrees)
#define PWM_FREQ_HZ 100               // PWM frequency
```

### Web Interface Configuration

```cpp
#define CONFIG_WIFI_SSID "your_wifi_ssid"
#define CONFIG_WIFI_PASSWORD "your_wifi_password"
#define WEB_SERVER_PORT 80
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

3. **Configure WiFi** (in `src/web_interface.cpp`):
   ```cpp
   #define CONFIG_WIFI_SSID "your_wifi_ssid"
   #define CONFIG_WIFI_PASSWORD "your_wifi_password"
   ```

4. **Build and Upload**:
   ```bash
   pio run -t upload
   ```

5. **Monitor Output**:
   ```bash
   pio device monitor
   ```

6. **Access Web Interface**:
   - Look for the IP address in the serial monitor output
   - Open your browser and navigate to `http://[robot_ip_address]`

### 3. Calibration

The system uses pre-calibrated MPU6050 offsets. You'll see:
```
I (xxxx) MPU6050: Testing device connections...
I (xxxx) MPU6050: MPU6050 connection successful
I (xxxx) MPU6050: Initializing DMP...
I (xxxx) MPU6050: DMP is ready!
```

## 🎛️ Tuning Guide

### Step 1: Basic Roll Control
1. Start with **KP_ROLL = 15.0f**
2. Gradually increase until robot shows response
3. Add **KD_ROLL = 0.3f** to reduce oscillations
4. Fine-tune for stability

### Step 2: Web Interface Tuning
1. Connect to the robot's web interface
2. Use the real-time sliders to adjust PID parameters
3. Monitor the live data chart for system response
4. Enable/disable balancing with the control button

### Step 3: Speed Control (Optional)
1. The system includes encoder feedback for speed control
2. Tune **KP_SPEED**, **KI_SPEED**, **KD_SPEED** for smooth movement
3. Adjust **TILT_ANGLE** for desired forward/backward tilt

### Step 4: Safety Tuning
1. Adjust **MAX_ANGLE** and **MIN_ANGLE** based on your robot's limits
2. Set appropriate **MIN_MOTOR_SPEED** to prevent stalling
3. Monitor the system through the web interface

## 🌐 Web Interface

The robot includes a comprehensive web interface accessible via WiFi:

### Features
- **Real-time Data Visualization**: Live charts showing angle and PID output
- **Parameter Tuning**: Sliders for adjusting PID parameters in real-time
- **Robot Control**: Enable/disable balancing with a single click
- **Status Monitoring**: Current angle, output values, and system status

### Access
1. Connect to the same WiFi network as configured in the code
2. Open your browser and navigate to the robot's IP address
3. Use the interface to tune parameters and monitor performance

### API Endpoints
- `GET /` - Main web interface
- `GET /data` - JSON data for real-time updates
- `GET /pid?kp_r=X&ki_r=Y&kd_r=Z` - Update PID parameters
- `GET /control?enable=1` - Enable/disable balancing

## 📊 Data Flow

### Sensor Data Structure
```cpp
typedef struct {
    float yaw, pitch, roll;        // Euler angles (degrees)
    float accel_x, accel_y, accel_z; // Linear acceleration (g)
    uint32_t timestamp;             // System timestamp
    float dt_since_last;            // Time delta since last reading
} sensor_data_t;
```

### Control Data Structure
```cpp
typedef struct {
    float roll_output;              // Roll PID output
    float speed_output;             // Speed PID output
    float left_motor_speed;         // Left motor PWM (-255 to 255)
    float right_motor_speed;        // Right motor PWM (-255 to 255)
    float left_motor_feedback;      // Left motor speed feedback (RPS)
    float right_motor_feedback;     // Right motor speed feedback (RPS)
    uint32_t timestamp;             // System timestamp
} control_data_t;
```

### Encoder Data Structure
```cpp
typedef struct {
    uint32_t left_encoder_pulseCount;   // Left encoder pulse count
    uint32_t right_encoder_pulseCount;  // Right encoder pulse count
    float left_motor_speed;             // Left motor speed (RPS)
    float right_motor_speed;            // Right motor speed (RPS)
} encoder_data_t;
```

### Web Control Structure
```cpp
typedef struct {
    float kp_roll, ki_roll, kd_roll;    // Roll PID parameters
    float kp_speed, ki_speed, kd_speed; // Speed PID parameters
    bool enable_balance;                 // Balance enable flag
    float target_angle;                  // Target tilt angle
} web_control_t;
```

## 🔍 Debugging

### Monitor Output
The system provides detailed logging:
```
I (xxxx) MPU: MPU6050 connection successful
I (xxxx) MPU: DMP is ready!
I (xxxx) WEB: Connect to: http://192.168.1.100
motor rps: LEFT: 2.1   RIGHT: 2.3
```

### Common Issues

1. **Robot falls immediately**:
   - Check MPU6050 orientation and mounting
   - Verify motor connections and polarity
   - Increase KP_ROLL gradually (start with 15.0)
   - Check if motors are responding correctly

2. **Oscillations**:
   - Reduce KP_ROLL
   - Increase KD_ROLL
   - Check for mechanical issues (loose connections)
   - Verify encoder readings are stable

3. **No response**:
   - Verify I2C connection (SDA/SCL pins)
   - Check motor driver power supply
   - Ensure proper WiFi configuration
   - Check serial monitor for error messages

4. **Web interface not accessible**:
   - Verify WiFi credentials in `web_interface.cpp`
   - Check if robot connected to WiFi (look for IP address in logs)
   - Ensure you're on the same network as the robot

5. **Encoder issues**:
   - Check encoder wiring (A/B channels)
   - Verify pull-up resistors are working
   - Check if encoder counts are incrementing in monitor output

## 🔌 PCB Design

The project includes a custom breakout board design in KiCad:

### Features
- **ESP32 NodeMCU-32S** mounting
- **MPU6050** module socket
- **Motor driver** connections (L298N compatible)
- **Encoder** input connectors
- **Power management** with voltage regulation
- **Debug headers** for easy testing

### Files
- `BreakoutBoard.kicad_pcb` - Main PCB layout
- `BreakoutBoard.kicad_sch` - Schematic design
- `Gerbers/` - Manufacturing files ready for PCB fabrication

### Usage
1. Order PCBs using the Gerber files
2. Solder components according to the schematic
3. Mount the ESP32 and MPU6050 modules
4. Connect motors and encoders to the appropriate headers

## 🛠️ Customization

### Adding Features

1. **Bluetooth Control**:
   - Add Bluetooth task alongside web server
   - Implement command parsing for mobile apps
   - Modify control task to accept external commands

2. **Enhanced Web Interface**:
   - Add data logging capabilities
   - Implement parameter persistence to flash
   - Add more sensor data visualization

3. **Data Logging**:
   - Add SD card support for data recording
   - Log sensor data for analysis
   - Export for MATLAB/Python analysis

4. **Advanced Control**:
   - Implement path following algorithms
   - Add obstacle avoidance
   - Implement remote control via web interface

### Motor Driver Support

The code supports various motor drivers through the `set_motor_speed()` function:

```cpp
void set_motor_speed(int motor, float speed) {
    // Current implementation supports L298N-style drivers
    // Easily adaptable for TB6612FNG, DRV8833, etc.
    // motor: 0 = right, 1 = left
    // speed: -255 to 255 (PWM value)
}
```

## 📚 Technical Details

### MPU6050 DMP Mode
- **Sample Rate**: 100Hz sensor reading
- **Fusion Algorithm**: Quaternion-based sensor fusion with gravity compensation
- **Output**: Direct Euler angles (yaw, pitch, roll) and linear acceleration
- **Calibration**: Pre-calibrated offsets for stable operation
- **DMP Features**: Built-in sensor fusion, FIFO buffer, interrupt support

### PID Control
- **Roll Control**: Primary balancing control using roll angle
- **Speed Control**: Forward/backward movement with encoder feedback
- **Anti-windup**: Integral term clamping to prevent windup
- **Derivative Filtering**: Prevents noise amplification
- **Real-time Tuning**: Parameters adjustable via web interface

### FreeRTOS Features
- **Task Priorities**: Sensor(5) > Control(4) > Motor(3) > Encoder(3) > Web(2) > Monitor(1)
- **Inter-task Communication**: Mutexes for data protection
- **Real-time Scheduling**: Deterministic timing with vTaskDelayUntil
- **Memory Management**: Optimized stack sizes for each task
- **Core Assignment**: All tasks run on Core 1 for consistency

### Motor Control
- **PWM Generation**: MCPWM hardware for precise motor control
- **Direction Control**: GPIO-based direction switching
- **Speed Range**: 30-255 PWM values with deadband protection
- **Encoder Feedback**: Quadrature encoder reading with interrupt handling

### Web Interface
- **HTTP Server**: Lightweight web server for parameter control
- **Real-time Updates**: 100ms refresh rate for live data
- **JSON API**: RESTful endpoints for data exchange
- **Responsive Design**: Works on desktop and mobile devices

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly
5. Submit a pull request

## 📄 License

This project is open source. Feel free to use, modify, and distribute.

## 🙏 Acknowledgments

- **Jeff Rowberg** for the excellent MPU6050 library and DMP implementation
- **ESP-IDF** team for the robust FreeRTOS implementation and ESP32 support
- **InvenSense** for the MPU6050 sensor and DMP technology
- **PlatformIO** team for the excellent development environment
- **KiCad** community for the open-source PCB design tools

## 📞 Support

For questions and support:
- Check the debugging section above
- Review the code comments
- Open an issue on GitHub

---

**Happy Balancing! 🤖⚖️**
