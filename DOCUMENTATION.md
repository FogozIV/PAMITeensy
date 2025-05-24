# PAMITeensy Robot Control System Documentation

## Overview
PAMITeensy is a robotics control system designed for Teensy microcontrollers. The system provides a comprehensive framework for robot control, motor management, and sensor integration. It features:

- Real-time motion control
- PLL-based speed estimation
- Thread-safe operations
- Task scheduling
- Configuration management
- Extensible architecture
- Advanced motion profiles
- Network communication

## System Architecture

### Core Components

#### 1. Robot Control
The robot control system is built around the `BaseRobot` class hierarchy:
- Position tracking and management
- Motor control interface
- Encoder integration
- Motion control algorithms
- Target management
- Parameter estimation

The concrete implementation `PAMIRobot` provides:
- Triple PID control (distance, angle, combined)
- SD card configuration storage
- Command interface
- Thread-safe operation
- Network communication

#### 2. Motion Control
The motion control system includes:

##### PID Controller
- Configurable PID parameters (Kp, Ki, Kd)
- Anti-windup protection
- JSON configuration support
- Real-time parameter adjustment

##### Speed Estimation
- PLL-based velocity estimation
- Position tracking
- Configurable bandwidth
- Noise filtering

##### Motion Profiles
- Basic quadratic ramping
- Dynamic quadratic ramping
- Calculated quadratic ramping
- Speed profile generation
- Position-based control
- Angle-based control
- Curve following

#### 3. Hardware Interface

##### Motor Control
- Direction and PWM control
- Configurable PWM resolution
- Thread-safe operation
- Safety limits
- Speed ramping

##### Encoder Interface
- Quadrature decoding
- Hardware-based counting
- Delta measurement
- Position tracking
- Parameter estimation

##### AX12 Servo Control
- Packet-based communication
- Command/response handling
- Error detection
- Register access
- Auto-generated commands

#### 4. Task Management

##### Thread Pool
- Parallel task execution
- Load balancing
- Resource management
- Graceful shutdown

##### Task Scheduler
- Periodic task execution
- One-time task scheduling
- Thread pool integration
- Task synchronization
- Command processing

#### 5. Communication

##### Network Interface
- Asynchronous TCP client
- Packet-based communication
- Event-driven architecture
- Type-safe packet handling
- Connection management

##### Stream Management
- Multi-destination output
- Stream multiplexing
- Raw and smart pointer support
- Buffered writing
- Error handling

## Configuration

### Motor Configuration
```json
{
    "inversed": false,
    "resolution": 12,
    "max_pwm": 4095
}
```

### PID Controller Configuration
```json
{
    "kp": 1.0,
    "ki": 0.1,
    "kd": 0.05,
    "anti_windup": 1.0
}
```

### Position Parameters
```json
{
    "left_wheel_diam": 0.06,
    "right_wheel_diam": 0.06,
    "wheel_distance": 0.2
}
```

### Motion Profile Configuration
```json
{
    "ramping": {
        "acceleration": 0.5,
        "max_speed": 1.0,
        "end_speed": 0.0
    },
    "curve_following": {
        "step_size": 20.0
    }
}
```

## Usage Examples

### Basic Robot Setup
```cpp
auto robot = std::make_shared<PAMIRobot>();
robot->init();  // Loads configuration from SD card
```

### Motor Control
```cpp
auto motor = std::make_shared<DirPWMMotor>(dirPin, pwmPin, parameters);
motor->setPWM(0.5);  // 50% speed forward
motor->setPWM(-0.3); // 30% speed reverse
```

### Encoder Reading
```cpp
auto encoder = std::make_shared<QuadEncoderImpl>(pinA, pinB, channel);
int32_t count = encoder->getEncoderCount();
int32_t delta = encoder->getDeltaCount();
```

### Task Scheduling
```cpp
auto pool = std::make_shared<ThreadPool>(4);
auto scheduler = std::make_shared<TaskScheduler>(pool);

// Schedule periodic task
scheduler->addTask(std::chrono::milliseconds(100), []() {
    // Task code
}, std::chrono::seconds(1));
```

### Motion Control
```cpp
// Position target
auto posTarget = std::make_shared<PositionTarget>(robot, Position{1.0, 0.0}, rampData);
robot->addTarget(posTarget);

// Angle target
auto angleTarget = std::make_shared<AngleTarget>(robot, Angle{90.0}, rampData);
robot->addTarget(angleTarget);

// Curve following
auto curve = std::make_shared<CircleCurve>(1.0);  // 1 meter radius
auto curveTarget = std::make_shared<CurveTarget>(robot, curve, rampData);
robot->addTarget(curveTarget);
```

### Network Communication
```cpp
auto client = std::make_shared<CustomAsyncClient>(tcpClient);

// Register packet listener
client->registerPacketListener<StatusPacket>([](auto packet) {
    // Handle status packet
    return true;
});

// Send command
auto command = std::make_shared<CommandPacket>("move", "forward");
client->sendPacket(command);
```

## System Requirements

### Hardware
- Teensy 4.0/4.1 microcontroller
- Quadrature encoders
- DC motors with PWM support
- Optional: AX12 servos
- Optional: SD card for configuration
- Optional: Network interface

### Software Dependencies
- Arduino IDE with Teensyduino
- Libraries:
  - TeensyThreads
  - ArduinoJson
  - QuadEncoder
  - SD (for configuration storage)
  - AsyncTCP (for network communication)

## Thread Safety
The system implements thread-safe operations through:
- Mutex protection for shared resources
- Atomic operations where appropriate
- Thread-safe communication handlers
- Task scheduling and synchronization
- Event-driven architecture

## Future Improvements
- Enhanced safety implementations
- Additional sensor support
- Extended motion planning
- Enhanced calibration features
- Advanced trajectory generation
- Real-time visualization
- Remote debugging support 