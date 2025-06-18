 # PAMITeensy System Architecture

## Class Hierarchy

### Core Components
```mermaid
classDiagram
    BaseRobot <|-- PAMIRobot
    BasicController <|-- PID
    BaseEncoder <|-- QuadEncoderImpl
    Motor <|-- DirPWMMotor
    
    class BaseRobot {
        +Position pos
        +BaseController controller
        +Motor leftMotor
        +Motor rightMotor
        +BaseEncoder leftEncoder
        +BaseEncoder rightEncoder
        +SpeedEstimator speedEstimator
        +computeTarget()
        +computePosition()
        +computeController()
        +compute()
    }
    
    class PAMIRobot {
        +PID pidDistance
        +PID pidAngle
        +PID pidDistanceAngle
        +TaskScheduler scheduler
        +init()
        +save()
        +computeTarget()
    }
    
    class PID {
        +double kp
        +double ki
        +double kd
        +double anti_windup
        +evaluate(error)
        +reset(error)
    }
    
    class Motor {
        +setPWM(double)
        +getPWM()
        +setMaxPWM(double)
        +isInversed()
    }
    
    class DirPWMMotor {
        -uint8_t dirPin
        -uint8_t pwmPin
        -double currentPWM
        +setPWM(double)
    }
    
    class BaseEncoder {
        +int32_t getCount()
        +void reset()
        +double getSpeed()
    }
    
    class QuadEncoderImpl {
        -QuadEncoder hardware
        +getEncoderCount()
        +getDeltaCount()
    }
```

### Motion Control
```mermaid
classDiagram
    class BaseTarget {
        +bool is_done()
        +void init()
        +void process()
        +void on_done()
    }
    
    class PositionTarget {
        -Position pos
        -Ramp speedRamp
        +void process()
        +bool is_done()
    }
    
    class AngleTarget {
        -Angle target_angle
        -Ramp speedRamp
        +void process()
        +bool is_done()
    }
    
    class CurveTarget {
        -BaseCurve curve
        -Ramp speedRamp
        -double step
        +void process()
        +bool is_done()
    }
    
    class Ramp {
        +void start()
        +double computeDelta()
        +double getCurrentSpeed()
        +void stop()
    }
    
    class BasicQuadRamp {
        -double acc
        -double maxSpeed
        +double computeDelta()
    }
    
    class DynamicQuadRamp {
        -double acc
        -double dec
        -double maxSpeed
        +double computeDelta()
    }
    
    class CalculatedQuadRamp {
        -RampData data
        -double t
        +double computeAtTime()
    }
    
    BaseTarget <|-- PositionTarget
    BaseTarget <|-- AngleTarget
    BaseTarget <|-- CurveTarget
    Ramp <|-- BasicQuadRamp
    Ramp <|-- DynamicQuadRamp
    Ramp <|-- CalculatedQuadRamp
```

### Task Management
```mermaid
classDiagram
    class ThreadPool {
        -vector~Thread~ workers
        -queue~Task~ tasks
        +addTask(Task)
        +stop()
    }
    
    class TaskScheduler {
        -ThreadPool pool
        -vector~Task~ tasks
        +addTask(duration, callback)
        +update()
    }
    
    class Task {
        +timePoint start
        +duration interval
        +callback function
        +bool repeat
    }
    
    TaskScheduler --> ThreadPool
    TaskScheduler --> Task
```

### Position Management
```mermaid
classDiagram
    class Position {
        +double x
        +double y
        +Angle theta
        +normalize()
        +rotate(Angle)
    }
    
    class PositionManager {
        +Position currentPos
        +SpeedEstimator estimator
        +updatePosition()
        +reset()
        +calibrate()
    }
    
    class SpeedEstimator {
        +double speed
        +double bandwidth
        +update(double)
        +reset()
    }
    
    PositionManager --> Position
    PositionManager --> SpeedEstimator
    BaseRobot --> Position
    BaseRobot --> SpeedEstimator
```

### Hardware Control
```mermaid
classDiagram
    class AX12Handler {
        +HardwareSerial serial
        +int baudrate
        +AX12 get(int id)
    }
    
    class AX12 {
        -int id
        -HardwareSerial serial
        +sendCommand()
        +readAX12()
        +writeAX12()
    }
    
    class DirPWMMotor {
        -uint8_t dirPin
        -uint8_t pwmPin
        +setPWM()
        +setDirection()
    }
    
    class QuadEncoderImpl {
        -QuadEncoder hardware
        +getCount()
        +getDelta()
    }
    
    AX12Handler --> AX12
    BaseRobot --> AX12Handler
    BaseRobot --> DirPWMMotor
    BaseRobot --> QuadEncoderImpl
```

### Communication
```mermaid
classDiagram
    class StreamSplitter {
        -vector~Print*~ streams
        +add(Print*)
        +write(uint8_t)
    }
    
    class CustomAsyncClient {
        -AsyncClient* client
        -PacketHandler handler
        +sendPacket(IPacket)
        +registerListener()
    }
    
    class CommandParser {
        -map~string,Command~ commands
        +registerCommand()
        +parse(string)
    }
    
    StreamSplitter --> Print
    CustomAsyncClient --> PacketHandler
    CommandParser --> Command
```

## Component Interactions

### Control Flow
```mermaid
sequenceDiagram
    participant MR as Main Robot Loop
    participant BC as Base Controller
    participant PM as Position Manager
    participant M as Motors
    participant E as Encoders
    
    MR->>PM: computePosition()
    PM->>E: getCount()
    E-->>PM: encoder counts
    PM-->>MR: current position
    
    MR->>BC: evaluate(error)
    BC-->>MR: control output
    
    MR->>M: setPWM()
    
    loop Speed Estimation
        MR->>E: getSpeed()
        E-->>MR: current speed
    end
```

### Data Flow
```mermaid
graph TD
    A[Encoders] -->|Position Data| B[Position Manager]
    B -->|Current Position| C[Controller]
    D[Target Position] -->|Error| C
    C -->|Control Output| E[Motors]
    F[Speed Estimator] -->|Speed Feedback| C
    A -->|Speed Data| F
```

## System Components

### 1. Robot Control
The robot control system is built around the `BaseRobot` class hierarchy:
- Position tracking and management
- Motor control interface
- Encoder integration
- Motion control algorithms
- Target management

### 2. Motion Control
The motion control system includes:
- PID controller implementation
- Speed estimation
- Position management
- Target tracking
- Speed ramping profiles
- Curve following
- Position and angle targets

### 3. Hardware Interface
Hardware abstraction layers for:
- Motor control (PWM and direction)
- Encoder reading
- AX12 servo communication
- Serial communication
- Network communication

### 4. Position Management
Position tracking system features:
- 2D position tracking (X, Y coordinates)
- Angular position management
- Speed estimation
- Encoder-based positioning
- Position calibration
- Parameter estimation

### 5. Task Management
Task scheduling and execution:
- Thread pool management
- Task scheduling
- Periodic task execution
- Resource management
- Command processing

### 6. Configuration Management
The system supports:
- JSON-based configuration
- Parameter persistence
- Runtime parameter adjustment
- Command-line interface
- Network configuration

### 7. Communication
Communication interfaces include:
- Serial communication
- Network communication
- Command parsing
- Status reporting
- Stream management
- Error handling 