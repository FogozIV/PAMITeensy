//
// Created by fogoz on 23/04/2025.
//

#ifndef ROBOT_H
#define ROBOT_H
#include <utils/config.h>
#include <memory>
#include <utils/AX12.h>

#include "CommandParser.h"
#include "../motor/Motor.h"
#include "utils/Position.h"
#include "controller/BaseController.h"
#include "utils/SpeedEstimator.h"
#include "target/BaseTarget.h"
#include "utils/PositionManager.h"
#include <utils/CallbackManager.h>
#include <utils/EventNotifierAndWaiter.h>
#include "robot/RobotTolerance.h"

extern std::shared_ptr<Mutex> sdMutex;

extern CommandParser parser;
extern CommandParser xbeeCommandParser;

#define CALLBACKS_LIST\
    CALLBACK(EndComputeHooks)\
    CALLBACK(AllTargetEndedHooks)\
    CALLBACK(TargetEndedHooks)

enum RobotType {
    BASE,
    PAMIRobotType
};


/**
 * @brief Base class for robot control and management
 * 
 * This class provides the fundamental framework for robot control, including:
 * - Position and motion management
 * - Motor control
 * - Encoder integration
 * - Speed estimation
 * - Target tracking
 * - AX12 servo control
 */
class BaseRobot {
protected:
    mutable Mutex positionMutex;  ///< Mutex for thread-safe position access
    Position pos = {0,0,AngleConstants::ZERO};  ///< Current robot position (x, y, angle)
    std::shared_ptr<BaseController> controller;  ///< Motion controller
    std::shared_ptr<SpeedEstimator> distanceSpeedEstimator;  ///< Linear speed estimator
    std::shared_ptr<SpeedEstimator> angleSpeedEstimator;     ///< Angular speed estimator

    std::shared_ptr<SpeedEstimator> wheelDistanceSpeedEstimator;  ///< Wheel linear speed estimator
    std::shared_ptr<SpeedEstimator> wheelAngleSpeedEstimator;     ///< Wheel angular speed estimator

    std::shared_ptr<Motor> leftMotor;   ///< Left drive motor
    std::shared_ptr<Motor> rightMotor;  ///< Right drive motor
    std::shared_ptr<MotorParameters> leftMotorParameters;   ///< Left motor configuration
    std::shared_ptr<MotorParameters> rightMotorParameters;  ///< Right motor configuration
    bool motorInversed = false;  ///< Flag for motor direction inversion

    std::shared_ptr<BaseEncoder> leftEncoder;        ///< Left motor encoder
    std::shared_ptr<BaseEncoder> rightEncoder;       ///< Right motor encoder
    std::shared_ptr<BaseEncoder> leftWheelEncoder;   ///< Left wheel encoder
    std::shared_ptr<BaseEncoder> rightWheelEncoder;  ///< Right wheel encoder
    std::shared_ptr<PositionParameters> positionManagerParameters;      ///< Position tracking parameters
    std::shared_ptr<PositionParameters> wheelPositionManagerParameters; ///< Wheel position parameters
    std::shared_ptr<PositionManager> positionManager;           ///< Position tracking manager
    std::shared_ptr<PositionManager> motorWheelPositionManager; ///< Wheel position manager

    std::shared_ptr<AX12Handler> ax12Handler;  ///< AX12 servo controller
    std::vector<std::tuple<double, double, double>> calibrations;

    double translationPos = 0.0;  ///< Current linear position
    Angle rotationPos = AngleConstants::ZERO;  ///< Current angular position

    double translationTarget = 0.0;  ///< Target linear position
    Angle rotationTarget = AngleConstants::ZERO;  ///< Target angular position

    bool done_angular = true;     ///< Angular motion completion flag
    bool done_distance = true;    ///< Linear motion completion flag
    volatile bool control_disabled = false;  ///< Control system disable flag

    double translationalSpeedRamp = 0.0;  ///< Linear speed ramping value
    Angle rotationalSpeedRamp = AngleConstants::ZERO;  ///< Angular speed ramping value
    
    //for calibration only
    int32_t left_encoder_count = 0;   ///< Left encoder calibration count
    int32_t right_encoder_count = 0;  ///< Right encoder calibration count
    Position motorPos;  ///< Motor position for calibration

    mutable std::shared_ptr<Mutex> motorUpdate = nullptr; ///< Motor Update Mutex

    std::shared_ptr<EventNotifierAndWaiter> endOfComputeNotifier = std::make_shared<EventNotifierAndWaiter>();
    RobotType robotType;
    std::shared_ptr<RobotTolerance> tolerances;

#define CALLBACK(name) CallbackManager name;
    CALLBACKS_LIST
#undef CALLBACK

public:
    RobotType getRobotType() const;

    virtual ~BaseRobot() = default;

    virtual void update(double left, double right) {

    }

    std::shared_ptr<RobotTolerance> getTolerances();

    BaseRobot(RobotType robotType, std::shared_ptr<Mutex> motorUpdate = nullptr);

    /**
     * @brief Gets the current robot position
     * @return Position Current x, y, angle position
     */
    virtual Position getCurrentPosition();

    /**
     * @brief Gets the current motor-based position
     * @return Position Current position based on motor encoders
     */
    virtual Position getMotorPosition();

    /**
     * @brief Gets the left motor instance
     * @return std::shared_ptr<Motor> Left motor pointer
     */
    virtual std::shared_ptr<Motor> getLeftMotor();

    /**
     * @brief Gets the right motor instance
     * @return std::shared_ptr<Motor> Right motor pointer
     */
    virtual std::shared_ptr<Motor> getRightMotor();

    /**
     * @brief Checks if motors are in inverse configuration
     * @return bool True if motors are inversed
     */
    virtual bool isMotorInversed();

    /**
     * @brief Gets the motion controller
     * @return std::shared_ptr<BaseController> Controller pointer
     */
    virtual std::shared_ptr<BaseController> getController();
    /**
     * @brief allow to change the controller
     * @param controller an std::shared of a controller
     */
    void setController(std::shared_ptr<BaseController> controller);

    // Position and target getters/setters
    virtual double getTranslationalPosition();
    virtual Angle getRotationalPosition();
    virtual double getTranslationalTarget();
    virtual Angle getRotationalTarget();

    virtual void setTranslationalPosition(double pos);
    virtual void setRotationalPosition(Angle pos);
    virtual void setTranslationalTarget(double pos);
    virtual void setRotationalTarget(Angle pos);

    virtual void resetTargetsCurvilinearAndAngular();

    /**
     * @brief Checks if angular motion is complete
     * @return bool True if angular motion is done
     */
    virtual bool isDoneAngular();

    /**
     * @brief Checks if linear motion is complete
     * @return bool True if linear motion is done
     */
    virtual bool isDoneDistance();

    virtual void setDoneAngular(bool done);
    virtual void setDoneDistance(bool done);

    // Speed management methods
    virtual double getTranslationalRampSpeed();
    virtual Angle getRotationalRampSpeed();
    virtual double getTranslationalEstimatedSpeed();
    virtual Angle getRotationalEstimatedSpeed();
    virtual double getTranslationalOtherEstimatedSpeed() = 0;
    virtual Angle getRotationalOtherEstimatedSpeed() = 0;

    virtual void setTranslationalRampSpeed(double speed);
    virtual void setRotationalRampSpeed(Angle speed);

    /**
     * @brief Gets the control loop time step
     * @return double Time step in seconds
     */
    virtual double getDT() = 0;

    /**
     * @brief Computes the next target position
     */
    virtual void computeTarget() = 0;

    /**
     * @brief Updates the current position
     */
    virtual void computePosition() = 0;

    /**
     * @brief Executes the control loop
     */
    virtual void computeController() = 0;

    /**
     * @brief Adds a new motion target
     * @param target Target position/motion to achieve
     */
    virtual void addTarget(std::shared_ptr<BaseTarget> target) = 0;

    /**
     * @brief Clears all pending targets
     */
    virtual void clearTarget() = 0;

    /**
     * @brief Gets the number of pending targets
     * @return size_t Number of targets in queue
     */
    virtual size_t getTargetCount() = 0;

    /**
     * @brief Main computation loop
     */
    virtual void compute() = 0;

    // Speed estimator access
    virtual std::shared_ptr<SpeedEstimator> getDistanceEstimator();
    virtual std::shared_ptr<SpeedEstimator> getAngleEstimator();
    virtual std::shared_ptr<SpeedEstimator> getWheelDistanceEstimator();
    virtual std::shared_ptr<SpeedEstimator> getWheelAngleEstimator();

    // Calibration methods
    virtual void beginCalibrationEncoder();
    virtual void resetCalibrationEncoderList();
    virtual void addCalibrationData(double data);
    virtual void finalizeCalibrationForward();
    virtual void finalizeCalibrationRotation();


    static void calibrateMotorEncoder(Stream& stream, std::shared_ptr<BaseRobot> robot);
    virtual void setEncoderToMotors();
    virtual void setEncoderToFreeWheel();
    virtual void endCalibrationAngleTurnEncoder(double turns);
    virtual void endCalibrationAngleDegEncoder(double angle);
    virtual void endCalibrationAngleRadEncoder(double angle);
    virtual void endCalibrationStraightEncoder(double distance);
    virtual int32_t getLeftEncoderValue();
    virtual int32_t getRightEncoderValue();
    virtual int32_t getRightWheelEncoderValue();
    virtual int32_t getLeftWheelEncoderValue();
    virtual double computeCalibrationAngleRadEncoder(double angle);
    virtual double computeCalibrationAngleRadEncoder(int32_t left, int32_t right, double angle);
    virtual void printCalibrationParameters(Stream& stream);
    virtual std::tuple<double, double> computeCalibrationStraightEncoder(double distance);
    virtual std::tuple<double, double> computeCalibrationStraightEncoder(int32_t left, int32_t right, double distance);
    virtual void calibrateMotors();

    virtual void controllerClear() = 0;
#define CALLBACK(name) \
    void call##name(); \
    uint64_t add##name(std::function<void()> fct); \
    void remove##name(uint64_t id);

    CALLBACKS_LIST
#undef CALLBACK


    /**
     * @brief Resets robot position to specified coordinates
     * @param pos New position to set
     */
    virtual void reset_to(Position pos) = 0;

    /**
     * @brief Saves robot configuration
     * @return bool True if save successful
     */
    virtual bool save() = 0;

    /**
     * @brief Initializes the robot
     */
    virtual void init() = 0;

    /**
     * @brief Gets the AX12 servo handler
     * @return std::shared_ptr<AX12Handler> AX12 handler pointer
     */
    virtual std::shared_ptr<AX12Handler> getAX12Handler() const;

    /**
     * @brief Enables/disables the control system
     * @param value True to disable control
     */
    void setControlDisabled(bool value);

    /**
     * @brief Checks if control system is disabled
     * @return bool True if control is disabled
     */
    bool isControlDisabled() const;

    /**
     * @brief Registers robot commands with command parser
     * @param parser Command parser instance
     */
    virtual void registerCommands(CommandParser & parser) = 0;

    /**
     * @brief Gets the position mutex for thread-safe access
     * @return Mutex& Reference to position mutex
     */
    Mutex& getPositionMutex() const;

    void lockMotorMutex() const;

    void unlockMotorMutex() const;

    std::shared_ptr<EventNotifierAndWaiter> getEventEndOfComputeNotifier();
};

#endif //ROBOT_H
