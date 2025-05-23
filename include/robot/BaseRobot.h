//
// Created by fogoz on 23/04/2025.
//

#ifndef ROBOT_H
#define ROBOT_H
#include <memory>
#include <utils/AX12.h>

#include "CommandParser.h"
#include "../motor/Motor.h"
#include "utils/Position.h"
#include "controller/BaseController.h"
#include "utils/SpeedEstimator.h"
#include "target/BaseTarget.h"
#include "utils/PositionManager.h"
extern std::shared_ptr<std::mutex> sdMutex;
class BaseRobot{
protected:
    mutable std::mutex positionMutex;
    Position pos = {0,0,AngleConstants::ZERO};
    std::shared_ptr<BaseController> controller;
    std::shared_ptr<SpeedEstimator> distanceSpeedEstimator;
    std::shared_ptr<SpeedEstimator> angleSpeedEstimator;

    std::shared_ptr<SpeedEstimator> wheelDistanceSpeedEstimator;
    std::shared_ptr<SpeedEstimator> wheelAngleSpeedEstimator;

    std::shared_ptr<Motor> leftMotor;
    std::shared_ptr<Motor> rightMotor;
    std::shared_ptr<MotorParameters> leftMotorParameters;
    std::shared_ptr<MotorParameters> rightMotorParameters;
    bool motorInversed = false;


    std::shared_ptr<BaseEncoder> leftEncoder;
    std::shared_ptr<BaseEncoder> rightEncoder;
    std::shared_ptr<BaseEncoder> leftWheelEncoder;
    std::shared_ptr<BaseEncoder> rightWheelEncoder;
    std::shared_ptr<PositionParameters> positionManagerParameters;
    std::shared_ptr<PositionParameters> wheelPositionManagerParameters;
    std::shared_ptr<PositionManager> positionManager;
    std::shared_ptr<PositionManager> motorWheelPositionManager;

    std::shared_ptr<AX12Handler> ax12Handler;

    double translationPos = 0.0;
    Angle rotationPos = AngleConstants::ZERO;

    double translationTarget = 0.0;
    Angle rotationTarget = AngleConstants::ZERO;

    bool done_angular = true;
    bool done_distance = true;
    bool control_disabled = false;

    double translationalSpeedRamp = 0.0;
    Angle rotationalSpeedRamp = AngleConstants::ZERO;
    //for calibration only
    int32_t left_encoder_count = 0;
    int32_t right_encoder_count = 0;
    Position motorPos;

public:
    virtual ~BaseRobot() = default;

    virtual Position getCurrentPosition();

    virtual Position getMotorPosition();

    virtual std::shared_ptr<Motor> getLeftMotor();
    virtual std::shared_ptr<Motor> getRightMotor();
    virtual bool isMotorInversed();

    virtual std::shared_ptr<BaseController> getController();

    virtual double getTranslationalPosition();
    virtual Angle getRotationalPosition();
    virtual double getTranslationalTarget();
    virtual Angle getRotationalTarget();

    virtual void setTranslationalPosition(double pos);
    virtual void setRotationalPosition(Angle pos);
    virtual void setTranslationalTarget(double pos);
    virtual void setRotationalTarget(Angle pos);

    virtual bool isDoneAngular();
    virtual bool isDoneDistance();

    virtual void setDoneAngular(bool done);
    virtual void setDoneDistance(bool done);

    virtual double getTranslationalRampSpeed();
    virtual Angle getRotationalRampSpeed();
    virtual double getTranslationalEstimatedSpeed();
    virtual Angle getRotationalEstimatedSpeed();

    virtual void setTranslationalRampSpeed(double speed);
    virtual void setRotationalRampSpeed(Angle speed);

    virtual double getDT() = 0;

    virtual void computeTarget() = 0;
    virtual void computePosition() = 0;
    virtual void computeController() = 0;

    virtual void addTarget(std::shared_ptr<BaseTarget> target) = 0;

    virtual void clearTarget() = 0;

    virtual size_t getTargetCount() = 0;

    virtual void compute() = 0;

    virtual std::shared_ptr<SpeedEstimator> getDistanceEstimator();

    virtual std::shared_ptr<SpeedEstimator> getAngleEstimator();

    virtual std::shared_ptr<SpeedEstimator> getWheelDistanceEstimator();

    virtual std::shared_ptr<SpeedEstimator> getWheelAngleEstimator();

    virtual void beginCalibrationEncoder();

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

    virtual std::tuple<double, double> computeCalibrationStraightEncoder(double distance);

    virtual void calibrateMotors();

    virtual void reset_to(Position pos) = 0;

    virtual bool save() = 0;

    virtual void init() = 0;

    virtual std::shared_ptr<AX12Handler> getAX12Handler() const;

    virtual void setControlDisabled(bool value);

    bool isControlDisabled() const;
    virtual void registerCommands(CommandParser & parser) = 0;

    std::mutex& getPositionMutex() const;
};



#endif //ROBOT_H
