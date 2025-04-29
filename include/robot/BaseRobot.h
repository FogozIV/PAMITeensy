//
// Created by fogoz on 23/04/2025.
//

#ifndef ROBOT_H
#define ROBOT_H
#include <memory>
#include <utils/AX12.h>

#include "../motor/Motor.h"
#include "utils/Position.h"
#include "controller/BaseController.h"
#include "utils/SpeedEstimator.h"
#include "target/BaseTarget.h"
#include "utils/PositionManager.h"

class BaseRobot {
protected:
    Position pos = {0,0,0};
    std::shared_ptr<BaseController> controller;
    std::shared_ptr<SpeedEstimator> distanceSpeedEstimator;
    std::shared_ptr<SpeedEstimator> angleSpeedEstimator;

    std::shared_ptr<Motor> leftMotor;
    std::shared_ptr<Motor> rightMotor;
    bool motorInversed = false;


    std::shared_ptr<BaseEncoder> leftEncoder;
    std::shared_ptr<BaseEncoder> rightEncoder;
    std::shared_ptr<PositionParameters> positionManagerParameters;
    std::shared_ptr<PositionManager> positionManager;

    std::shared_ptr<AX12Handler> ax12Handler;

    double translationPos = 0.0;
    double rotationPos = 0.0;

    double translationTarget = 0.0;
    double rotationTarget = 0.0;

    bool done_angular = true;
    bool done_distance = true;
    bool control_disabled = false;

    double translationalSpeedRamp = 0.0;
    double rotationalSpeedRamp = 0.0;
    //for calibration only
    int32_t left_encoder_count = 0;
    int32_t right_encoder_count = 0;


    public:
    virtual ~BaseRobot() = default;

    virtual Position getCurrentPosition();

    virtual std::shared_ptr<Motor> getLeftMotor();
    virtual std::shared_ptr<Motor> getRightMotor();
    virtual bool isMotorInversed();

    virtual std::shared_ptr<BaseController> getController();

    virtual double getTranslationalPosition();
    virtual double getRotationalPosition();
    virtual double getTranslationalTarget();
    virtual double getRotationalTarget();

    virtual void setTranslationalPosition(double pos);
    virtual void setRotationalPosition(double pos);
    virtual void setTranslationalTarget(double pos);
    virtual void setRotationalTarget(double pos);

    virtual bool isDoneAngular();
    virtual bool isDoneDistance();

    virtual void setDoneAngular(bool done);
    virtual void setDoneDistance(bool done);

    virtual double getTranslationalRampSpeed();
    virtual double getRotationalRampSpeed();
    virtual double getTranslationalEstimatedSpeed();
    virtual double getRotationalEstimatedSpeed();

    virtual void setTranslationalRampSpeed(double speed);
    virtual void setRotationalRampSpeed(double speed);

    virtual double getDT() = 0;

    virtual void computeTarget() = 0;
    virtual void computePosition() = 0;
    virtual void computeController() = 0;

    virtual void addTarget(std::shared_ptr<BaseTarget> target) = 0;

    virtual void compute() = 0;

    virtual std::shared_ptr<SpeedEstimator> getDistanceEstimator();

    virtual std::shared_ptr<SpeedEstimator> getAngleEstimator();

    virtual void beginCalibrationEncoder();

    virtual void endCalibrationAngleTurnEncoder(double turns);

    virtual void endCalibrationAngleDegEncoder(double angle);

    virtual void endCalibrationAngleRadEncoder(double angle);

    virtual void endCalibrationStraightEncoder(double distance);

    virtual double computeCalibrationAngleRadEncoder(double angle);

    virtual std::tuple<double, double> computeCalibrationStraightEncoder(double distance);

    virtual void calibrateMotors();

    virtual void reset_to(Position pos) = 0;

    virtual bool save() = 0;

    virtual std::shared_ptr<AX12Handler> getAX12Handler() const;

    virtual void setControlDisabled(bool value);

    bool isControlDisabled() const;

};



#endif //ROBOT_H
