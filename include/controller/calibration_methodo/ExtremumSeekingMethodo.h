//
// Created by fogoz on 05/06/2025.
//

#ifndef EXTREMUMSEEKINGMETHODO_H
#define EXTREMUMSEEKINGMETHODO_H
#include "BaseCalibrationMethodo.h"
#include "ramp/DynamicQuadRamp.h"
#include "robot/PAMIRobot.h"
#include "target/ContinuousCurveTarget.h"
#include <controller/calibration_methodo/ESCType.h>


class ExtremumSeekingMethodo : public CalibrationMethodo {
protected:
    std::shared_ptr<PAMIRobot> robot;
    ESCType::ESC distance;
    double time = 0;

    double filtered_Jt = 0.0;
    double lpf_alpha = 0.1;

    std::vector<double> initialGains;

    std::shared_ptr<BasicController> controller;
    uint64_t allTargetEndedHook;
    uint64_t endComputeHook;
    uint64_t waitTurnHook;
    double lambda = 0.000;
    double previousLeft = 0;
    double previousRight = 0;
    bool waiting_turn = false;
    std::vector<std::pair<double, double>> iqs;
    std::shared_ptr<ContinuousCurveTarget<DynamicQuadRamp>> target ;

    double gamma;
    double alpha;


    void launchStage();

    void cleanupStage(std::function<void()> callback, bool update=true);

public:
    ExtremumSeekingMethodo(const std::shared_ptr<PAMIRobot> &robot, const std::shared_ptr<Mutex> &sdMutex, ESCType::ESC distance, double gamma=-1, double alpha=-1);

    void save() override;

    void printStatus(Stream &stream) override;

    void start() override;

    void stop() override;

    void setLambda(double lambda);

    double ITAE(double error);

    double IAE(double error);

    double ISE(double error);

    double ISE_DU_DT(double error);

    virtual ~ExtremumSeekingMethodo() = default;
};



#endif //EXTREMUMSEEKINGMETHODO_H
