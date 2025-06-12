//
// Created by fogoz on 12/06/2025.
//

#ifndef CURVEBENCHMARK_H
#define CURVEBENCHMARK_H

#include <controller/calibration_methodo/BaseCalibrationMethodo.h>
#include <robot/PAMIRobot.h>
#include <target/ContinuousCurveTarget.h>
#include <ramp/CalculatedQuadramp.h>
#include <basic_controller/PIDSpeedFeedForward.h>

class CurveBenchmark : public CalibrationMethodo {
protected:
    std::shared_ptr<PAMIRobot> robot;
    uint64_t benchmarkComputeHook;
    uint64_t allTargetHook;
    double error = 0;
    double dt = 0;
    std::shared_ptr<ContinuousCurveTarget<CalculatedQuadramp>> curveTarget;

public:
    CurveBenchmark(const std::shared_ptr<BaseRobot> &robot, const std::shared_ptr<Mutex> &sdMutex,std::shared_ptr<ContinuousCurveTarget<CalculatedQuadramp>> curveTarget);

    void save() override;

    void printStatus(Stream &stream) override;

    void start() override;

    void stop() override;
};

#endif //CURVEBENCHMARK_H
