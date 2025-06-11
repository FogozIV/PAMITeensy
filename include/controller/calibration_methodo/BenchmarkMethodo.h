//
// Created by fogoz on 04/06/2025.
//

#ifndef BENCHMARKMETHODO_H
#define BENCHMARKMETHODO_H
#include "BaseCalibrationMethodo.h"
#include "robot/PAMIRobot.h"


enum BenchmarkMode {
    ANGLE,
    DISTANCE,
    ANGLE_DISTANCE
};

class BenchmarkMethodo : public CalibrationMethodo {
protected:
    BenchmarkMode benchmark_type;
    std::shared_ptr<PAMIRobot> robot;
    uint64_t benchmarkComputeHook;
    uint64_t allTargetHook;
    double error = 0;
    double dt = 0;

    double multDistance = 1;
    double multAngle = DEG_TO_RAD * 10;

public:
    BenchmarkMethodo(const std::shared_ptr<BaseRobot> &robot, const std::shared_ptr<Mutex> &sdMutex, BenchmarkMode benchmark_type);

    void save() override;

    void printStatus(Stream &stream) override;

    void start() override;

    void stop() override;

    void setMultDistance(double multDistance);

    void setMultAngle(double multAngle);
};



#endif //BENCHMARKMETHODO_H
