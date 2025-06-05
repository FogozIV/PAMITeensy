//
// Created by fogoz on 05/06/2025.
//

#ifndef EXTREMUMSEEKINGMETHODO_H
#define EXTREMUMSEEKINGMETHODO_H
#include "BaseCalibrationMethodo.h"
#include "robot/PAMIRobot.h"


class ExtremumSeekingMethodo : public CalibrationMethodo {
protected:
    std::shared_ptr<PAMIRobot> robot;
    bool distance;
    double time = 0;
    double frequencyKP = 0.9 * M_PI;
    double frequencyKI = 0.93 * M_PI;
    double frequencyKD = 0.96 * M_PI;

    double initialKP = 0;
    double initialKI = 0;
    double initialKD = 0;

    double alphaKP = 0.05;
    double alphaKI = 0.05;
    double alphaKD = 0.05;

    double gammaKP = 0.2;
    double gammaKI = 0.2;
    double gammaKD = 0.2;

    std::shared_ptr<PID> pid;
    uint64_t allTargetEndedHook;
    uint64_t endComputeHook;
    double lambda = 0.1;
    double previousLeft = 0;
    double previousRight = 0;

    struct IQ{
        double I,Q;
    };
    IQ iqs[3];

    void launchStage();

    void cleanupStage();

public:
    ExtremumSeekingMethodo(const std::shared_ptr<PAMIRobot> &robot, const std::shared_ptr<Mutex> &sdMutex, bool distance);

    void save() override;

    void printStatus(Stream &stream) override;

    void start() override;

    void stop() override;

    void setAlphaKP(double alpha_kp);

    void setAlphaKI(double alpha_ki);

    void setAlphaKD(double alpha_kd);

    void setGammaKP(double gamma_kp);

    void setGammaKI(double gamma_ki);

    void setGammaKD(double gamma_kd);

    double ITAE(double error);

    double IAE(double error);

    double ISE(double error);

    double ISE_DU_DT(double error);

    virtual ~ExtremumSeekingMethodo() = default;
};



#endif //EXTREMUMSEEKINGMETHODO_H
