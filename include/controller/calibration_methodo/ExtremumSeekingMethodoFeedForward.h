//
// Created by fogoz on 05/06/2025.
//

#ifndef EXTREMUMSEEKINGMETHODOFEEDFORWARD_H
#define EXTREMUMSEEKINGMETHODOFEEDFORWARD_H
#include "BaseCalibrationMethodo.h"
#include "basic_controller/PIDSpeedFeedForward.h"
#include "robot/PAMIRobot.h"


class ExtremumSeekingMethodoFeedForward : public CalibrationMethodo {
protected:
    std::shared_ptr<PAMIRobot> robot;
    bool distance;
    double time = 0;
    double frequencyKP = 0.9 * M_PI;
    double frequencyKI = 0.93 * M_PI;
    double frequencyKD = 0.96 * M_PI;
    double frequencyKPP = 0.98 * M_PI;

    double initialKP = 0;
    double initialKI = 0;
    double initialKD = 0;
    double initialKPP = 0;

    double alphaKP = 0.05;
    double alphaKI = 0.05;
    double alphaKD = 0.05;
    double alphaKPP = 0.05;

    double gammaKP = 0.2;
    double gammaKI = 0.2;
    double gammaKD = 0.2;
    double gammaKPP = 0.2;

    std::shared_ptr<PIDSpeedFeedForward> pid;
    std::shared_ptr<BasicController> previousController;
    uint64_t allTargetEndedHook;
    uint64_t endComputeHook;
    double lambda = 0.001;
    double previousLeft = 0;
    double previousRight = 0;

    struct IQ{
        double I,Q;
    };
    IQ iqs[4];

    void launchStage();

    void cleanupStage();

public:
    ExtremumSeekingMethodoFeedForward(const std::shared_ptr<PAMIRobot> &robot, const std::shared_ptr<Mutex> &sdMutex, bool distance);

    void save() override;

    void printStatus(Stream &stream) override;

    void start() override;

    void stop() override;

    void setAlphaKP(double alpha_kp);

    void setAlphaKI(double alpha_ki);

    void setAlphaKD(double alpha_kd);

    void setAlphaKPP(double alpha_kpp);

    void setGammaKP(double gamma_kp);

    void setGammaKI(double gamma_ki);

    void setGammaKD(double gamma_kd);

    void setGammaKPP(double gamma_kpp);

    void setLambda(double lambda);

    double ITAE(double error);

    double IAE(double error);

    double ISE(double error);

    double ISE_DU_DT(double error);

};



#endif //EXTREMUMSEEKINGMETHODOFEEDFORWARD_H
