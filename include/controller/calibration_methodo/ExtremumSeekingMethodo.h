//
// Created by fogoz on 05/06/2025.
//

#ifndef EXTREMUMSEEKINGMETHODO_H
#define EXTREMUMSEEKINGMETHODO_H
#include "BaseCalibrationMethodo.h"
#include "ramp/DynamicQuadRamp.h"
#include "robot/PAMIRobot.h"
#include "target/ContinuousCurveTarget.h"

namespace ESCType{
    enum ESC{
        ANGLE, DISTANCE, DISTANCE_ANGLE
    };
}

class ExtremumSeekingMethodo : public CalibrationMethodo {
protected:
    std::shared_ptr<PAMIRobot> robot;
    ESCType::ESC distance;
    double time = 0;
    double frequencyKP = 0.2 * 2 * M_PI;
    double frequencyKI = 0.31* 2 * M_PI;
    double frequencyKD = 0.47 * 2 * M_PI;

    double filtered_Jt = 0.0;
    double lpf_alpha = 0.1;


    double initialKP = 0;
    double initialKI = 0;
    double initialKD = 0;

    double alphaKP = 0.5;
    double alphaKI = 0.5;
    double alphaKD = 0.5;

    double gammaKP = 0.2;
    double gammaKI = 0.2;
    double gammaKD = 0.2;

    std::shared_ptr<PID> pid;
    uint64_t allTargetEndedHook;
    uint64_t endComputeHook;
    uint64_t waitTurnHook;
    double lambda = 0.000;
    double previousLeft = 0;
    double previousRight = 0;
    bool waiting_turn = false;

    struct IQ{
        double I,Q;
    };
    IQ iqs[3];
    std::shared_ptr<ContinuousCurveTarget<DynamicQuadRamp>> target ;


    void launchStage();

    void cleanupStage(std::function<void()> callback);

public:
    ExtremumSeekingMethodo(const std::shared_ptr<PAMIRobot> &robot, const std::shared_ptr<Mutex> &sdMutex, ESCType::ESC distance);

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

    void setLambda(double lambda);

    double ITAE(double error);

    double IAE(double error);

    double ISE(double error);

    double ISE_DU_DT(double error);

    virtual ~ExtremumSeekingMethodo() = default;
};



#endif //EXTREMUMSEEKINGMETHODO_H
