//
// Created by fogoz on 08/06/2025.
//

#ifndef PIDSPEEDFEEDFORWARD_H
#define PIDSPEEDFEEDFORWARD_H
#include "PID.h"

namespace PIDSpeedFeedForwardType {
    enum FeedForward {
        DISTANCE,
        ANGLE
    };
}

class PIDSpeedFeedForward : public PID{
protected:
    double ff_gain;
public:
    double getUff() const;

protected:
    double uFF;
    std::function<double()> get_speed;
    PIDSpeedFeedForwardType::FeedForward feedforward_type;

public:

    PIDSpeedFeedForward(const std::shared_ptr<BaseRobot> &robot, double kp, double ki, double kd, double anti_windup, double ff_gain, PIDSpeedFeedForwardType::FeedForward type);

    PIDSpeedFeedForward(const std::shared_ptr<BaseRobot>& robot, const std::shared_ptr<PID>& pid=nullptr);

    double evaluate(double error) override;

    double simulate(double error) const override;

    double& getFeedForwardRef();

    static std::shared_ptr<PIDSpeedFeedForward> fromPID(std::shared_ptr<PID> pid, double ff_gain, std::shared_ptr<BaseRobot> robot, PIDSpeedFeedForwardType::FeedForward type);

    void serialize(JsonObject json) override;

    std::shared_ptr<BasicController> deserialize(std::shared_ptr<BaseRobot> robot, JsonVariant &json) override;
    /**
     * @brief Creates a PID controller from JSON configuration
     *
     * @param robot Reference to the robot being controlled
     * @param json JSON object containing PID parameters
     * @return std::shared_ptr<BasicController> Configured PID controller
     */
    template<typename T>
    static std::shared_ptr<T> deserialize_as_T(std::shared_ptr<BaseRobot> robot, const JsonVariant &json);

    void speedFromFeedForward();

    void registerCommands(CommandParser &parser, const char* name) override;

    void unregisterCommands(CommandParser &parser, const char* name) override;
};

template<typename T>
std::shared_ptr<T> PIDSpeedFeedForward::deserialize_as_T(std::shared_ptr<BaseRobot> robot, const JsonVariant &json) {
    std::shared_ptr<T> p = PID::deserialize_as_T<T>(robot, json);
    GET_AND_CHECK_JSON(p, ff_gain, double)
    GET_AND_CHECK_JSON(p, feedforward_type, PIDSpeedFeedForwardType::FeedForward)
    p->speedFromFeedForward();
    return p;
}


#endif //PIDSPEEDFEEDFORWARD_H
