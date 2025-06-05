//
// Created by fogoz on 02/06/2025.
//

#ifndef FUNCTIONTARGET_H
#define FUNCTIONTARGET_H
#include "BaseTarget.h"
#include "robot/BaseRobot.h"
#include "utils/Position.h"
extern std::shared_ptr<BaseRobot> base_robot;

#define MAKE_FUNCTION_TARGET(function) makeFunctionTarget(function)

#define COMPLETE_FUNCTION_TARGET(function) robot->addTarget(MAKE_FUNCTION_TARGET(function))

class FunctionTarget : public BaseTarget {

protected:
    std::function<void()> function;
public:
    FunctionTarget(const std::shared_ptr<BaseRobot> &robot, const std::function<void()> &function)
        : BaseTarget(robot), function(function) {
    }

    void process() override {
        function();
        done = true;
        BaseTarget::process();
    }
};

inline std::shared_ptr<FunctionTarget> makeFunctionTarget(const std::function<void()> &function) {
    return std::make_shared<FunctionTarget>(base_robot, function);
}

inline std::shared_ptr<FunctionTarget> makeSetPositionTarget(Position pos) {
    return makeFunctionTarget([&](){base_robot->reset_to(pos);});
}

inline std::shared_ptr<FunctionTarget> makeSetXTarget(double x) {
    return makeFunctionTarget([&]() {
        Position pos = base_robot->getCurrentPosition();
        Position target = pos.offsetRelative(Position(x, pos.getY(), pos.getAngle()));
        base_robot->reset_to(target);
    });
}

inline std::shared_ptr<FunctionTarget> makeSetYTarget(double y) {
    return makeFunctionTarget([&]() {
        Position pos = base_robot->getCurrentPosition();
        Position target = pos.offsetRelative(Position(pos.getX(), y, pos.getAngle()));
        base_robot->reset_to(target);
    });
}

inline std::shared_ptr<FunctionTarget> makeSetAngleTarget(Angle angle) {
    return makeFunctionTarget([&]() {
        Position pos = base_robot->getCurrentPosition();
        Position target = pos.offsetRelative(Position(pos.getX(), pos.getY(), angle));
        base_robot->reset_to(target);
    });
}


#endif //FUNCTIONTARGET_H
