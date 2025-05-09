//
// Created by fogoz on 23/04/2025.
//

#ifndef BASETARGET_H
#define BASETARGET_H
#include <memory>
#include <vector>
#include <functional>

enum TargetError {
    TARGET_ERROR_BLOCKED,
};

class BaseRobot;
class BaseTarget{
protected:
    friend class BaseRobot;
    std::shared_ptr<BaseRobot> robot;
    bool is_init = false;
    bool done_called = false;
    std::vector<std::function<void()>> end_callbacks{};
    std::vector<std::function<void(TargetError)>> error_callbacks{};


public:
    explicit BaseTarget(std::shared_ptr<BaseRobot> robot)
        : robot(robot) {
    }

    void addErrorCallback(std::function<void(TargetError)> callback) {
        error_callbacks.push_back(callback);
    }

    virtual void init() {
    };

    virtual bool is_done() = 0;

    virtual void on_done() {
    };

    virtual void process() {
    };

    virtual void reInitAfterStop() {
    }
    void call_init() {
        if (!is_init) {
            init();
            is_init = true;
        }
    }

    void call_done() {
        if (!done_called) {
            on_done();
            for (auto callback: end_callbacks) {
                callback();
            }
            done_called = true;
        }
    }

    void addEndCallback(std::function<void()> callback) {
        end_callbacks.push_back(callback);
    }

    virtual ~BaseTarget() = default;
};

#endif //BASETARGET_H
