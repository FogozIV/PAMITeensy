//
// Created by fogoz on 23/04/2025.
//

#ifndef BASETARGET_H
#define BASETARGET_H
#include <memory>
#include <utility>
#include <vector>
#include <functional>
#include "utils/StreamSplitter.h"
#include "../utils/TaskScheduler.h"
#include "utils/Angle.h"

/**
 * @brief Error codes for target execution
 */
enum TargetError {
    TARGET_ERROR_BLOCKED,  ///< Target execution blocked (e.g., obstacle detected)
};

class BaseRobot;

/**
 * @brief Base class for robot motion targets
 * 
 * This class defines the interface for robot motion targets.
 * A target represents a desired robot state or motion, such as:
 * - Moving to a position
 * - Rotating to an angle
 * - Following a trajectory
 * - Executing a sequence of actions
 * 
 * The target system provides:
 * - State machine functionality
 * - Initialization and cleanup
 * - Completion detection
 * - Error handling
 * - Callback support
 */
class BaseTarget {
protected:
    friend class BaseRobot;
    std::shared_ptr<BaseRobot> robot;      ///< Reference to robot instance
    bool is_init = false;                  ///< Initialization flag
    bool done_called = false;              ///< Completion flag
    std::vector<std::function<void()>> end_callbacks{};        ///< Completion callbacks
    std::vector<std::function<void(TargetError)>> error_callbacks{};  ///< Error callbacks
    bool done = false;                     ///< Completion flag

    Angle previous_angle = AngleConstants::ZERO;
    double previous_distance = 0.0;
    uint16_t ticks = 0;
public:
    /**
     * @brief Constructs a new target
     * @param robot Reference to robot instance
     */
    explicit BaseTarget(std::shared_ptr<BaseRobot> robot)
        : robot(std::move(robot)) {
    }

    /**
     * @brief Adds an error callback
     * 
     * The callback will be called when a target error occurs.
     * Multiple callbacks can be registered.
     * 
     * @param callback Function to call on error
     */
    void addErrorCallback(std::function<void(TargetError)> callback) {
        error_callbacks.push_back(callback);
    }

    /**
     * @brief Initializes the target
     * 
     * This method is called once before target processing begins.
     * Override to perform target-specific initialization.
     */
    virtual void init() {
    }

    /**
     * @brief Checks if target is complete
     * 
     * This method determines when the target has been achieved.
     * Must be implemented by derived classes.
     * 
     * @return bool True if target is complete
     */
    virtual bool is_done() {
        return done;
    }

    /**
     * @brief Called when target completes
     * 
     * This method is called once when the target is achieved.
     * Override to perform target-specific cleanup.
     */
    virtual void on_done() {
    }

    /**
     * @brief Processes the target
     * 
     * This method is called repeatedly while the target is active.
     * Override to implement target-specific behavior.
     */
    virtual void process();

    /**
     * @brief Reinitializes target after stop
     * 
     * This method is called when target execution resumes
     * after being stopped. Override to handle state recovery.
     */
    virtual void reInitAfterStop() {
    }

    /**
     * @brief Initializes target if needed
     * 
     * Internal method that ensures init() is called exactly once.
     * Do not override this method.
     */
    void call_init() {
        if (!is_init) {
            init();
            is_init = true;
        }
    }

    void forceDone(){
        done = true;
    }

    /**
     * @brief Handles target completion
     * 
     * Internal method that:
     * - Calls on_done() once
     * - Executes completion callbacks
     * Do not override this method.
     */
    void call_done() {
        if (!done_called) {
            on_done();
            for (const auto& callback: end_callbacks) {
                scheduler->addTask(std::chrono::milliseconds(0), callback);
                //callback();
            }
            done_called = true;
        }
    }

    /**
     * @brief Adds a completion callback
     * 
     * The callback will be called when the target completes.
     * Multiple callbacks can be registered.
     * 
     * @param callback Function to call on completion
     */
    void addEndCallback(std::function<void()> callback) {
        end_callbacks.push_back(callback);
    }

    /**
     * @brief Virtual destructor
     */
    virtual ~BaseTarget() = default;
};

#endif //BASETARGET_H
