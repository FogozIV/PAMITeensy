//
// Created by fogoz on 24/04/2025.
//

#ifndef PAMITEENSY_BASICCONTROLLER_H
#define PAMITEENSY_BASICCONTROLLER_H

/**
 * @brief Base class for single-input controllers
 * 
 * This class defines the interface for basic feedback controllers
 * that operate on a single error value. It's used for:
 * - PID control
 * - Simple feedback loops
 * - Error-based control
 * 
 * The controller takes an error value (difference between
 * desired and actual values) and produces a control output.
 */
class BasicController {
public:
    /**
     * @brief Evaluates controller output
     * 
     * This method computes the control output based on
     * the current error value. The error is typically:
     * error = setpoint - measured_value
     * 
     * @param error Current error value
     * @return double Control output
     */
    virtual double evaluate(double error) = 0;

    /**
     * @brief Resets controller with initial error
     * 
     * Resets the controller state using a specific
     * initial error value. This is useful for:
     * - Initialization
     * - Mode changes
     * - Preventing control bumps
     * 
     * @param error Initial error value
     */
    virtual void reset(double error) = 0;

    /**
     * @brief Resets controller to zero
     * 
     * Convenience method that resets the controller
     * with an initial error of zero.
     */
    void reset() {
        reset(0.0f);
    }

    /**
     * @brief Virtual destructor
     */
    virtual ~BasicController() = default;
};

#endif //PAMITEENSY_BASICCONTROLLER_H
