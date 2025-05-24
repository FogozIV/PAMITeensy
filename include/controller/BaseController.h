//
// Created by fogoz on 24/04/2025.
//

#ifndef PAMITEENSY_BASECONTROLLER_H
#define PAMITEENSY_BASECONTROLLER_H
#include "memory"
#include "Arduino.h"

/**
 * @brief Constrains a value within symmetric limits
 * 
 * This function limits a value to the range [-max_min, max_min].
 * Useful for:
 * - Limiting motor speeds
 * - Constraining control outputs
 * - Enforcing safety limits
 * 
 * @param value Value to constrain
 * @param max_min Maximum absolute value
 * @return double Constrained value
 */
inline double applyMaxMin(double value, double max_min) {
    return constrain(value, -max_min, max_min);
}

/**
 * @brief Applies minimum speed offset
 * 
 * This function adds a minimum speed offset to non-zero values.
 * Used to overcome:
 * - Motor stiction
 * - Dead zones
 * - Minimum operating thresholds
 * 
 * The offset is:
 * - Added to positive values
 * - Subtracted from negative values
 * - Zero remains unchanged
 * 
 * @param value Input value
 * @param speed_min Minimum speed offset
 * @return double Value with minimum speed applied
 */
inline double applySpeedMin(double value, double speed_min) {
    if(value > 0) {
        value += speed_min;
    } else if(value < 0) {
        value -= speed_min;
    }
    return value;
}

/**
 * @brief Base class for motion controllers
 * 
 * This class defines the interface for robot motion controllers.
 * Controllers are responsible for:
 * - Computing control outputs
 * - Managing control state
 * - Handling transitions
 * 
 * Implementations include:
 * - PID controllers
 * - Cascaded controllers
 * - Custom control algorithms
 */
class BaseController {
public:
    /**
     * @brief Executes one control cycle
     * 
     * This method should:
     * - Read sensor inputs
     * - Compute control outputs
     * - Update controller state
     */
    virtual void compute() = 0;

    /**
     * @brief Resets controller state
     * 
     * @param correct_error If true, maintains current error for smooth transition
     */
    virtual void reset(bool correct_error=false) = 0;

    /**
     * @brief Virtual destructor
     */
    virtual ~BaseController() = default;
};

#endif //PAMITEENSY_BASECONTROLLER_H
