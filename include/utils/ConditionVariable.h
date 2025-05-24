//
// Created by fogoz on 08/05/2025.
//

#ifndef CONDITIONVARIABLE_H
#define CONDITIONVARIABLE_H
#include <functional>

/**
 * @brief Variable with conditional callback functionality
 * 
 * This class implements a variable that can trigger a callback
 * function whenever its value changes. It is useful for:
 * - Parameter monitoring
 * - Value change notifications
 * - Automatic updates
 * - Configuration propagation
 * 
 * The class supports:
 * - Direct value assignment
 * - Callback registration
 * - Automatic type conversion
 * - Value propagation
 */
class ConditionalVariable {
    double value;                           ///< Stored value
    std::function<void(double)> applyFunction;  ///< Callback function

public:
    /**
     * @brief Constructs a new conditional variable
     * 
     * @param bandwidth Initial value (default: 0.0)
     * @param fct Optional callback function
     */
    ConditionalVariable(double bandwidth=0.0f, std::function<void(double)> fct=nullptr)
        : value(bandwidth), applyFunction(fct) {}

    /**
     * @brief Assignment operator for direct value
     * 
     * Assigns a new value and triggers the callback if one
     * is registered.
     * 
     * @param value New value to assign
     * @return ConditionalVariable& Reference to this
     */
    ConditionalVariable& operator=(double value) {
        this->value = value;
        if(applyFunction != nullptr){
            applyFunction(value);
        }
        return *this;
    }

    /**
     * @brief Assignment operator for another conditional variable
     * 
     * Copies value and optionally the callback from another
     * conditional variable. If this variable already has a
     * callback, it keeps its own.
     * 
     * @param bandwidth Source conditional variable
     * @return ConditionalVariable& Reference to this
     */
    ConditionalVariable& operator=(ConditionalVariable bandwidth) {
        this->value = bandwidth.value;
        if (this->applyFunction == nullptr) {
            this->applyFunction = bandwidth.applyFunction;
        }
        if(applyFunction != nullptr){
            applyFunction(bandwidth.value);
        }
        return *this;
    }

    /**
     * @brief Conversion operator to double
     * 
     * Allows using the conditional variable directly in
     * numerical expressions.
     * 
     * @return double Current value
     */
    operator double() const {
        return value;
    }
};

#endif //CONDITIONVARIABLE_H
