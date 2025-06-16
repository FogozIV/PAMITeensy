//
// Created by fogoz on 27/04/2025.
//

#ifndef RAMP_H
#define RAMP_H

/**
 * @brief Configuration data for speed ramping
 * 
 * This structure holds parameters for speed ramping:
 * - Acceleration limit
 * - Maximum speed limit
 * - Target end speed
 * 
 * Used to configure smooth speed transitions
 * while respecting system constraints.
 */
struct RampData{
    double acc;        ///< Acceleration limit
    double maxSpeed;   ///< Maximum allowed speed
    double endSpeed = 0;  ///< Target end speed
    double dec = 0;    ///< Deceleration limit (default: acc)
    double max_lateral_accel = 0; ///< Maximum lateral acceleration (for supported ramps)
    public:
    /**
     * @brief Constructs ramping parameters
     * 
     * @param acc Maximum acceleration
     * @param maxSpeed Maximum speed
     * @param endSpeed Target end speed (default: 0)
     * @param dec Maximum deceleration (default: acc)
     */
    RampData(double acc, double maxSpeed, double endSpeed=0, double dec=0.0, double max_lateral_accel=0.0) : acc(acc), maxSpeed(maxSpeed), endSpeed(endSpeed), dec(dec), max_lateral_accel(max_lateral_accel) {
        if (dec == 0.0) {
            this->dec = acc;
        }
        if (max_lateral_accel == 0.0) {
            this->max_lateral_accel = 200;
        }
    };

    RampData& setDeceleration(double dec) {
        this->dec = dec;
        return *this;
    }

    RampData& setMaxLateralAccel(double max_lateral_accel) {
        if (max_lateral_accel == 0.0) {
            return *this;
        }
        this->max_lateral_accel = max_lateral_accel;
        return *this;
    }
};

/**
 * @brief Abstract base class for speed ramping
 * 
 * This class defines the interface for speed ramping systems.
 * Ramping provides:
 * - Smooth speed transitions
 * - Acceleration control
 * - Speed limiting
 * - Motion profiling
 * 
 * Implementations can provide different ramping behaviors:
 * - Linear ramping
 * - S-curve profiles
 * - Trapezoidal profiles
 * - Custom acceleration curves
 */
class Ramp {
public:
    /**
     * @brief Virtual destructor
     */
    virtual ~Ramp() = default;

    /**
     * @brief Starts the ramping process
     * 
     * Initializes ramping from a given speed.
     * 
     * @param initialSpeed Starting speed
     */
    virtual void start(double initialSpeed) = 0;

    /**
     * @brief Computes speed change
     * 
     * Calculates the speed change for the current
     * time step based on ramping parameters.
     * 
     * @return double Speed delta
     */
    virtual double computeDelta() = 0;

    /**
     * @brief Gets current speed
     * @return double Current speed
     */
    virtual double getCurrentSpeed() = 0;

    /**
     * @brief Stops the ramping process
     * 
     * Initiates a controlled stop, respecting
     * acceleration limits.
     */
    virtual void stop() = 0;
};



#endif //RAMP_H
