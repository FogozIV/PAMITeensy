//
// Created by fogoz on 24/04/2025.
//

#ifndef PAMITEENSY_PAMIROBOT_H
#define PAMITEENSY_PAMIROBOT_H

#include <list>
#include "robot/BaseRobot.h"
#include "utils/PositionManager.h"
#include "basic_controller/PID.h"
#include "controller/SimpleTripleBasicController.h"
#include "chrono"
#include <FS.h>
#include <queue>
#include <SD.h>

#include "utils/KalmanFiltering.h"


namespace KalmanFilter{
    enum KalmanStates {
        s,
        v,
        theta,
        omega,
        v_L,
        v_R
    };
}
/**
 * @brief Concrete implementation of BaseRobot for PAMI robot platform
 * 
 * This class implements a differential drive robot using:
 * - PID controllers for motion control
 * - Quadrature encoders for position feedback
 * - PWM-based motor control
 * - SD card configuration storage
 * - Thread-safe operation
 * 
 * The robot uses three BasicController:
 * - Distance BasicController: Controls linear motion
 * - Angle BasicController: Controls rotational motion
 * - Distance-Angle BasicController: Controls combined motion
 */
class PAMIRobot : public BaseRobot, public std::enable_shared_from_this<PAMIRobot> {
protected:
    std::queue<std::shared_ptr<BaseTarget>> targets = {};  ///< Queue of motion targets
    std::chrono::time_point<std::chrono::steady_clock, std::chrono::duration<double, std::ratio<1,1>>> previous_time;  ///< Last control loop timestamp
    double dt = 0.005;  ///< Control loop time step (seconds)

    std::shared_ptr<BasicController> controllerDistance;        ///< Linear motion BasicController
    std::shared_ptr<BasicController> controllerAngle;           ///< Angular motion BasicController
    std::shared_ptr<BasicController> controllerDistanceAngle;   ///< Combined motion BasicController

    std::shared_ptr<TripleBasicParameters> pidParameters;  ///< PID controller parameters

    Mutex targetMutex;  ///< Mutex for thread-safe target queue access



    std::shared_ptr<KalmanFiltering<6,2>> kalmanFilter = nullptr;



    virtual Matrix<6,6> makeA();

    virtual Matrix<2,6> makeH();

    virtual Matrix<6,6> makeQ();

    virtual Matrix<2,2> makeR();

    virtual double getState(KalmanFilter::KalmanStates state);



public:
    explicit PAMIRobot(std::shared_ptr<Mutex> motorUpdate = nullptr);
    /**
     * @brief Gets the control loop time step
     * @return double Time step in seconds
     */
    double getDT() override;

    /**
     * @brief Processes the next target in the queue
     * 
     * Executes the current target's state machine and removes
     * completed targets from the queue.
     */
    void computeTarget() override;

    /**
     * @brief Updates the robot's position
     * 
     * Computes new position based on encoder readings and
     * updates both wheel and robot positions.
     */
    void computePosition() override;

    /**
     * @brief Executes the control loop
     * 
     * Runs the PID controllers to generate motor commands
     * based on current position and target.
     */
    void computeController() override;

    /**
     * @brief Adds a new motion target to the queue
     * @param target Target to add
     */
    void addTarget(std::shared_ptr<BaseTarget> target) override;

    /**
     * @brief Main computation loop
     * 
     * Updates timing, position, and executes control loop
     * if control is enabled.
     */
    void compute() override;

    /**
     * @brief Initializes the robot
     * 
     * Sets up:
     * - Encoders
     * - Motors
     * - PID controllers
     * - Position managers
     * - Speed estimators
     * Loads configuration from SD card if available
     */
    void init() override;

    /**
     * @brief Saves current configuration
     * @return bool True if save successful
     */
    bool save() override;

    /**
     * @brief Saves configuration to specified file
     * @param filename Target filename
     * @return bool True if save successful
     */
    bool save(const char *filename);

    void controllerClear() override;

    /**
     * @brief Resets robot position
     * @param pos New position
     */
    void reset_to(Position pos) override;

    /**
     * @brief Clears all pending targets
     */
    void clearTarget() override;

    /**
     * @brief Gets number of pending targets
     * @return size_t Target count
     */
    size_t getTargetCount() override;

    /**
     * @brief Registers robot-specific commands
     * @param parser Command parser to register with
     */
    void registerCommands(CommandParser &parser) override;

    std::shared_ptr<BasicController> getControllerDistance() const;

    std::shared_ptr<BasicController> getControllerAngle() const;

    std::shared_ptr<BasicController> getControllerDistanceAngle() const;

    void setControllerDistance(std::shared_ptr<BasicController> pidDistance);

    void setControllerAngle(std::shared_ptr<BasicController> pidAngle);

    void setControllerDistanceAngle(std::shared_ptr<BasicController> pidDistanceAngle);

    std::shared_ptr<TripleBasicParameters> getPIDParameters() const;

    void update(double left, double right) override;

    double getTranslationalOtherEstimatedSpeed() override;

    Angle getRotationalOtherEstimatedSpeed() override;
};

#endif //PAMITEENSY_PAMIROBOT_H
