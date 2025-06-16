//
// Created by fogoz on 21/05/2025.
//

#ifndef MOTOENCODERPARAMETERESTIMATION_H
#define MOTOENCODERPARAMETERESTIMATION_H
#include "utils/PositionManager.h"

/**
 * @brief Data point for encoder parameter estimation
 * 
 * This structure holds a single measurement point used in
 * encoder calibration, containing:
 * - Linear distance traveled
 * - Angular rotation
 * - Left and right encoder tick counts
 */
struct DataPoint {
    double distance;    ///< Linear distance traveled
    double angle;      ///< Angular rotation
    int32_t tic_left;  ///< Left encoder tick count
    int32_t tic_right; ///< Right encoder tick count
};

/**
 * @brief Recursive least squares estimator for encoder parameters
 * 
 * This class implements a recursive least squares algorithm to estimate:
 * - Wheel diameters
 * - Track width
 * - Encoder scaling factors
 * 
 * The estimation is based on:
 * - Collected motion data points
 * - Encoder measurements
 * - Known robot geometry
 * 
 * The algorithm provides:
 * - Online parameter updates
 * - Minimal memory usage
 * - Fast convergence
 * - Robust estimation
 */
class RecursiveLeastSquares3x3 {
    std::vector<DataPoint> data;     ///< Collected data points
    Matrix<3,3> ATA;                 ///< Normal equation matrix
    Matrix<3,1> ATb;                 ///< Normal equation vector
    double left_wheel = 1;           ///< Left wheel diameter estimate
    double right_wheel = 1;          ///< Right wheel diameter estimate

    /**
     * @brief Updates estimation matrices with new data
     * @param s New data point
     */
    void update(const DataPoint &s);

public:
    /**
     * @brief Adds a new data point for estimation
     * @param s Data point to add
     */
    void add(const DataPoint &s);

    /**
     * @brief Gets the estimated parameters
     * @return std::shared_ptr<PositionParameters> Estimated parameters
     */
    std::shared_ptr<PositionParameters> getParams();
};

#endif //MOTOENCODERPARAMETERESTIMATION_H
