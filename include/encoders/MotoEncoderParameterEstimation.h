//
// Created by fogoz on 21/05/2025.
//

#ifndef MOTOENCODERPARAMETERESTIMATION_H
#define MOTOENCODERPARAMETERESTIMATION_H
#include "utils/PositionManager.h"


struct DataPoint {
    double distance;
    double angle;
    int32_t tic_left;
    int32_t tic_right;
};

class RecursiveLeastSquares {
    std::vector<DataPoint> data;
    Matrix<3,3> ATA;
    Matrix<3,1> ATb;
    double left_wheel = 1;
    double right_wheel = 1;

    void update(const DataPoint &s);
public:
    void add(const DataPoint &s);
    std::shared_ptr<PositionParameters> getParams();
};

#endif //MOTOENCODERPARAMETERESTIMATION_H
