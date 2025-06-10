//
// Created by fogoz on 21/05/2025.
//

#include "encoders/MotoEncoderParameterEstimation.h"

#include "utils/StreamSplitter.h"

void RecursiveLeastSquares::update(const DataPoint &s) {

    double tl = left_wheel * s.tic_left;
    double tr = right_wheel * s.tic_right;

    double d = s.distance;
    double a = s.angle;
    std::array<double, 3> a_i = {tl, tr, 0.0};
    double b_i = 2*d;
    // First equation: tl * x + tr * y = 2*d
    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
            ATA(r,c) += a_i[r] * a_i[c];
        }
        ATb(r, 0) += a_i[r] * b_i;
    }
    // Second equation: tl * x - tr * y + a * z = 0
    if (std::abs(a) > 1e-6) {
        a_i = {-tl/(a), tr/(a), -1};
        b_i = 0;
        for (int r = 0; r < 3; r++) {
            for (int c = 0; c < 3; c++) {
                ATA(r,c) += a_i[r] * a_i[c];
            }
            ATb(r, 0) += a_i[r] * b_i;
        }
    }
    ATA *= 0.5;
    ATb *= 0.5;
}

void RecursiveLeastSquares::add(const DataPoint &s) {
    this->data.push_back(s);
}

std::shared_ptr<PositionParameters> RecursiveLeastSquares::getParams() {
    recompute:
    for (auto data: this->data) {
        update(data);
    }
    for (int i = 0; i < 3; i++) {
        ATA(i, i) += 1e-6;  // Small diagonal bias
    }
    auto ATA_inv = ATA.inverse();
    if (!ATA_inv.has_value()) {
        return nullptr;
    }
    auto p = ATA_inv.value() * ATb;
    bool changed = false;
    if (p(0,0) < 0) {
        left_wheel *= -1;
        changed = true;
    }
    if (p(1,0) < 0) {
        right_wheel *= -1;
        changed = true;
    }
    if (changed) {
        ATA.clear();
        ATb.clear();
        goto recompute;
    }
    return std::make_shared<PositionParameters>(p(2,0), left_wheel * p(0,0), right_wheel * p(1,0));
}
