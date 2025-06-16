//
// Created by fogoz on 16/06/2025.
//

#ifndef RECURSIVELEASTSQUARE_H
#define RECURSIVELEASTSQUARE_H
#include <functional>
#include <optional>

#include "utils/Matrix.h"

template<size_t N>
class RecursiveLeastSquare {
    Matrix<N, N> ATA = Matrix<N, N>::zero();
    Matrix<N, 1> ATb = Matrix<N, 1>::zero();
public:
    void addMatrix(Matrix<N, N> A, Matrix<N, 1> B) {
        ATA += A;
        ATb += B;
    }

    void addEquation(const Matrix<N, 1>& coeffs, double rhs) {
        ATA += coeffs * coeffs.transpose();
        ATb += coeffs * rhs;
    }

    std::optional<Matrix<N, 1>> computeResult() {
        auto res = ATA.inverse();
        if (!res.has_value()) {
            res = std::nullopt;
        }
        auto inversed = res.value();
        return inversed * ATb;
    }

    void reset() {
        ATA = Matrix<N, N>::zero();
        ATb = Matrix<N, 1>::zero();
    }

};

#endif //RECURSIVELEASTSQUARE_H
