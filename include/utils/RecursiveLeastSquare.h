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

    void addEquation(std::array<double, N> coeffs, double rhs){
        Matrix<N, 1> m;
        for(size_t i = 0; i < N; i++){
            m(i, 0) = coeffs[i];
        }
        addEquation(m, rhs);
    }

    void addEquation(Matrix<N, 1> coeffs, double rhs) {
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

template<size_t N>
class NonLinearRecursiveLeastSquare {
    RecursiveLeastSquare<N> rls;
    std::function<std::vector<double>(const Matrix<N, 1>& x)> residual;
    std::function<std::vector<Matrix<N, 1>>(const Matrix<N, 1>& x)> jacobian;
    Matrix<N, 1> x;
public:
    NonLinearRecursiveLeastSquare(std::function<std::vector<double>(const Matrix<N, 1>& x)> residual, std::vector<std::function<Matrix<N, 1>>(const Matrix<N, 1>& x)> jacobian, Matrix<N, 1> x=Matrix<N,1>::zero()): residual(residual), jacobian(jacobian), x(x) {

    }

    void updateX(const Matrix<N, 1>& x) {
        this->x = x;
        this->reset();
        auto jac = jacobian(x);
        auto res = residual(x);
        for (size_t i = 0; i < std::min(jac.size(), res.size()); i++) {
            rls.addEquation(jac[i], res[i]);
        }
    }

    std::optional<Matrix<N, 1>> computeResult() {
        return rls.computeResult();
    }

    std::optional<Matrix<N, 1>> iterate(size_t count=10, double tolerance=1e-6, double damping=1) {
        for (size_t i = 0; i < count; i++) {
            updateX(this->x);
            auto res = rls.computeResult();
            if (!res.has_value()) {
                return std::nullopt;
            }
            x -= res.value() * damping;
            if (res->norm() < tolerance) {
                return this->x;
            }
        }
        return std::nullopt;
    }

    void reset() {
        rls.reset();
    }

};


#endif //RECURSIVELEASTSQUARE_H
