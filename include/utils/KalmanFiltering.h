//
// Created by fogoz on 07/06/2025.
//

#ifndef KALMANFILTERING_H
#define KALMANFILTERING_H
#include "utils/Matrix.h"


template<size_t stateCount, size_t measurementCount>
class KalmanFiltering {
protected:
    Matrix<stateCount,1> stateVectorX = Matrix<stateCount,1>::zero();
    Matrix<stateCount,stateCount> stateTransitionA;
    Matrix<stateCount,stateCount> stateCovarianceP = Matrix<stateCount,stateCount>::identity();
    Matrix<stateCount,stateCount> processNoiseQ;
    Matrix<measurementCount,stateCount> measurementMatrixH;
    Matrix<measurementCount, measurementCount> measurementNoiseR;
    Matrix<stateCount,measurementCount> kalmanGainK;
    Matrix<stateCount, stateCount> identity = Matrix<stateCount, stateCount>::identity();
    Matrix<measurementCount, 1> residualVector = Matrix<measurementCount, 1>::zero();
public:

    KalmanFiltering(
    const Matrix<stateCount, stateCount>& A,
    const Matrix<measurementCount, stateCount>& H,
    const Matrix<stateCount, stateCount>& Q,
    const Matrix<measurementCount, measurementCount>& R,
    const Matrix<stateCount, 1>& initialState = Matrix<stateCount, 1>::zero(),
    const Matrix<stateCount, stateCount>& initialP = Matrix<stateCount, stateCount>::identity()
) :
    stateVectorX(initialState),
    stateTransitionA(A),
    stateCovarianceP(initialP),
    processNoiseQ(Q),
    measurementMatrixH(H),
    measurementNoiseR(R)
    {}


    Matrix<stateCount, 1>  computeWithMeasurement(Matrix<measurementCount,1> measurementVectorZ);

    Matrix<stateCount, 1>  getResiduals();

    void setA(Matrix<stateCount, stateCount> A);

    void update(
        const Matrix<stateCount, stateCount>& A,
        const Matrix<measurementCount, stateCount>& H,
        const Matrix<stateCount, stateCount>& Q,
        const Matrix<measurementCount, measurementCount>& R);

    double getState(size_t x) const;

    double getResidual(size_t x);

};

template<size_t stateCount, size_t measurementCount>
Matrix<stateCount, 1> KalmanFiltering<stateCount, measurementCount>::computeWithMeasurement(
    Matrix<measurementCount, 1> measurementVectorZ) {
    // Predict
    stateVectorX = stateTransitionA * stateVectorX;
    stateCovarianceP = stateTransitionA * stateCovarianceP * stateTransitionA.transpose() + processNoiseQ;

    // Update
    residualVector = measurementVectorZ - measurementMatrixH * stateVectorX;
    auto S = measurementMatrixH * stateCovarianceP * measurementMatrixH.transpose() + measurementNoiseR;
    auto invS = S.inverse();
    if (!invS.has_value()) {
        // Don't update — skip or reset
        return stateVectorX;  // prediction-only step
    }
    auto SInv = invS.value();
    kalmanGainK = stateCovarianceP * measurementMatrixH.transpose() * SInv;

    stateVectorX = stateVectorX + kalmanGainK * residualVector;
    stateCovarianceP = (identity - kalmanGainK * measurementMatrixH) * stateCovarianceP;
    return stateVectorX;
}

template<size_t stateCount, size_t measurementCount>
Matrix<stateCount, 1> KalmanFiltering<stateCount, measurementCount>::getResiduals() {
    return residualVector;//
}

template<size_t stateCount, size_t measurementCount>
void KalmanFiltering<stateCount, measurementCount>::setA(Matrix<stateCount, stateCount> A) {
    this->stateTransitionA = A;
}

template<size_t stateCount, size_t measurementCount>
void KalmanFiltering<stateCount, measurementCount>::update(const Matrix<stateCount, stateCount> &A,
    const Matrix<measurementCount, stateCount> &H, const Matrix<stateCount, stateCount> &Q,
    const Matrix<measurementCount, measurementCount> &R) {
    this->stateTransitionA = A;
    this->measurementMatrixH = H;
    this->processNoiseQ = Q;
    this->measurementNoiseR = R;
}

template<size_t stateCount, size_t measurementCount>
double KalmanFiltering<stateCount, measurementCount>::getState(size_t x) const {
    assert(x < stateCount);
    return stateVectorX(x, 0);
}

template<size_t stateCount, size_t measurementCount>
double KalmanFiltering<stateCount, measurementCount>::getResidual(size_t x) {
    assert(x < measurementCount);
    return residualVector(x, 0);
}


#endif //KALMANFILTERING_H
