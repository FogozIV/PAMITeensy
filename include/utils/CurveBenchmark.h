//
// Created by fogoz on 17/05/2025.
//

#ifndef CURVEBENCHMARK_H
#define CURVEBENCHMARK_H
#include "Arduino.h"
#include <chrono>
#include <memory>

#include "curves/ClothoidCurve.h"
#include "curves/CurveList.h"
#include "utils/G2Solve3Arc.h"
#include "utils/Position.h"

inline void curveBenchmark() {
    Position start(0.0, 0.0, Angle::fromDegrees(0), 0);
    Position end(55, 100, Angle::fromDegrees(180), 0);
    Position end2(-200, 450, Angle::fromDegrees(180), 0.02);
    Position end3(1000, 450, Angle::fromDegrees(180), 0);
    Position end4(1200, 800, Angle::fromDegrees(180), 0);
    G2Solve3Arc arc;
    std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
    std::shared_ptr<CurveList> curveList = std::make_shared<CurveList>();
    int sum = arc.build(start, end);
    curveList->addCurve(std::make_shared<ClothoidCurve>(arc.getSegment0()));
    curveList->addCurve(std::make_shared<ClothoidCurve>(arc.getSegmentMiddle()));
    curveList->addCurve(std::make_shared<ClothoidCurve>(arc.getSegment1()));
    sum += arc.build(end, end2);
    curveList->addCurve(std::make_shared<ClothoidCurve>(arc.getSegment0()));
    curveList->addCurve(std::make_shared<ClothoidCurve>(arc.getSegmentMiddle()));
    curveList->addCurve(std::make_shared<ClothoidCurve>(arc.getSegment1()));
    sum += arc.build(end3, end2);
    arc.reverse();
    curveList->addCurve(std::make_shared<ClothoidCurve>(arc.getSegment0()));
    curveList->addCurve(std::make_shared<ClothoidCurve>(arc.getSegmentMiddle()));
    curveList->addCurve(std::make_shared<ClothoidCurve>(arc.getSegment1()));
    sum += arc.build(end3, end4);
    curveList->addCurve(std::make_shared<ClothoidCurve>(arc.getSegment0()));
    curveList->addCurve(std::make_shared<ClothoidCurve>(arc.getSegmentMiddle()));
    curveList->addCurve(std::make_shared<ClothoidCurve>(arc.getSegment1()));
    std::chrono::steady_clock::time_point end_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed_seconds = end_time - start_time;
    printf("Time elapsed: %f s\r\nSum is : %d", elapsed_seconds.count(), sum);
    start_time = std::chrono::steady_clock::now();
    double norm = 0;
    int i = 0;
    for (double s = 0; s < curveList->getMaxValue(); s+=10.0) {
        Position pos = curveList->getPosition(s);
        norm += pos.norm();
        i ++;
    }
    end_time = std::chrono::steady_clock::now();
    elapsed_seconds = end_time - start_time;
    Serial.printf("Time elapsed: %f s\r\nNorm is : %f\r\n", elapsed_seconds.count(), norm);
    Serial.printf("Number of points calculated : %d\r\n", i);
}
#endif //CURVEBENCHMARK_H
