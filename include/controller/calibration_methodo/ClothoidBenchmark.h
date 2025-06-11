//
// Created by fogoz on 12/06/2025.
//

#ifndef CLOTHOIDBENCHMARK_H
#define CLOTHOIDBENCHMARK_H

#include <controller/calibration_methodo/BaseCalibrationMethodo.h>

class ClothoidBenchmark : public CalibrationMethodo {
public:
    void save() override;

    void printStatus(Stream &stream) override;
};

#endif //CLOTHOIDBENCHMARK_H
