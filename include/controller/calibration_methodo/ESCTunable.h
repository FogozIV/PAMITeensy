//
// Created by fogoz on 03/08/2025.
//

#ifndef ESCTUNABLE_H
#define ESCTUNABLE_H

#include <vector>
#include <controller/calibration_methodo/ESCType.h>

#include "utils/StreamSplitter.h"


class ESCTunable {
protected:
    virtual void init(ESCType::ESC type) = 0;
public:
    std::vector<double*> variables = {};
    std::vector<double> frequency = {};
    std::vector<double> alpha = {};
    std::vector<double> gamma = {};
    std::vector<double> low_bound = {};
    std::vector<double> high_bound = {};

    ESCType::ESC is_init = ESCType::NONE;

    virtual ~ESCTunable() = default;

    void call_init(ESCType::ESC type) {
        if (type!=is_init) {
            variables.clear();
            frequency.clear();
            alpha.clear();
            gamma.clear();
            low_bound.clear();
            high_bound.clear();
            init(type);
            is_init = type;
        }
    }

    void initVariable(double* variable, double freq, double alpha=0.01, double gamma=0.05, double low_bound=0.0001, double high_bound=100) {
        this->variables.emplace_back(variable);
        this->frequency.emplace_back(freq);
        this->alpha.emplace_back(alpha);
        this->gamma.emplace_back(gamma);
        this->low_bound.emplace_back(low_bound);
        this->high_bound.emplace_back(high_bound);
    }

    void initVariable(std::vector<double*> variable, std::vector<double> freq, std::vector<double> alpha, std::vector<double> gamma, std::vector<double> low_bound, std::vector<double> high_bound) {
        this->variables.insert(this->variables.end(), variable.begin(), variable.end());
        this->frequency.insert(this->frequency.end(), freq.begin(), freq.end());
        this->alpha.insert(this->alpha.end(), alpha.begin(), alpha.end());
        this->gamma.insert(this->gamma.end(), gamma.begin(), gamma.end());
        this->low_bound.insert(this->low_bound.end(), low_bound.begin(), low_bound.end());
        this->high_bound.insert(this->high_bound.end(), high_bound.begin(), high_bound.end());
    }

    virtual std::vector<double> getGains(ESCType::ESC type) {
        call_init(type);
        std::vector<double> gains;
        for (size_t i = 0; i < variables.size(); i++) {
            gains.emplace_back(*variables[i]);
        }
        return gains;
    };

    virtual size_t final_update(ESCType::ESC type, std::vector<double> initialGain, std::vector<double> multiplier) {
        call_init(type);
        for (size_t i = 0; i < variables.size(); i++) {
            *variables[i] = initialGain[i] - gamma[i] * multiplier[i];
            *variables[i] = constrain(*variables[i], low_bound[i], high_bound[i]);
            streamSplitter.printf("Variable %d is updated to %f\r\n", i, *variables[i]);
        }
        return variables.size();
    }

    virtual std::vector<std::pair<double, double>> update_gains(ESCType::ESC type, std::vector<double> initialGain, double t) {
        call_init(type);
        std::vector<std::pair<double, double>> gains;
        for (size_t i = 0; i < variables.size(); i++) {
            *variables[i] = initialGain[i] + alpha[i] * initialGain[i] * cos(2*PI*(t /*+ base_robot->getDT()*/)*frequency[i]);
            gains.emplace_back(cos(2*PI*t*frequency[i]), sin(2*PI*t*frequency[i]));
        }
        return gains;
    };

    virtual size_t setGains(ESCType::ESC type, std::vector<double> gains) {
        call_init(type);
        for (size_t i = 0; i < variables.size(); i++) {
            *variables[i] = gains[i];
        }
        return variables.size();
    }

    virtual void setAlpha(ESCType::ESC type, double alpha) {
        call_init(type);
        for (size_t i = 0; i < variables.size(); i++) {
            this->alpha[i] = alpha;
        }
    }

    virtual void setGamma(ESCType::ESC type, double gamma) {
        call_init(type);
        for (size_t i = 0; i < this->gamma.size(); i++) {
            this->gamma[i] = gamma;
        }
    }
};

#endif //ESCTUNABLE_H
