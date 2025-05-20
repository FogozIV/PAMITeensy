//
// Created by fogoz on 19/05/2025.
//

#ifndef CONDITIONVARIABLEWRAPPER_H
#define CONDITIONVARIABLEWRAPPER_H

#include <mutex>

#include "ChRt.h"
#include "utils/MutexWrapper.h"

class ConditionVariableWrapper {
    condition_variable_t cond;

public:
    ConditionVariableWrapper();

    // Wait without timeout
    void wait(MutexWrapper& mutex);

    void wait(std::unique_lock<MutexWrapper>& lock);

    // Signal one waiting thread
    void notify_one();

    // Signal all waiting threads
    void notify_all();

    bool wait_for(std::unique_lock<MutexWrapper>& lock, std::chrono::milliseconds timeout);
};



#endif //CONDITIONVARIABLEWRAPPER_H
