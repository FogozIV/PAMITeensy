//
// Created by fogoz on 19/05/2025.
//

#ifndef MUTEXWRAPPER_H
#define MUTEXWRAPPER_H

#include "ChRt.h"

class MutexWrapper {
    mutex_t mutex;

public:
    MutexWrapper();

    void lock();

    void unlock();

    bool try_lock();

    mutex_t& getNativeMutex();
};



#endif //MUTEXWRAPPER_H
