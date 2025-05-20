//
// Created by fogoz on 19/05/2025.
//
#include "utils/MutexWrapper.h"

MutexWrapper::MutexWrapper() {
    chMtxObjectInit(&mutex);
}

void MutexWrapper::lock() {
    chMtxLock(&mutex);
}

void MutexWrapper::unlock() {
    chMtxUnlock(&mutex);
}

bool MutexWrapper::try_lock() {
    return chMtxTryLock(&mutex);
}

mutex_t & MutexWrapper::getNativeMutex() {
    return mutex;
}
