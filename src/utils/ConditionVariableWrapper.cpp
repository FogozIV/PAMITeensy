//
// Created by fogoz on 19/05/2025.
//

#include "utils/ConditionVariableWrapper.h"

ConditionVariableWrapper::ConditionVariableWrapper() {
    chCondObjectInit(&cond);
}

void ConditionVariableWrapper::wait(MutexWrapper &mutex) {
    mutex.unlock();
    chCondWait(&cond);
    mutex.lock();
}

void ConditionVariableWrapper::wait(std::unique_lock<MutexWrapper> &lock) {
    lock.unlock();
    chCondWait(&cond);
    lock.lock();
}

void ConditionVariableWrapper::notify_one() {
    chCondSignal(&cond);
}

void ConditionVariableWrapper::notify_all() {
    chCondBroadcast(&cond);
}

bool ConditionVariableWrapper::wait_for(std::unique_lock<MutexWrapper> &lock, std::chrono::milliseconds timeout) {
    // Convert timeout to systime_t (ChibiOS tick count)
    systime_t timeoutTicks = TIME_MS2I(timeout.count());

    lock.unlock();
    msg_t result = chCondWaitTimeout(&cond, timeoutTicks);
    lock.lock();

    return result == MSG_OK; // true if notified, false if timeout
}
