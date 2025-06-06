//
// Created by fogoz on 03/06/2025.
//

#ifndef PAMITEENSY_EVENTNOTIFIERANDWAITER_H
#define PAMITEENSY_EVENTNOTIFIERANDWAITER_H
#include <atomic>
#include <utils/Mutex.h>

class EventNotifierAndWaiter {
private:
    Mutex mutex;
    volatile int version = 0;

public:
    void wait() {
        int my_version;
        mutex.lock();
        my_version = version;
        mutex.unlock();

        while (true) {
            mutex.lock();
            if (version >= my_version) {
                mutex.unlock();
                return;
            }
            mutex.unlock();
            threads.delay_us(100);
        }
    }

    void notify() {
        mutex.lock();
        ++version;
        mutex.unlock();
    }
};
#endif //PAMITEENSY_EVENTNOTIFIERANDWAITER_H
