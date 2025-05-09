//
// Created by fogoz on 09/05/2025.
//

#ifndef SIMPLESEMAPHORE_H
#define SIMPLESEMAPHORE_H
#include <TeensyThreads.h>

class SimpleSemaphore {
private:
    volatile int count;
    std::mutex mutex;
public:
    SimpleSemaphore(int count = 0) : count(count) {}

    void signal() {
        mutex.lock();
        ++count;
        mutex.unlock();
    }

    void wait() {
        while (true) {
            mutex.lock();
            while (count > 0) {
                --count;
                mutex.unlock();
                return;
            }
            mutex.unlock();
            threads.delay_us(100);
        }
    }

};
#endif //SIMPLESEMAPHORE_H
