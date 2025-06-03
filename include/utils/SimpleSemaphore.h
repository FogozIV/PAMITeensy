//
// Created by fogoz on 09/05/2025.
//

#ifndef SIMPLESEMAPHORE_H
#define SIMPLESEMAPHORE_H
#include <TeensyThreads.h>
#include "Mutex.h"

/**
 * @brief Simple counting semaphore implementation
 * 
 * This class provides a basic counting semaphore for thread synchronization.
 * It supports:
 * - Thread-safe signaling
 * - Blocking wait operations
 * - Resource counting
 * - Multi-thread coordination
 * 
 * The semaphore is useful for:
 * - Producer-consumer patterns
 * - Resource management
 * - Task synchronization
 * - Event signaling
 */
class SimpleSemaphore {
private:
    volatile int count;     ///< Current semaphore count
    Mutex mutex;       ///< Mutex for thread-safe operations

public:
    /**
     * @brief Constructs a new semaphore
     * 
     * @param count Initial semaphore count (default: 0)
     */
    SimpleSemaphore(int count = 0) : count(count) {}

    /**
     * @brief Signals the semaphore
     * 
     * Increments the semaphore count, potentially
     * unblocking a waiting thread. This operation
     * is atomic and thread-safe.
     */
    void signal() {
        mutex.lock();
        ++count;
        mutex.unlock();
    }

    /**
     * @brief Waits on the semaphore
     * 
     * Blocks until the semaphore count is greater than zero,
     * then decrements the count and returns. This operation
     * uses a polling approach with a short delay to reduce
     * CPU usage while waiting.
     */
    void wait() {
        while (true) {
            mutex.lock();
            while (count > 0) {
                --count;
                mutex.unlock();
                return;
            }
            mutex.unlock();
            threads.delay_us(100);  // Short delay to reduce CPU usage
        }
    }
};

#endif //SIMPLESEMAPHORE_H
