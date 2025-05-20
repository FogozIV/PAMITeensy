//
// Created by fogoz on 19/05/2025.
//

#include "utils/TaskScheduler.h"
[[noreturn]] void panic(const char* msg) {
    // Print message to debug console (adjust depending on your platform)
    printf("PANIC: %s\n", msg);

    // Halt the system
    chSysHalt(msg);
}
TaskScheduler::TaskScheduler() {
    chPoolObjectInit(&threadPool, sizeof(threadStacks[0]), nullptr);
    chPoolLoadArray(&threadPool, threadStacks, THREAD_POOL_SIZE);

}

void TaskScheduler::schedule(std::chrono::milliseconds delay, std::function<void()> fn) {
    auto runAt = std::chrono::steady_clock::now() + delay;

    std::unique_lock lock(mutex);
    tasks.push({runAt, std::move(fn)});
    cond.notify_one();
}

void TaskScheduler::init() {
    if (!is_init) {
        Serial.println("Initializing Task Scheduler");

        // Start worker threads
        for (int i = 0; i < THREAD_POOL_SIZE; i++) {
            Serial.printf("Starting thread %d\r\n", i);
            void* wa = chPoolAlloc(&threadPool);
            if (!wa) panic("No stack!");
            char chars[] = "Thread Pool %d";
            char buffer[100];
            snprintf(buffer, sizeof(buffer), chars, i);

            chThdCreateFromMemoryPool(&threadPool, buffer, NORMALPRIO, &workerThreadFunc, this);
        }
    }
}
