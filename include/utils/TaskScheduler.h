//
// Created by fogoz on 19/05/2025.
//

#ifndef TASKSCHEDULER_H
#define TASKSCHEDULER_H
#include <ChRt.h>
#include <algorithm>
#include <chrono>
#include <mutex>
#include <queue>

#include "ConditionVariableWrapper.h"
#include "MutexWrapper.h"
#define THREAD_POOL_SIZE 4
#define THREAD_STACK_SIZE 512

struct ScheduledTask {
    std::chrono::steady_clock::time_point runAt;
    std::function<void()> fn;

    // For priority queue ordering (earliest runAt first)
    bool operator<(const ScheduledTask& other) const {
        return runAt > other.runAt; // reversed for min-heap
    }
};

class TaskScheduler {
    memory_pool_t threadPool;
    stkalign_t threadStacks[THREAD_POOL_SIZE][THREAD_STACK_SIZE / sizeof(stkalign_t)];
    bool is_init=  false;
public:
    TaskScheduler();

    // Schedule a task to run after a delay
    void schedule(std::chrono::milliseconds delay, std::function<void()> fn);

    void init();

private:
    std::priority_queue<ScheduledTask> tasks;
    MutexWrapper mutex;
    ConditionVariableWrapper cond;

    static THD_FUNCTION(workerThreadFunc, arg) {
        auto* scheduler = static_cast<TaskScheduler*>(arg);
        chRegSetThreadName("Worker");

        while (true) {
            std::function<void()> taskToRun;

            {
                std::unique_lock lock(scheduler->mutex);
                while (scheduler->tasks.empty()) {
                    // Wait until a task is available
                    scheduler->cond.wait(lock);
                }

                // Check next task time
                auto now = std::chrono::steady_clock::now();
                auto& nextTask = scheduler->tasks.top();

                if (nextTask.runAt <= now) {
                    taskToRun = std::move(nextTask.fn);
                    scheduler->tasks.pop();
                } else {
                    // Wait until next task's scheduled time or new task arrival
                    auto waitTime = nextTask.runAt - now;
                    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(waitTime);
                    scheduler->cond.wait_for(lock, duration);
                    continue;  // re-check the condition
                }
            }

            // Run task outside the lock
            if (taskToRun) {
                taskToRun();
            }
        }
    }
};
#endif //TASKSCHEDULER_H
