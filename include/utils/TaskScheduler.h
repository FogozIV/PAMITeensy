//
// Created by fogoz on 09/05/2025.
//

#ifndef TASKSCHEDULER_H
#define TASKSCHEDULER_H
#include <chrono>
#include <functional>
#include <memory>

#include "ThreadPool.h"


using namespace std::chrono;

/**
 * @brief Task scheduling and execution manager
 * 
 * This class provides functionality for:
 * - Scheduling tasks to run at specific times
 * - Periodic task execution
 * - Thread pool integration
 * - Task management
 * 
 * Tasks can be scheduled as:
 * - One-time execution
 * - Periodic execution with fixed intervals
 */
class TaskScheduler {
    /**
     * @brief Internal task representation
     * 
     * Holds information about a scheduled task including:
     * - Execution time
     * - Callback function
     * - Interval for periodic tasks
     * - Repeat flag
     */
    struct Task {
        steady_clock::time_point start;      ///< Task execution time
        duration<double> interval{0};        ///< Interval for periodic tasks
        std::function<void()> callback;      ///< Task callback function
        bool repeat;                         ///< Periodic execution flag

        /**
         * @brief Constructs a one-time task
         * 
         * @tparam T Duration value type
         * @tparam K Duration period type
         * @param start Time until task execution
         * @param callback Function to execute
         */
        template<typename T, typename K>
        Task(duration<T, K> start, std::function<void()> callback) {
            this->start = steady_clock::now() + duration_cast<steady_clock::duration>(start);
            this->callback = callback;
            this->repeat = false;
        }

        /**
         * @brief Constructs a periodic task
         * 
         * @tparam T Duration value type
         * @tparam K Duration period type
         * @param start Time until first execution
         * @param callback Function to execute
         * @param interval Time between executions
         */
        template<typename T, typename K>
        Task(duration<T,K> start, std::function<void()> callback, duration<double> interval)
            : Task(start, callback) {
            this->interval = interval;
            this->repeat = true;
        }
    };

    std::shared_ptr<ThreadPool> threadPool;  ///< Thread pool for task execution
    std::vector<Task> tasks;                 ///< List of scheduled tasks

public:
    /**
     * @brief Constructs a new task scheduler
     * 
     * @param threadPool Thread pool for executing tasks
     */
    TaskScheduler(std::shared_ptr<ThreadPool> threadPool);

    /**
     * @brief Schedules a periodic task
     * 
     * Adds a task that will execute repeatedly at the specified interval.
     * The first execution occurs after the initial delay.
     * 
     * @tparam T Duration value type
     * @tparam K Duration period type
     * @param callback_in Initial delay before first execution
     * @param callback Function to execute
     * @param interval Time between executions
     */
    template<typename T, typename K>
    void addTask(duration<T,K> callback_in, std::function<void()> callback, duration<double> interval) {
        tasks.emplace_back(callback_in, callback, interval);
    }

    /**
     * @brief Schedules a one-time task
     * 
     * Adds a task that will execute once after the specified delay.
     * 
     * @tparam T Duration value type
     * @tparam K Duration period type
     * @param callback_in Delay before execution
     * @param callback Function to execute
     */
    template<typename T, typename K>
    void addTask(duration<T,K> callback_in, std::function<void()> callback) {
        tasks.emplace_back(callback_in, callback);
    }

    /**
     * @brief Updates the task scheduler
     * 
     * This method should be called regularly to:
     * - Check for tasks that are due
     * - Execute due tasks
     * - Reschedule periodic tasks
     * - Remove completed one-time tasks
     */
    void update();
};



#endif //TASKSCHEDULER_H
