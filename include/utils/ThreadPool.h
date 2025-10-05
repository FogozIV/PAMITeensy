//
// Created by fogoz on 09/05/2025.
//

#ifndef THREADPOOL_H
#define THREADPOOL_H
#include <functional>
#include <queue>
#include "utils/Mutex.h"
#include "SimpleSemaphore.h"

/**
 * @brief Thread pool for parallel task execution
 * 
 * This class provides a thread pool implementation for Teensy that:
 * - Manages a pool of worker threads
 * - Handles task queuing and distribution
 * - Provides thread-safe task submission
 * - Supports graceful shutdown
 * 
 * The thread pool is designed for:
 * - Parallel task execution
 * - Load balancing
 * - Resource management
 * - Efficient task processing
 */
class ThreadPool {
    std::queue<std::function<void()>> tasks = {};  ///< Queue of pending tasks
    Mutex queueMutex;                         ///< Mutex for thread-safe queue access
    SimpleSemaphore taskAvailable;                 ///< Semaphore for task availability signaling
    std::vector<int> threadIds = {};               ///< IDs of worker threads
    bool running = true;                           ///< Thread pool running state

    /**
     * @brief Worker thread function
     * 
     * This static method runs in each worker thread and:
     * - Waits for available tasks
     * - Executes tasks from the queue
     * - Handles shutdown signals
     * 
     * @param arg Pointer to ThreadPool instance
     */
    static void workerLoop(void* arg);

    /**
     * @brief Main thread pool execution loop
     * 
     * Internal method that:
     * - Manages worker threads
     * - Distributes tasks
     * - Handles synchronization
     */
    void run();

public:
    /**
     * @brief Constructs a new thread pool
     * 
     * Creates and initializes a pool of worker threads.
     * 
     * @param nbThreads Number of worker threads to create (default: 4)
     */
    ThreadPool(int nbThreads = 4);

    /**
     * @brief Adds a task to the pool
     * 
     * Submits a task for execution by the thread pool.
     * Tasks are executed in FIFO order by available threads.
     * 
     * @param task Function to execute
     */
    void addTask(std::function<void()> task);

    /**
     * @brief Stops the thread pool
     * 
     * Initiates a graceful shutdown of the thread pool:
     * - Prevents new task submissions
     * - Waits for queued tasks to complete
     * - Signals worker threads to exit
     * - Cleans up resources
     */
    void stop();
};


#endif //THREADPOOL_H
