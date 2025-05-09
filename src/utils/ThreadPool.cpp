//
// Created by fogoz on 09/05/2025.
//

#include "utils/ThreadPool.h"

ThreadPool::ThreadPool(int nbThreads) {
    for (int i = 0; i < nbThreads; ++i) {
        threadIds.push_back(threads.addThread(workerLoop, this));
    }
}

void ThreadPool::workerLoop(void *arg) {
    static_cast<ThreadPool*>(arg)->run();
}

void ThreadPool::addTask(std::function<void()> task) {
    queueMutex.lock();
    tasks.push(task);
    queueMutex.unlock();
    taskAvailable.signal();
}

void ThreadPool::run() {
    while (running) {
        taskAvailable.wait();
        queueMutex.lock();
        if (tasks.empty()) {
            queueMutex.unlock();
            continue;
        }
        std::function<void()> task = tasks.front();
        tasks.pop();
        queueMutex.unlock();
        task();
    }
}

void ThreadPool::stop() {
    running = false;
    for (int i = 0; i < threadIds.size(); ++i) {
        taskAvailable.signal(); // Signal each thread to stop
    }
}
