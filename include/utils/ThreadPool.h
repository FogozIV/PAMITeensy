//
// Created by fogoz on 09/05/2025.
//

#ifndef THREADPOOL_H
#define THREADPOOL_H
#include <functional>
#include <queue>
#include <TeensyThreads.h>

#include "SimpleSemaphore.h"


class ThreadPool {
    std::queue<std::function<void()>> tasks = {};
    std::mutex queueMutex;
    SimpleSemaphore taskAvailable;
    std::vector<int> threadIds ={};
    bool running = true;

    static void workerLoop(void* arg);

    void run();
public:
    ThreadPool(int nbThreads = 4);

    void addTask(std::function<void()> task);

    void stop();
};


#endif //THREADPOOL_H
