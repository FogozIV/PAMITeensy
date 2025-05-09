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
class TaskScheduler {
    struct Task {
        steady_clock::time_point start;
        duration<double> interval = duration<double>(0);
        std::function<void()> callback;
        bool repeat;
        template<typename T, typename K>
        Task(duration<T, K> start, std::function<void()> callback){
            this->start = steady_clock::now() + duration_cast<steady_clock::duration>(start);
            this->callback = callback;
            this->repeat = false;
        }
        template<typename T, typename K>
        Task(duration<T,K> start, std::function<void()> callback, duration<double> interval): Task(start, callback) {
            this->interval = interval;
            this->repeat = true;
        }
    };
    std::shared_ptr<ThreadPool> threadPool;
    std::vector<Task> tasks;
public:
    TaskScheduler(std::shared_ptr<ThreadPool> threadPool);
    template<typename T, typename K>
    void addTask(duration<T,K> callback_in, std::function<void()> callback, duration<double> interval) {
        tasks.emplace_back(callback_in, callback, interval);
    }
    template<typename T, typename K>
    void addTask(duration<T,K> callback_in, std::function<void()> callback) {
        tasks.emplace_back(callback_in, callback);
    }
    void update();
};



#endif //TASKSCHEDULER_H
