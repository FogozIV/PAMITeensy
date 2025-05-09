//
// Created by fogoz on 09/05/2025.
//

#include "../../include/utils/TaskScheduler.h"
#include "Arduino.h"
using namespace std::chrono;


TaskScheduler::TaskScheduler(std::shared_ptr<ThreadPool> threadPool): threadPool(threadPool) {
}

void TaskScheduler::update() {
    for (auto it = tasks.begin(); it != tasks.end();) {
        if (steady_clock::now() >= it->start) {
            if (threadPool != nullptr) {
                threadPool->addTask(it->callback);
            }else {
                it->callback();
            }
            if (it->repeat) {
                it->start = it->start + duration_cast<steady_clock::duration>(it->interval);
                ++it;
            } else {
                it = tasks.erase(it);
            }
        }else {
            ++it;
        }
    }
}
