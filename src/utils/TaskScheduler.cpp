//
// Created by fogoz on 09/05/2025.
//

#include "../../include/utils/TaskScheduler.h"
#include "Arduino.h"
using namespace std::chrono;


TaskScheduler::TaskScheduler(std::shared_ptr<ThreadPool> threadPool): threadPool(threadPool) {

}

void TaskScheduler::update() {
    taskMutex.lock();
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
    //Locking the unloaded tasks to insert them in the tasks array
    unloadedTaskMutex.lock();
    //Inserting all new tasks into the tasks array
    tasks.insert(tasks.end(), unloadedTask.begin(), unloadedTask.end());
    //Clearing the temp task array
    unloadedTask.clear();
    //Unlocking all mutex to resume normal behavior
    unloadedTaskMutex.unlock();
    taskMutex.unlock();

}

bool TaskScheduler::deleteTaskId(uint64_t taskId) {
    taskMutex.lock();
    for(auto it = tasks.begin(); it != tasks.end(); it++){
        if(it->id == taskId){
            //We found task so we erase it
            tasks.erase(it);
            break;
        }
    }
    taskMutex.unlock();
    return false;
}
