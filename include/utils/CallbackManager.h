//
// Created by fogoz on 03/06/2025.
//

#ifndef PAMITEENSY_CALLBACKMANAGER_H
#define PAMITEENSY_CALLBACKMANAGER_H
#include "Arduino.h"
#include "functional"
#include <utils/Mutex.h>
class Callback{
protected:
    std::function<void()> fct;
    uint64_t id;
public:
    Callback(std::function<void()> fct, uint64_t id) : fct(fct), id(id){

    }
    bool operator==(const Callback& callback){
        return this->id == callback.id;
    }

    void operator()(){
        if(this->fct)
            this->fct();
    }

    uint64_t getID(){
        return id;
    }

};

class CallbackManager{
protected:
    std::vector<Callback> callbacks;
    uint64_t current_counter = 0;
    Mutex mutex;
public:
    void call(){
        lock_guard lg(mutex);
        for(auto callback : callbacks){
            callback();
        }
    }

    uint64_t addCallback(std::function<void()> fct){
        lock_guard lg(mutex);
        callbacks.emplace_back(fct, current_counter);
        return current_counter++;
    }

    void removeCallback(uint64_t id){
        lock_guard lg(mutex);
        auto ptr = std::find_if(callbacks.begin(), callbacks.end(),[id](auto data){
            return id == data.getID();
        });
        callbacks.erase(ptr);
    }


};

#endif //PAMITEENSY_CALLBACKMANAGER_H
