//
// Created by fogoz on 03/06/2025.
//

#ifndef PAMITEENSY_CALLBACKMANAGER_H
#define PAMITEENSY_CALLBACKMANAGER_H
#include <unordered_set>

#include "Arduino.h"
#include "functional"
#include <utils/Mutex.h>

extern std::shared_ptr<ThreadPool> threadPool;

class Callback {
protected:
    std::function<void()> fct;
    uint64_t id;

public:
    Callback(std::function<void()> fct, uint64_t id) : fct(fct), id(id) {
    }

    bool operator==(const Callback &callback) const{
        return this->id == callback.id;
    }

    void operator()() const{
        if (this->fct) this->fct();
    }

    uint64_t getID() const{
        return id;
    }
};

class CallbackManager {
protected:
    std::vector<Callback> callbacks{}; ///< The vector containing all callbacks
    uint64_t current_counter = 0; ///< The counter for the callback id
    Mutex mutex; ///< the main mutex here to protect all the variables and callbacks
    std::unordered_set<uint64_t> toRemove{}; ///< The toRemove set is just a set of value to remove in the case we have already locked the main mutex
    Mutex removeVectorMutex; ///< The remove vector mutex is there to protect the unordered set of callbacks id to remove it SHOULD NOT be locked without first locking the mutex variable otherwise deadlock can occur
    void unsafeRemove(uint64_t id) {
        auto ptr = std::find_if(callbacks.begin(), callbacks.end(), [id](const auto& data) {
            return id == data.getID();
        });
        if (ptr != callbacks.end()) {
            callbacks.erase(ptr);
        }
    }
public:
    void call() {
        lock_guard lg(mutex);
        for (auto& callback: callbacks) {
            callback();
        }

        lock_guard removeVectorLG(removeVectorMutex);
        for (uint64_t data: toRemove) {
            unsafeRemove(data);
        }
        toRemove.clear();
    }

    uint64_t addCallback(std::function<void()> fct) {
        lock_guard lg(mutex);
        callbacks.emplace_back(std::move(fct), current_counter);
        return current_counter++;
    }

    void removeCallback(uint64_t id) {
        unique_lock ul(mutex, try_to_lock);
        if (ul.owns_lock()) {
            unsafeRemove(id);
            lock_guard lg(removeVectorMutex);
            if (!toRemove.empty()) {
                for (auto a : toRemove) {
                    unsafeRemove(a);
                }
                toRemove.clear();
            }
        } else {
            lock_guard lg(removeVectorMutex);
            toRemove.insert(id);
        }
    }
};

#endif //PAMITEENSY_CALLBACKMANAGER_H
