//
// Created by fogoz on 03/06/2025.
//

#ifndef PAMITEENSY_MUTEX_H
#define PAMITEENSY_MUTEX_H
#ifdef TEENSY41
#include "TeensyThreads.h"
#endif
#ifdef ESP32
#include "mutex"
#endif
#include "memory"

class Mutex{
#ifdef TEENSY41
    Threads::Mutex mutex;
#endif
#ifdef ESP32
    std::mutex mutex;
#endif
public:
    void lock(){
        mutex.lock();
    }

    void unlock(){
        mutex.unlock();
    }

    bool try_lock(){
        return mutex.try_lock();
    }
};
struct defer_lock_t { explicit defer_lock_t() = default; };
struct try_to_lock_t { explicit try_to_lock_t() = default; };
constexpr defer_lock_t defer_lock{};
constexpr try_to_lock_t try_to_lock{};
template<class MutexClass>
class lock_guard{
protected:
    MutexClass* mutex;

public:
    lock_guard(MutexClass& mutex) : mutex(&mutex){
        this->mutex->lock();
    }
    lock_guard(std::shared_ptr<MutexClass> mutex) : mutex(mutex.get()){
        this->mutex->lock();
    }
    virtual ~lock_guard() {
        mutex->unlock();
    }

};

template <typename Mutex>
class unique_lock {
public:
    unique_lock() : mtx(nullptr), owns(false) {}

    explicit unique_lock(Mutex& m) : mtx(&m), owns(true) {
        if (mtx != nullptr) {
            mtx->lock();
        }
    }

    unique_lock(Mutex& m, defer_lock_t) : mtx(&m), owns(false) {}

    unique_lock(Mutex& m, try_to_lock_t) : mtx(&m), owns(false) {
        if (mtx!=nullptr && mtx->try_lock()) {
            owns = true;
        }
    }

    ~unique_lock() {
        if (owns && mtx)
            mtx->unlock();
    }

    void lock() {
        if (!mtx || owns) return;
        mtx->lock();
        owns = true;
    }

    void unlock() {
        if (!mtx || !owns) return;
        mtx->unlock();
        owns = false;
    }

    bool owns_lock() const {
        return owns;
    }

    // Move constructor
    unique_lock(unique_lock&& other) noexcept
            : mtx(other.mtx), owns(other.owns) {
        other.mtx = nullptr;
        other.owns = false;
    }

    // Move assignment
    unique_lock& operator=(unique_lock&& other) noexcept {
        if (this != &other) {
            if (owns && mtx)
                mtx->unlock();

            mtx = other.mtx;
            owns = other.owns;
            other.mtx = nullptr;
            other.owns = false;
        }
        return *this;
    }

    Mutex* release() {
        Mutex* old = mtx;
        mtx = nullptr;
        owns = false;
        return old;
    }

    // Deleted copy operations
    unique_lock(const unique_lock&) = delete;
    unique_lock& operator=(const unique_lock&) = delete;

private:
    Mutex* mtx;
    bool owns;
};

#endif //PAMITEENSY_MUTEX_H
