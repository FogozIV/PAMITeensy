//
// Created by fogoz on 07/06/2025.
//

#ifndef THREADSAFESERIAL_H
#define THREADSAFESERIAL_H
#include "Arduino.h"
#include <cassert>

#include "Mutex.h"


class ThreadSafeSerial : public Stream {
public:
    virtual ~ThreadSafeSerial() = default;

    HardwareSerial* serial = nullptr;  // Optional (for begin/end)
    Stream* stream = nullptr;          // Used for most functions
    Threads::Mutex mutex;

    // Constructor for HardwareSerial
    ThreadSafeSerial(HardwareSerial& s);

    // Constructor for any Stream (e.g., Serial, File, etc.)
    ThreadSafeSerial(Stream& s);

    ThreadSafeSerial() = delete;

    // ---- Serial-specific methods (only if using HardwareSerial) ----
    void begin(uint32_t baud, uint16_t format=0);

    void end() const;

    void flush() override;

    // ---- Common Stream methods ----
    int available() override;

    int read() override;

    // ---- Print methods ----
    template <typename... Args>
    void print(Args&&... args);

    template <typename... Args>
    void println(Args&&... args);

    // ---- printf method (HardwareSerial only) ----
    template <typename... Args>
    void printf(const char* fmt, Args&&... args);

    // ---- write (available in both Stream and HardwareSerial) ----
    size_t write(uint8_t val) override;

    String readString() const;

    int peek() override;

    size_t write(const uint8_t *buffer, size_t size) override;

    int availableForWrite() override;
};

template<typename ... Args>
void ThreadSafeSerial::print(Args &&...args) {
    mutex.lock();
    stream->print(std::forward<Args>(args)...);
    mutex.unlock();
}

template<typename ... Args>
void ThreadSafeSerial::println(Args &&...args) {
    mutex.lock();
    stream->println(std::forward<Args>(args)...);
    mutex.unlock();
}

template<typename ... Args>
void ThreadSafeSerial::printf(const char *fmt, Args &&...args) {
    mutex.lock();
    stream->printf(fmt, std::forward<Args>(args)...);
    mutex.unlock();
}

extern ThreadSafeSerial SerialTS;
extern ThreadSafeSerial Serial1TS;
extern ThreadSafeSerial Serial2TS;
extern ThreadSafeSerial Serial3TS;
extern ThreadSafeSerial Serial4TS;
extern ThreadSafeSerial Serial5TS;
extern ThreadSafeSerial Serial6TS;
extern ThreadSafeSerial Serial7TS;
extern ThreadSafeSerial Serial8TS;

#endif //THREADSAFESERIAL_H
