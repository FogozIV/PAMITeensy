//
// Created by fogoz on 07/06/2025.
//
#include <utils/ThreadSafeSerial.h>

ThreadSafeSerial::ThreadSafeSerial(HardwareSerial &s): serial(&s), stream(&s) {
    Serial.printf("New Hardware serial added %p\r\n", (void*)stream);
}

ThreadSafeSerial::ThreadSafeSerial(Stream &s): stream(&s) {
    Serial.printf("New Hardware serial added %p\r\n", (void*)stream);
}

void ThreadSafeSerial::begin(uint32_t baud, uint16_t format) {
    if (serial != nullptr) {
        serial->begin(baud, format);
    }
}

void ThreadSafeSerial::end() const {
    if (serial != nullptr) {
        serial->end();
    }
}

void ThreadSafeSerial::flush() {
    if (serial != nullptr) {
        serial->flush();
    }
}

int ThreadSafeSerial::available() {
    return stream->available();
}

int ThreadSafeSerial::read() {
    return stream->read();
}

size_t ThreadSafeSerial::write(uint8_t val){
    mutex.lock();
    size_t res = stream->write(val);
    mutex.unlock();
    return res;
}

String ThreadSafeSerial::readString() const {
    return stream->readString();
}

int ThreadSafeSerial::peek() {
    return stream->peek();
}

size_t ThreadSafeSerial::write(const uint8_t *buffer, size_t size) {
    assert(stream != nullptr);
    mutex.lock();
    size_t res = stream->write(buffer, size);
    mutex.unlock();
    return res;
}

int ThreadSafeSerial::availableForWrite() {
    return stream->availableForWrite();
}

ThreadSafeSerial SerialTS(Serial);
ThreadSafeSerial Serial1TS(Serial1);
ThreadSafeSerial Serial2TS(Serial2);
ThreadSafeSerial Serial3TS(Serial3);
ThreadSafeSerial Serial4TS(Serial4);
ThreadSafeSerial Serial5TS(Serial5);
ThreadSafeSerial Serial6TS(Serial6);
ThreadSafeSerial Serial7TS(Serial7);
ThreadSafeSerial Serial8TS(Serial8);