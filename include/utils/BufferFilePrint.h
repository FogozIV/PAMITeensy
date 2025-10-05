//
// Created by fogoz on 26/02/2025.
//

#ifndef TEENSYCODE2_0_BUFFERFILEPRINT_H
#define TEENSYCODE2_0_BUFFERFILEPRINT_H

#include<memory>
#include <utility>
#include <FS.h>
#include <cassert>

#include "Print.h"
#include "Mutex.h"
#include "StreamSplitter.h"
#include "machine/endian.h"

class BufferFilePrint: public Print{
    Print& f;
    uint32_t current_index = 0;
    uint32_t size;
    mutable Mutex writing_mutex;
    mutable Mutex flush_mutex;
    std::unique_ptr<volatile uint8_t[]> data;
    std::unique_ptr<volatile uint8_t[]> copy_result;
    std::shared_ptr<Mutex> sd_mutex = nullptr;
public:
    BufferFilePrint(Print& f, uint32_t size=8192) : f(f){
        data = std::make_unique<volatile uint8_t[]>(size);
        copy_result = std::make_unique<volatile uint8_t[]>(size);
        this->size = size;
    }
    BufferFilePrint(File& f, std::shared_ptr<Mutex> sdMutex, uint32_t size=8192) : f(f), sd_mutex(std::move(sdMutex)){
        data = std::make_unique<volatile uint8_t[]>(size);
        copy_result = std::make_unique<volatile uint8_t[]>(size);
        assert(data != nullptr && copy_result != nullptr);
        this->size = size;
    }

    size_t write(uint8_t b) override {
        writing_mutex.lock();
        if(this->current_index == this->size){
            writing_mutex.unlock();
            return 0;
        }
        data[current_index++] = b;
        writing_mutex.unlock();
        return 1;
    }

    size_t write(const uint8_t *buffer, size_t size) override {
        writing_mutex.lock();
        size = min(this->size - current_index, size);
        memcpy((void*)&(data[current_index]), buffer, size);
        current_index+= size;
        writing_mutex.unlock();
        return size;
    }

    int availableForWrite(void) override {
        writing_mutex.lock();
        int available = size - current_index;
        writing_mutex.unlock();
        return available;
    }

    void flush() override {
        // Step 1: Early exit if nothing to do
        writing_mutex.lock();
        if (current_index == 0) {
            writing_mutex.unlock();
            return;
        }

        // Step 2: Swap buffers and get size
        std::swap(copy_result, data);
        uint32_t data_size = current_index;
        current_index = 0;
        writing_mutex.unlock();

        // Step 3: Lock flush and sd_mutex — in correct order
        flush_mutex.lock();
        if (sd_mutex) sd_mutex->lock();

        f.write(const_cast<uint8_t*>(copy_result.get()), data_size);
        f.flush();

        if (sd_mutex) sd_mutex->unlock();
        flush_mutex.unlock();
    }

    size_t write_raw(double d){
        uint64_t data;
        memcpy(&data, &d, sizeof(double));
        return write_raw(data);
    }

    size_t write_raw(uint64_t d){
        uint64_t data = __bswap64(d);
        return write((uint8_t*)(&data), sizeof(uint64_t));
    }

    size_t write_raw(uint32_t d){
        uint32_t data = __bswap32(d);
        return write((uint8_t*)(&data), sizeof(uint32_t));
    }

    size_t write_raw(int32_t d){
        int32_t data = __bswap32(d);
        return write((uint8_t*)(&data), sizeof(int32_t));
    }

    size_t write_raw(uint16_t d){
        uint16_t data = __bswap16(d);
        return write((uint8_t *)&data, sizeof(uint16_t));
    }

    size_t write_raw(uint8_t d){
        uint8_t data = d;
        return write((uint8_t*)&data, sizeof(uint8_t));
    }

    bool isFlushed() const {
        lock_guard lg(writing_mutex);
        return current_index == 0;
    }

    bool isOk() {
        return copy_result != nullptr && data != nullptr;
    }
};
class MultipleBufferPrint{
protected:
    std::vector<std::shared_ptr<BufferFilePrint>> printers{};
    std::vector<std::shared_ptr<BufferFilePrint>> locked_buffer{};
    Mutex mutex{};
    Mutex lockedBufferMutex{};
    void unsafeRemove(std::shared_ptr<BufferFilePrint> printer) {
        auto it = std::find(printers.begin(), printers.end(), printer);
        if (it != printers.end()) {
            printers.erase(it);
        }
    }

public:
    void add(std::shared_ptr<BufferFilePrint> printer){
        if (mutex.try_lock()) {
            printers.push_back(printer);
            mutex.unlock();
        }else {
            lock_guard lg(lockedBufferMutex);
            locked_buffer.push_back(printer);
        }
    }

    void remove(std::shared_ptr<BufferFilePrint> printer){
        lock_guard l_g(mutex);
        if(!printer){
            return;
        }
        unsafeRemove(printer);
    }

    void flushAll(){
        lock_guard l_g(mutex);
        for(auto p : printers){
            p->flush();
        }
        {
            lock_guard lg(lockedBufferMutex);
            if (locked_buffer.size() > 0) {
                printers.insert(printers.end(), locked_buffer.begin(), locked_buffer.end());
            }
            locked_buffer.clear();
        }
    }

};

#ifndef bufferPrinter
extern std::shared_ptr<BufferFilePrint> bufferPrinter;
extern MultipleBufferPrint bufferPrinters;
#endif
#endif //TEENSYCODE2_0_BUFFERFILEPRINT_H
