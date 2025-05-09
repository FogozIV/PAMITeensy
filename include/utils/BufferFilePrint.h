//
// Created by fogoz on 26/02/2025.
//

#ifndef TEENSYCODE2_0_BUFFERFILEPRINT_H
#define TEENSYCODE2_0_BUFFERFILEPRINT_H

#include <TeensyThreads.h>
#include<memory>
#include "Print.h"

class BufferFilePrint: public Print{
    Print& f;
    volatile uint8_t* data;
    uint32_t current_index = 0;
    uint32_t size;
    Threads::Mutex writing_mutex;
    Threads::Mutex flush_mutex;
    volatile uint8_t* copy_result;
public:
    BufferFilePrint(Print& f, uint32_t size=8192) : f(f){
        this->f = f;
        data = (uint8_t*)malloc(size);
        copy_result = (uint8_t*) malloc(size);
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
        flush_mutex.lock();
        writing_mutex.lock();
        if(current_index == 0){
            writing_mutex.unlock();
            flush_mutex.unlock();
            return;
        }
        auto a = copy_result;
        copy_result = data;
        data = a;
        //memcpy((void *) copy_result, (const void *) data, current_index);
        int data_size = current_index;
        current_index = 0;
        writing_mutex.unlock();
        f.write((uint8_t*)copy_result, data_size);
        f.flush();
        flush_mutex.unlock();
    }

    bool isOk() {
        return copy_result != nullptr && data != nullptr;
    }

    virtual ~BufferFilePrint() {
        free((void *) data);
        free((void *) copy_result);
    }
};
extern std::shared_ptr<BufferFilePrint> bufferPrinter;

#endif //TEENSYCODE2_0_BUFFERFILEPRINT_H
