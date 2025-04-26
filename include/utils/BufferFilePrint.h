//
// Created by fogoz on 26/02/2025.
//

#ifndef TEENSYCODE2_0_BUFFERFILEPRINT_H
#define TEENSYCODE2_0_BUFFERFILEPRINT_H

#include <TeensyThreads.h>

#include <utility>
#include "Print.h"
#include "FS.h"

class BufferFilePrint: public Print{
    File f;
    volatile uint8_t* data;
    int current_index = 0;
    int size;
    Threads::Mutex writing_mutex;
    Threads::Mutex flush_mutex;
    volatile uint8_t* copy_result;
public:
    BufferFilePrint(File f, int size=8192){
        this->f = std::move(f);
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
        std::swap(copy_result, data);
        //memcpy((void *) copy_result, (const void *) data, current_index);
        int data_size = current_index;
        current_index = 0;
        writing_mutex.unlock();
        f.write((uint8_t*)copy_result, data_size);
        f.flush();
        flush_mutex.unlock();
    }

    virtual ~BufferFilePrint() {
        free((void *) data);
        free((void *) copy_result);
    }
};

#endif //TEENSYCODE2_0_BUFFERFILEPRINT_H
