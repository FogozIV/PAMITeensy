//
// Created by fogoz on 26/02/2025.
//

#ifndef TEENSYCODE2_0_BUFFERFILEPRINT_H
#define TEENSYCODE2_0_BUFFERFILEPRINT_H

#include<memory>
#include "Print.h"

class BufferFilePrint: public Print{
    Print& f;
    volatile uint8_t* data;
    uint32_t current_index = 0;
    uint32_t size;
    mutex_t writing_mutex;
    mutex_t flush_mutex;
    volatile uint8_t* copy_result;
public:
    BufferFilePrint(Print& f, uint32_t size=8192) : f(f){
        this->f = f;
        data = (uint8_t*)malloc(size);
        copy_result = (uint8_t*) malloc(size);
        this->size = size;
        chMtxObjectInit(&writing_mutex);
        chMtxObjectInit(&flush_mutex);
    }
    size_t write(uint8_t b) override {
        chMtxLock(&writing_mutex);
        if(this->current_index == this->size){
        chMtxUnlock(&writing_mutex);
            return 0;
        }
        data[current_index++] = b;
        chMtxUnlock(&writing_mutex);
        return 1;
    }

    size_t write(const uint8_t *buffer, size_t size) override {
        chMtxLock(&writing_mutex);
        size = min(this->size - current_index, size);
        memcpy((void*)&(data[current_index]), buffer, size);
        current_index+= size;
        chMtxUnlock(&writing_mutex);
        return size;
    }

    int availableForWrite(void) override {
        chMtxLock(&writing_mutex);
        int available = size - current_index;
        chMtxUnlock(&writing_mutex);
        return available;
    }

    void flush() override {
        chMtxUnlock(&flush_mutex);
        chMtxLock(&writing_mutex);
        if(current_index == 0){
            chMtxUnlock(&writing_mutex);
            chMtxUnlock(&flush_mutex);
            return;
        }
        auto a = copy_result;
        copy_result = data;
        data = a;
        //memcpy((void *) copy_result, (const void *) data, current_index);
        int data_size = current_index;
        current_index = 0;
        chMtxUnlock(&writing_mutex);
        f.write((uint8_t*)copy_result, data_size);
        f.flush();
        chMtxUnlock(&flush_mutex);
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
