//
// Created by fogoz on 26/02/2025.
//

#ifndef TEENSYCODE2_0_BUFFERFILEPRINT_H
#define TEENSYCODE2_0_BUFFERFILEPRINT_H

#include <TeensyThreads.h>
#include<memory>
#include <utility>
#include <FS.h>
#include "Print.h"
#include "Mutex.h"

class BufferFilePrint: public Print{
    Print& f;
    volatile uint8_t* data;
    uint32_t current_index = 0;
    uint32_t size;
    Mutex writing_mutex;
    Mutex flush_mutex;
    volatile uint8_t* copy_result;
    std::shared_ptr<Mutex> sd_mutex = nullptr;
public:
    BufferFilePrint(Print& f, uint32_t size=8192) : f(f){
        data = (uint8_t*)malloc(size);
        copy_result = (uint8_t*) malloc(size);
        this->size = size;
    }
    BufferFilePrint(File& f, std::shared_ptr<Mutex> sdMutex, uint32_t size=8192) : f(f), sd_mutex(std::move(sdMutex)){
        data = (uint8_t*)malloc(size);
        copy_result = (uint8_t*)malloc(size);
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
        if(sd_mutex != nullptr){
            sd_mutex->lock();
        }
        f.write((uint8_t*)copy_result, data_size);
        f.flush();
        if(sd_mutex != nullptr){
            sd_mutex->unlock();
        }
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

class MultipleBufferPrint{
protected:
    std::vector<std::shared_ptr<BufferFilePrint>> printers{};
    Mutex mutex{};
public:
    void add(std::shared_ptr<BufferFilePrint> printer){
        lock_guard l_g(mutex);
        printers.push_back(printer);
    }

    void remove(std::shared_ptr<BufferFilePrint> printer){
        lock_guard l_g(mutex);
        if(!printer){
            return;
        }
        printer->flush();
        printers.erase(std::find(printers.begin(), printers.end(),printer));
    }

    void flushAll(){
        lock_guard l_g(mutex);
        for(auto p : printers){
            p->flush();
        }
    }

};

#ifndef bufferPrinter
extern std::shared_ptr<BufferFilePrint> bufferPrinter;
extern MultipleBufferPrint bufferPrinters;
#endif
#endif //TEENSYCODE2_0_BUFFERFILEPRINT_H
