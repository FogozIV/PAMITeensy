//
// Created by fogoz on 13/05/2025.
//

#include "../../include/utils/StreamSplitter.h"
#include "Arduino.h"

void StreamSplitter::add(Print *print) {
    vector.push_back(print);
}

size_t StreamSplitter::write(uint8_t b) {
    for (auto stream : vector) {
        stream->write(b);
    }
    for (auto stream : shared_vector) {
        stream->write(b);
    }
    return 1;
}

size_t StreamSplitter::write(const uint8_t *buffer, size_t size) {
    for (auto stream : vector) {
        stream->write(buffer, size);
    }
    for (auto stream : shared_vector) {
        stream->write(buffer, size);
    }
    return size;
}

StreamSplitter streamSplitter({
    &Serial,
#ifndef DEBUG_MODE_CUSTOM
    &Serial7
#endif
});
