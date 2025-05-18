//
// Created by fogoz on 13/05/2025.
//

#include "../../include/utils/StreamSplitter.h"
#include "Arduino.h"
size_t StreamSplitter::write(uint8_t b) {
    for (auto stream : vector) {
        stream->write(b);
    }
    return 1;
}

size_t StreamSplitter::write(const uint8_t *buffer, size_t size) {
    for (auto stream : vector) {
        stream->write(buffer, size);
    }
    return size;
}

StreamSplitter streamSplitter({&Serial, &Serial7});
