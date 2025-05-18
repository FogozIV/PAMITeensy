//
// Created by fogoz on 13/05/2025.
//

#ifndef STREAMSPLITTER_H
#define STREAMSPLITTER_H
#include <Print.h>
#include <vector>


class StreamSplitter : public Print {
    std::vector<Print*> vector;
public:
    explicit StreamSplitter(const std::vector<Print*> &vector)
        : vector(vector) {
    }

    virtual ~StreamSplitter() = default;

    size_t write(uint8_t b) override;

    size_t write(const uint8_t *buffer, size_t size) override;
};

extern StreamSplitter streamSplitter;


#endif //STREAMSPLITTER_H
