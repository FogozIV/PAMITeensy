//
// Created by fogoz on 13/05/2025.
//

#ifndef STREAMSPLITTER_H
#define STREAMSPLITTER_H
#include <memory>
#include <Print.h>
#include <vector>


class StreamSplitter : public Print {
    std::vector<Print*> vector;
    std::vector<std::shared_ptr<Print>> shared_vector;
public:
    explicit StreamSplitter(const std::vector<Print*> &vector)
        : vector(vector) {
    }

    virtual void add(Print* print);

    virtual void add(std::shared_ptr<Print> print) {
        shared_vector.push_back(print);
    }

    virtual ~StreamSplitter() = default;

    size_t write(uint8_t b) override;

    size_t write(const uint8_t *buffer, size_t size) override;
};

extern StreamSplitter streamSplitter;


#endif //STREAMSPLITTER_H
