//
// Created by fogoz on 13/05/2025.
//

#ifndef STREAMSPLITTER_H
#define STREAMSPLITTER_H
#include <memory>
#include <Print.h>
#include <vector>

/**
 * @brief Stream multiplexer for output splitting
 * 
 * This class implements a stream splitter that:
 * - Duplicates output to multiple streams
 * - Supports raw and smart pointers
 * - Provides buffered writing
 * - Manages stream lifecycle
 * 
 * Useful for:
 * - Multi-destination logging
 * - Output mirroring
 * - Stream monitoring
 * - Debug output
 */
class StreamSplitter : public Print {
    std::vector<Print*> vector;                    ///< Raw pointer streams
    std::vector<std::shared_ptr<Print>> shared_vector;  ///< Smart pointer streams

public:
    /**
     * @brief Constructs a new stream splitter
     * 
     * @param vector List of output streams
     */
    explicit StreamSplitter(const std::vector<Print*> &vector)
        : vector(vector) {
    }

    /**
     * @brief Adds a raw pointer stream
     * @param print Stream to add
     */
    virtual void add(Print* print);

    /**
     * @brief Adds a smart pointer stream
     * @param print Stream to add
     */
    virtual void add(std::shared_ptr<Print> print) {
        shared_vector.push_back(print);
    }

    /**
     * @brief Virtual destructor
     */
    virtual ~StreamSplitter() = default;

    /**
     * @brief Writes a single byte
     * 
     * Writes the byte to all registered streams.
     * 
     * @param b Byte to write
     * @return size_t Number of bytes written
     */
    size_t write(uint8_t b) override;

    /**
     * @brief Writes a buffer
     * 
     * Writes the buffer to all registered streams.
     * 
     * @param buffer Data buffer
     * @param size Buffer size
     * @return size_t Number of bytes written
     */
    size_t write(const uint8_t *buffer, size_t size) override;
};

/// Global stream splitter instance
extern StreamSplitter streamSplitter;

#endif //STREAMSPLITTER_H
