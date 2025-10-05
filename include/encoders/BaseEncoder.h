//
// Created by fogoz on 24/04/2025.
//

#ifndef PAMITEENSY_BASEENCODER_H
#define PAMITEENSY_BASEENCODER_H
#include "Arduino.h"

/**
 * @brief Abstract base class for encoder interfaces
 * 
 * This class defines the interface for reading encoder values in the system.
 * It provides methods for:
 * - Reading absolute encoder counts
 * - Getting incremental count changes
 * - Setting/resetting encoder values
 * 
 * Implementations of this interface should handle:
 * - Hardware-specific encoder reading
 * - Count overflow protection
 * - Thread-safe operation
 * - Proper count direction
 */
class BaseEncoder {
public:
    /**
     * @brief Gets the current absolute encoder count
     * 
     * This method returns the current absolute position of the encoder
     * in encoder ticks. The count direction depends on the encoder's
     * physical mounting and wiring.
     * 
     * @return int32_t Current encoder count
     */
    virtual int32_t getEncoderCount() = 0;

    /**
     * @brief Gets the encoder count change since last call
     * 
     * This method returns the number of ticks the encoder has moved
     * since the last time this method was called. It provides a way
     * to measure incremental motion without needing to track the
     * previous value externally.
     * 
     * The delta count continues to work correctly even if the absolute
     * count is reset using setEncoderCount().
     * 
     * @return int32_t Number of ticks moved since last call
     */
    virtual int32_t getDeltaCount() = 0;

    /**
     * @brief Sets the current encoder count
     * 
     * This method allows setting the absolute encoder count to a specific
     * value, which is useful for:
     * - Initializing the encoder
     * - Resetting the count to zero
     * - Synchronizing multiple encoders
     * 
     * Note that this does not affect the behavior of getDeltaCount().
     * 
     * @param count New encoder count value
     * @return int32_t Previous encoder count
     */
    virtual int32_t setEncoderCount(int32_t count) = 0;

    /**
     * @brief Virtual destructor
     */
    virtual ~BaseEncoder() = default;
};

#endif //PAMITEENSY_BASEENCODER_H
