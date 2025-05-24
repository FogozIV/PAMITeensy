//
// Created by fogoz on 24/04/2025.
//

#ifndef PAMITEENSY_QUADENCODERIMPL_H
#define PAMITEENSY_QUADENCODERIMPL_H
#include <QuadEncoder.h>
#include "BaseEncoder.h"
#include <memory>

/**
 * @brief Quadrature encoder implementation for Teensy
 * 
 * This class implements the BaseEncoder interface using Teensy's
 * hardware quadrature decoder. It supports:
 * - 4x quadrature decoding
 * - Hardware-based counting
 * - Multiple encoder channels
 * - Automatic overflow handling
 * 
 * The implementation uses Teensy's QuadEncoder library to interface
 * with the hardware quadrature decoder modules.
 */
class QuadEncoderImpl: public BaseEncoder {
    std::unique_ptr<QuadEncoder> quadEncoder;  ///< Hardware encoder interface
    int32_t previous_count;                    ///< Last count for delta calculation

public:
    /**
     * @brief Constructs a new quadrature encoder
     * 
     * Initializes the hardware quadrature decoder for the specified pins
     * and channel. The pins must be compatible with the selected channel.
     * 
     * @param pinA Phase A input pin
     * @param pinB Phase B input pin
     * @param channel Hardware decoder channel (1-4)
     */
    QuadEncoderImpl(uint8_t pinA, uint8_t pinB, uint8_t channel);

    virtual ~QuadEncoderImpl() = default;

    /**
     * @brief Gets the current encoder count
     * 
     * Reads the current count from the hardware quadrature decoder.
     * The count increases when phase A leads phase B and decreases
     * when phase B leads phase A.
     * 
     * @return int32_t Current encoder count
     */
    int32_t getEncoderCount() const override;

    /**
     * @brief Gets count change since last call
     * 
     * Calculates the difference between the current count and
     * the last time this method was called. Handles counter
     * overflow/underflow automatically.
     * 
     * @return int32_t Count change since last call
     */
    int32_t getDeltaCount() override;

    /**
     * @brief Sets the encoder count
     * 
     * Updates the hardware counter value. This is useful for:
     * - Zeroing the encoder
     * - Setting a specific starting position
     * - Synchronizing multiple encoders
     * 
     * @param count New count value
     * @return int32_t Previous count value
     */
    int32_t setEncoderCount(int32_t count) override;
};

#endif //PAMITEENSY_QUADENCODERIMPL_H
