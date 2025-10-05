//
// Created by 540170 on 04-10-25.
//

#ifndef PAMITEENSY_ESP32QUADENCODERIMPL_H
#define PAMITEENSY_ESP32QUADENCODERIMPL_H
#ifdef ESP32
#include <Arduino.h>
#include "BaseEncoder.h"

#include <ESP32Encoder.h>

/**
 * ESP32 implementation of BaseEncoder using the madhephaestus ESP32Encoder library.
 */
class ESP32QuadEncoderImpl : public BaseEncoder {
    ESP32Encoder encoder;
    int64_t previous_count = 0;
public:
    // On ESP32 there is no hardware channel argument; keep signature similar for drop-in replacement.
    ESP32QuadEncoderImpl(uint8_t pinA, uint8_t pinB, uint8_t channel = 0);
    ~ESP32QuadEncoderImpl() override = default;

    int32_t getEncoderCount() override;
    int32_t getDeltaCount() override;
    int32_t setEncoderCount(int32_t count) override;
};
#endif
#endif //PAMITEENSY_ESP32QUADENCODERIMPL_H