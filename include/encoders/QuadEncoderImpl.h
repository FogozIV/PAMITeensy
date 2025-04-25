//
// Created by fogoz on 24/04/2025.
//

#ifndef PAMITEENSY_QUADENCODERIMPL_H
#define PAMITEENSY_QUADENCODERIMPL_H
#include <QuadEncoder.h>
#include "BaseEncoder.h"
#include <memory>
class QuadEncoderImpl: public BaseEncoder{
    std::unique_ptr<QuadEncoder> quadEncoder;
    int32_t previous_count;
public:
    QuadEncoderImpl(uint8_t pinA, uint8_t pinB, uint8_t channel);

    virtual ~QuadEncoderImpl();

    int32_t getEncoderCount() const override;

    int32_t getDeltaCount() override;

    int32_t setEncoderCount(int32_t count) override;
};
#endif //PAMITEENSY_QUADENCODERIMPL_H
