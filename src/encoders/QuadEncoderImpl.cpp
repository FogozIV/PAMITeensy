//
// Created by fogoz on 24/04/2025.
//

#include "encoders/QuadEncoderImpl.h"

QuadEncoderImpl::QuadEncoderImpl(uint8_t pinA, uint8_t pinB, uint8_t channel){
    pinMode(pinA, INPUT);
    pinMode(pinB, INPUT);
    quadEncoder = std::make_unique<QuadEncoder>(channel, pinA, pinB);
    quadEncoder->write(0);
    quadEncoder->setInitConfig();
    quadEncoder->init();
    previous_count = 0;
}

int32_t QuadEncoderImpl::getEncoderCount() const {
    return quadEncoder->read();
}

int32_t QuadEncoderImpl::getDeltaCount() {
    int32_t read = quadEncoder->read();
    int32_t ret = read - previous_count;
    previous_count = read;
    return ret;
}

int32_t QuadEncoderImpl::setEncoderCount(int32_t count) {
    int32_t read = quadEncoder->read();
    quadEncoder->write(*((uint32_t*)&count));
    //update the previous to have no jump
    previous_count-=read;
    previous_count+=count;
    return read;
}