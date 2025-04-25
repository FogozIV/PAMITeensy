//
// Created by fogoz on 24/04/2025.
//

#ifndef PAMITEENSY_BASEENCODER_H
#define PAMITEENSY_BASEENCODER_H
#include "Arduino.h"

class BaseEncoder {
public:
    /**
     * @return the current encoder count
     */
    virtual int32_t getEncoderCount() const = 0;
    /**
     * This method can be use to get the delta count from the last call of this function
     * Its behavior should be that if the setEncoderCount is used then the delta should continue to work
     * @return the number of count since last call
     */
    virtual int32_t getDeltaCount() = 0;

    /**
     * This method allows to set the count
     * @param count the count we want to set
     * @return the previous count
     */
    virtual int32_t setEncoderCount(int32_t count) = 0;
};

#endif //PAMITEENSY_BASEENCODER_H
