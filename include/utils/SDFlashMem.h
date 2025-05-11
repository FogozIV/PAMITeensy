//
// Created by fogoz on 10/05/2025.
//

#ifndef SDFLASHMEM_H
#define SDFLASHMEM_H
#include <SD.h>

#include "Arduino.h"

inline bool initSDCard() {
    if (!SD.begin(BUILTIN_SDCARD)) {
        return false;
    }
    return true;
}
#endif //SDFLASHMEM_H
