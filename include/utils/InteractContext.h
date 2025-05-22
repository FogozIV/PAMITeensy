//
// Created by fogoz on 21/05/2025.
//

#ifndef INTERACTCONTEXT_H
#define INTERACTCONTEXT_H
#include <memory>
#include <Stream.h>
#include <TeensyThreads.h>

#include "StateMachine.h"

class BaseRobot;

const char* enterInteractContext(std::shared_ptr<BaseRobot> robot, Stream &stream, std::function<void()> idle=[]{});

#endif //INTERACTCONTEXT_H
