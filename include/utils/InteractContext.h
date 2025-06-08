//
// Created by fogoz on 21/05/2025.
//

#ifndef INTERACTCONTEXT_H
#define INTERACTCONTEXT_H
#include <memory>
#include <Stream.h>

#include "StateMachine.h"

class BaseRobot;

/**
 * @brief Interactive context manager
 * 
 * This module provides interactive context management for:
 * - Command line interface
 * - Robot control
 * - State machine interaction
 * - Stream communication
 * 
 * Features:
 * - Command processing
 * - State tracking
 * - Idle handling
 * - Error reporting
 * 
 * @param robot Robot instance
 * @param stream Communication stream
 * @param idle Optional idle callback
 * @return const char* Error message or nullptr on success
 */
const char* enterInteractContext(std::shared_ptr<BaseRobot> robot, Stream &stream, std::function<void()> idle=[]{});

#endif //INTERACTCONTEXT_H
