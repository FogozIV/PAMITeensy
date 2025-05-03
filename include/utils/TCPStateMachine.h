//
// Created by fogoz on 03/05/2025.
//

#ifndef PAMITEENSY_TCPSTATEMACHINE_H
#define PAMITEENSY_TCPSTATEMACHINE_H

#include "Teensy41_AsyncTCP.hpp"
#include "NetworkUtils.h"
#include <functional>
#include "PacketHandler.h"




class TCPStateMachine {
    PacketHandler packetHandler;
public:
    TCPStateMachine();

    void handleData(AsyncClient * client, void * data, size_t len);
};


#endif //PAMITEENSY_TCPSTATEMACHINE_H
