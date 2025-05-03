//
// Created by fogoz on 03/05/2025.
//

#include "utils/TCPStateMachine.h"

#include <Arduino.h>

#include "packets/DataPacket.h"
#include "utils/CRC.h"
#define CASE(x, content)\
    case x:\
        {content}\
        break;


TCPStateMachine::TCPStateMachine(){
}


void TCPStateMachine::handleData(AsyncClient *client, void *data_p, size_t len) {
    packetHandler.receiveData(static_cast<const uint8_t *>(data_p), len);
    auto [result, packet] = packetHandler.checkPacket();
    switch (result) {
        case WAITING_LENGTH:
            break;
        case WAITING_DATA:
            break;
        case BAD_CRC:
        case BAD_PACKET_ID:
        case PACKET_TOO_SMALL:
        case NULL_PTR_RETURN: {
            for (auto& a : issue) {
                a(client, result);
            }
        }
            break;
    }
}

void TCPStateMachine::registerListeners() {
    DataPacket::callbacks.push_back([&](std::shared_ptr<DataPacket> packet) {

    });
}
