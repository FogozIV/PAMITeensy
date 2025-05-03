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
    DataPacket::callbacks.push_back([](std::shared_ptr<DataPacket> packet) {
        Serial.println("Data packet received");
        Serial.println(packet->getPacketID());
        Serial.println(packet->getData().size());
    });
}


void TCPStateMachine::handleData(AsyncClient *client, void *data_p, size_t len) {
    packetHandler.receiveData(static_cast<const uint8_t *>(data_p), len);
    Serial.printf("Buffer size : %f ", packetHandler.getBuffer().size());
    auto [result, packet] = packetHandler.checkPacket();
    switch (result) {
        case WAITING_LENGTH:
            Serial.println("Waiting length");
            break;
        case WAITING_DATA:
            Serial.println("Waiting data");
            break;
        case BAD_CRC:
            Serial.println("Bad CRC");
            break;
        case EXECUTED_PACKET:
            Serial.println(packet->getPacketID());
            Serial.println("Packet Received and executed");
            break;
        case BAD_PACKET_ID:
            Serial.println("Bad packet ID deleting full row");
            break;
        case PACKET_TOO_SMALL:
            Serial.println("Packet too small deleting full row");
            break;
        case NULL_PTR_RETURN:
            Serial.println("Null ptr return deleting full row");
            break;
    }

}
