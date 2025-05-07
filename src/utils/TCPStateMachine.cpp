//
// Created by fogoz on 03/05/2025.
//

#include "utils/TCPStateMachine.h"

#include <Arduino.h>
#include <TeensyThreads.h>
#include <utils/RegisterCommands.h>
#include "utils/CRC.h"
#define CASE(x, content)\
    case x:\
        {content}\
        break;


TCPStateMachine::TCPStateMachine(PacketHandler& handler, std::shared_ptr<CustomAsyncClient> client):client(client),  packetHandler(handler){
}


void TCPStateMachine::handleData(AsyncClient *client, void *data_p, size_t len) {
    Serial.printf("Received %d bytes from client : %d \r\n", len, client->getConnectionId());
    packetHandler.receiveData(static_cast<const uint8_t *>(data_p), len);
    auto [result, packet] = packetHandler.checkPacket(client);
    Serial.printf("Received check result %d\r\n", result);
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
        case EXECUTED_PACKET:
            break;
        case CRC_ISSUE:
            break;
        default:
            break;
    }
}
void TCPStateMachine::registerListeners() {
    PingPacket::callbacks.emplace_back([&](std::shared_ptr<PingPacket> packet, AsyncClient* client) {
        if (client != this->client->getClient())
            return false;
        uint64_t id = packet->getUniqueID();
        PongPacket return_packet(id);
        Serial.printf("Received ping packet with id %lld\r\n", id);
        auto a =packetHandler.createPacket(return_packet);
        client->write(reinterpret_cast<const char *>(a.data()), a.size(), TCP_WRITE_FLAG_COPY);
        client->send();
        return false;
    });
    StartFlashPacket::callbacks.emplace_back([this](std::shared_ptr<StartFlashPacket> packet, AsyncClient* client) {
        if (client != this->client->getClient()) {
            return false;
        }
        Serial.println("Flashing packet");
        if (flashing_process) {
            sendPacket(client, std::make_shared<AlreadyFlashingPacket>());
            return false;
        }
        if (!updater.startFlashMode()) {
            sendPacket(client, std::make_shared<IssueStartingFlashingPacket>());
            return false;
        }
        sendPacket(client, std::make_shared<StartFlashPacket>());
        DataPacket::callbacks.emplace_back([this](std::shared_ptr<DataPacket> packet, AsyncClient* client) {
            Serial.println("Client verification");
            if (client != this->client->getClient()) {
                return false;
            }
            Serial.println("Flashing in progress");
            std::vector<uint8_t>& packet_raw = packet->getDataRef();
            Serial.println(packet_raw.size());
            updater.addData(reinterpret_cast<const char *>(packet_raw.data()), packet_raw.size());
            Serial.println("Changed buffer");
            sendPacket(client, std::make_shared<ReceivedDataPacket>(packet_raw.size()));
            if (!updater.parse()) {
                sendPacket(client, std::make_shared<IssueFlashingPacket>());
                updater.abort();
                return true;
            }
            if (updater.isDone()) {
                if (updater.isValid()) {
                    sendPacket(client, std::make_shared<FlashingSoftwarePacket>());
                    updater.callDone();
                    return true;
                }
            }
            return false;
        });
        return false;
    });
}

void TCPStateMachine::sendPacket(AsyncClient *client, std::shared_ptr<IPacket> packet) {
    auto a = packetHandler.createPacket(packet);
    Serial.printf("Sending packet %d %d\r\n", packet->getPacketID(), a.size());
    client->write(reinterpret_cast<const char *>(a.data()), a.size(), TCP_WRITE_FLAG_COPY);
    client->send();
}
