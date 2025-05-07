//
// Created by fogoz on 06/05/2025.
//

#include "../../include/network/CustomAsyncClient.h"

#include <Arduino.h>

#include "utils/RegisterCommands.h"

void CustomAsyncClient::onData(void *data, size_t len) {
    std::vector<uint8_t> data_vec(static_cast<uint8_t *>(data), static_cast<uint8_t *>(data) + len);
    for (auto it = data_listeners.begin(); it != data_listeners.end();) {
        if (it->operator()(data_vec)) {
            it = data_listeners.erase(it);
        }else {
            ++it;
        }
    }
}

void CustomAsyncClient::onConnect() {
    for (auto it = connect_listeners.begin(); it != connect_listeners.end();) {
        if (it->operator()()) {
            it = connect_listeners.erase(it);
        }else {
            ++it;
        }
    }
}

void CustomAsyncClient::onDisconnect() {
    for (auto it = disconnect_listeners.begin(); it != disconnect_listeners.end(); ) {
        if (it->operator()()) {
            it = disconnect_listeners.erase(it);
        }else {
            ++it;
        }
    }
}

void CustomAsyncClient::onError(err_t error) {
    for (auto it = error_listeners.begin(); it != error_listeners.end();) {
        if (it->operator()(error)) {
            it = error_listeners.erase(it);
        }else {
            ++it;
        }
    }
}

void CustomAsyncClient::onCheck(CheckStatus status, std::shared_ptr<IPacket> packet) {
    for (auto it = check_listeners.begin(); it != check_listeners.end();) {
        if (it->operator()(status, packet)) {
            it = check_listeners.erase(it);
        }else {
            ++it;
        }
    }
}

CustomAsyncClient::CustomAsyncClient(AsyncClient *client): client(client) {
    client->onData(_onData, this);
    client->onConnect(_onConnect, this);
    client->onDisconnect(_onDisconnect, this);
    client->onError(_onError, this);

    registerDataListener([this](std::vector<uint8_t> data) -> bool {
        packet_handler->receiveData(data);
        CheckStatus result = CheckStatus::BAD_CRC;
        std::shared_ptr<IPacket> packet;
        while (result != CheckStatus::WAITING_DATA && result != CheckStatus::WAITING_LENGTH) {
            auto a = packet_handler->checkPacket(this->client);
            result = std::get<0>(a);
            packet = std::get<1>(a);
            if (packet != nullptr) {
                packetDispatcher->dispatch(packet);
            }
            this->onCheck(result, packet);

        }
        return false;
    });

    packetDispatcher->registerCallBack<PingPacket>([this](std::shared_ptr<PingPacket> packet) {
        std::shared_ptr<PongPacket> pong = std::make_shared<PongPacket>(packet->getUniqueID());
        sendPacket(pong);
        return false;
    });

    packetDispatcher->registerCallBack<StartFlashPacket>([this](std::shared_ptr<StartFlashPacket> packet) {
        Serial.println("Received start flash packet");
        if (updater.isFlashing()) {
            sendPacket(std::make_shared<AlreadyFlashingPacket>());
            return false;
        }
        if (updater.startFlashMode()) {
            Serial.println("Flashing started");
            sendPacket(std::make_shared<StartFlashPacket>());
            packetDispatcher->registerCallBack<DataPacket>([this](std::shared_ptr<DataPacket> packet) {
                Serial.println("Received data packet");
                updater.addData(reinterpret_cast<const char *>(packet->getDataRef().data()), packet->getDataRef().size());
                if (!updater.parse()) {
                    sendPacket(std::make_shared<IssueFlashingPacket>());
                    Serial.println("Issue flashing");
                    return true;
                }
                if (updater.isDone()) {
                    sendPacket(std::make_shared<FlashingSoftwarePacket>());
                    Serial.println("Flashing software");
                    updater.callDone();
                    return true;
                }
                Serial.println("Flashing in progress sending ack");
                sendPacket(std::make_shared<ReceivedDataPacket>(packet->getDataRef().size()));
                return false;
            });

        }else {
            sendPacket(std::make_shared<IssueStartingFlashingPacket>());
        }
        return false;
    });

}

void CustomAsyncClient::registerDataListener(const std::function<bool(std::vector<uint8_t>)> &data_listener) {
    data_listeners.push_back(data_listener);
}

void CustomAsyncClient::registerConnectListener(const std::function<bool()> &connect_listener) {
    connect_listeners.push_back(connect_listener);
}

void CustomAsyncClient::registerDisconnectListener(const std::function<bool()> &disconnect_listener) {
    disconnect_listeners.push_back(disconnect_listener);
}

void CustomAsyncClient::registerErrorListener(const std::function<bool(err_t error)> &error_listener) {
    error_listeners.push_back(error_listener);
}

std::shared_ptr<PacketDispatcher> CustomAsyncClient::getPacketDispatcher() {
    return packetDispatcher;
}

void CustomAsyncClient::sendPacket(std::shared_ptr<IPacket> packet) {
    auto a = packet_handler->createPacket(packet);
    client->write(reinterpret_cast<const char *>(a.data()), a.size(), TCP_WRITE_FLAG_COPY);
    client->send();
}

void CustomAsyncClient::sendPing(uint32_t id) {
    PingPacket ping(id);
    auto a = packet_handler->createPacket(ping);
    Serial.printf("Sending ping with id: %lu\r\n", id);
    client->write(reinterpret_cast<const char *>(a.data()), a.size(), TCP_WRITE_FLAG_COPY);
    client->send();
    packetDispatcher->registerCallBack<PongPacket>([id](std::shared_ptr<PongPacket> packet) {
        if (packet->getUniqueID() == id) {
            Serial.printf("Received pong packet with id %d\r\n", id);
        }else {
            Serial.printf("Received pong packet with id %d but expected %d\r\n", packet->getUniqueID(), id);
        }
        return true;
    });
}

