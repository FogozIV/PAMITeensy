//
// Created by fogoz on 06/05/2025.
//

#include "../../include/network/CustomAsyncClient.h"

#include <usb_serial.h>

void CustomAsyncClient::onData(void *_, AsyncClient *client, void *data, size_t len) {
    Serial.printf("Received %d bytes from client : %d \r\n", len, client->getConnectionId());
    std::vector<uint8_t> data_vec(data, data + len);
    for (auto it = data_listeners.begin(); it != data_listeners.end();) {
        if (it->operator()(data_vec)) {
            it = data_listeners.erase(it);
        }else {
            ++it;
        }
    }
}

void CustomAsyncClient::onConnect(void *_, AsyncClient *client) {
    for (auto it = connect_listeners.begin(); it != connect_listeners.end();) {
        if (it->operator()()) {
            it = connect_listeners.erase(it);
        }else {
            ++it;
        }
    }
}

void CustomAsyncClient::onDisconnect(void *_, AsyncClient *client) {
    for (auto it = disconnect_listeners.begin(); it != disconnect_listeners.end(); ) {
        if (it->operator()()) {
            it = disconnect_listeners.erase(it);
        }else {
            ++it;
        }
    }
}

void CustomAsyncClient::onError(void *_, AsyncClient *client, err_t error) {
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

CustomAsyncClient::CustomAsyncClient(AsyncClient *client, std::shared_ptr<PacketHandler>): client(client), packet_handler(std::move(packet_handler)) {
    client->onData(this->onData);
    client->onConnect(this->onConnect);
    client->onDisconnect(this->onDisconnect);
    client->onError(this->onError);

    registerDataListener([*this](std::vector<uint8_t> data) -> bool {
        packet_handler->receiveData(data);
        auto [result, packet] = packet_handler->checkPacket(this->client);
        this->onCheck(result, packet);
        return false;
    });

    packetDispatcher->registerCallBack<PingPacket>([*this](std::shared_ptr<PingPacket> packet) {
        std::shared_ptr<PongPacket> pong = std::make_shared<PongPacket>(packet->getUniqueID());
        sendPacket(pong);
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

