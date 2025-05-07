//
// Created by fogoz on 06/05/2025.
//

#ifndef CUSTOMASYNCCLIENT_H
#define CUSTOMASYNCCLIENT_H
#include "Teensy41_AsyncTCP.hpp"
#include <functional>
#include "Arduino.h"

#include "PacketHandler.h"
#include "TCPTeensyUpdater.h"
#include "packets/PacketDefinition.h"
#include "utils/PacketDispatcher.h"

class CustomAsyncClient {
    AsyncClient* client = nullptr;
    std::shared_ptr<PacketHandler> packet_handler = std::make_shared<PacketHandler>();
    std::shared_ptr<PacketDispatcher> packetDispatcher = std::make_shared<PacketDispatcher>();
    TCPTeensyUpdater updater;

    std::vector<std::function<bool(std::vector<uint8_t>)>> data_listeners{};
    std::vector<std::function<bool()>> connect_listeners{};
    std::vector<std::function<bool()>> disconnect_listeners{};
    std::vector<std::function<bool(err_t error)>> error_listeners{};
    std::vector<std::function<bool(CheckStatus status, std::shared_ptr<IPacket> packet)>> check_listeners {};

    static void _onData(void* pCustom, AsyncClient* client, void *data, size_t len) {
        auto* custom_client = static_cast<CustomAsyncClient *>(pCustom);
        custom_client->onData(data, len);
    }

    static void _onConnect(void* _, AsyncClient* client) {
        auto* custom_client = static_cast<CustomAsyncClient *>(_);
        custom_client->onConnect();
    }

    static void _onDisconnect(void* _, AsyncClient* client) {
        auto* custom_client = static_cast<CustomAsyncClient *>(_);
        custom_client->onDisconnect();
    }

    static void _onError(void* _, AsyncClient* client, err_t error) {
        auto* custom_client = static_cast<CustomAsyncClient *>(_);
        custom_client->onError(error);
    }

    void onData(void *data, size_t len);

    void onConnect();

    void onDisconnect();

    void onError(err_t error);

    void onCheck(CheckStatus status, std::shared_ptr<IPacket> packet);

public:

    CustomAsyncClient(AsyncClient* client);

    void registerDataListener(const std::function<bool(std::vector<uint8_t>)> &data_listener);

    void registerConnectListener(const std::function<bool()> &connect_listener);

    void registerDisconnectListener(const std::function<bool()> &disconnect_listener);

    void registerErrorListener(const std::function<bool(err_t error)> &error_listener);

    std::shared_ptr<PacketDispatcher> getPacketDispatcher();

    void sendPacket(std::shared_ptr<IPacket> packet);

    void sendPing(uint32_t id);

    template<typename PacketType>
    void registerPacketListener(const std::function<bool(std::shared_ptr<PacketType>)> &listener) {
        packetDispatcher->registerCallBack(listener);
    }

    AsyncClient * getClient() const {
        return client;
    }
};



#endif //CUSTOMASYNCCLIENT_H
