//
// Created by fogoz on 06/05/2025.
//

#ifndef CUSTOMASYNCCLIENT_H
#define CUSTOMASYNCCLIENT_H
#include "Teensy41_AsyncTCP.hpp"
#include <functional>

#include "PacketHandler.h"
#include "packets/PacketDefinition.h"
#include "utils/PacketDispatcher.h"

class CustomAsyncClient {
    AsyncClient* client = nullptr;
    std::shared_ptr<PacketHandler> packet_handler;
    std::shared_ptr<PacketDispatcher> packetDispatcher = std::make_shared<PacketDispatcher>();

    std::vector<std::function<bool(std::vector<uint8_t>)>> data_listeners{};
    std::vector<std::function<bool()>> connect_listeners{};
    std::vector<std::function<bool()>> disconnect_listeners{};
    std::vector<std::function<bool(err_t error)>> error_listeners{};
    std::vector<std::function<bool(CheckStatus status, std::shared_ptr<IPacket> packet)>> check_listeners {};


    void onData(void* _, AsyncClient* client, void *data, size_t len);

    void onConnect(void* _, AsyncClient* client);

    void onDisconnect(void* _, AsyncClient* client);

    void onError(void* _, AsyncClient* client, err_t error);

    void onCheck(CheckStatus status, std::shared_ptr<IPacket> packet);

public:

    CustomAsyncClient(AsyncClient* client, std::shared_ptr<PacketHandler> handler = std::make_shared<PacketHandler>());

    void registerDataListener(const std::function<bool(std::vector<uint8_t>)> &data_listener);

    void registerConnectListener(const std::function<bool()> &connect_listener);

    void registerDisconnectListener(const std::function<bool()> &disconnect_listener);

    void registerErrorListener(const std::function<bool(err_t error)> &error_listener);

    std::shared_ptr<PacketDispatcher> getPacketDispatcher();

    void sendPacket(std::shared_ptr<IPacket> packet);

    void sendPing(uint32_t id);
};



#endif //CUSTOMASYNCCLIENT_H
