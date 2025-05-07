//
// Created by fogoz on 03/05/2025.
//

#ifndef PAMITEENSY_TCPSTATEMACHINE_H
#define PAMITEENSY_TCPSTATEMACHINE_H

#include "Teensy41_AsyncTCP.hpp"
#include "NetworkUtils.h"
#include <functional>
#include <FXUtil.h>

#include "PacketHandler.h"
#include "BasePacket.h"
#include "TCPTeensyUpdater.h"
#include "network/CustomAsyncClient.h"


class TCPStateMachine {
    std::shared_ptr<CustomAsyncClient> client;
    PacketHandler& packetHandler;
    TCPTeensyUpdater updater;
    std::vector<std::function<void(AsyncClient * client, CheckStatus status)>> issue;
public:
    TCPStateMachine(PacketHandler& packetHandler, std::shared_ptr<CustomAsyncClient> client);

    void handleData(AsyncClient * client, void * data, size_t len);

    void registerListeners();

    void sendPacket(AsyncClient * client, std::shared_ptr<IPacket> packet);


};


#endif //PAMITEENSY_TCPSTATEMACHINE_H
