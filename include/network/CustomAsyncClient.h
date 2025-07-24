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

/**
 * @brief Asynchronous TCP client with packet handling
 * 
 * This class implements an asynchronous TCP client that provides:
 * - Packet-based communication
 * - Event-driven architecture
 * - Type-safe packet handling
 * - Connection management
 * - Error handling
 * 
 * Features:
 * - Automatic packet parsing
 * - Event listener registration
 * - Connection state tracking
 * - Error reporting
 * - Ping functionality
 */
class CustomAsyncClient {
    AsyncClient* client;                  ///< Underlying TCP client
    std::shared_ptr<PacketHandler> packet_handler = std::make_shared<PacketHandler>();  ///< Packet parsing/handling
    std::shared_ptr<PacketDispatcher> packetDispatcher = std::make_shared<PacketDispatcher>();  ///< Packet dispatch
    TCPTeensyUpdater updater;                      ///< Firmware update handler

    std::vector<std::function<bool(std::vector<uint8_t>)>> data_listeners{};     ///< Raw data listeners
    std::vector<std::function<bool()>> connect_listeners{};                      ///< Connection listeners
    std::vector<std::function<bool()>> disconnect_listeners{};                   ///< Disconnection listeners
    std::vector<std::function<bool(err_t error)>> error_listeners{};            ///< Error listeners
    std::vector<std::function<bool(CheckStatus status, std::shared_ptr<IPacket> packet)>> check_listeners {};  ///< Packet check listeners

    /**
     * @brief Static data callback wrapper
     * @param pCustom Client instance pointer
     * @param client TCP client pointer
     * @param data Received data
     * @param len Data length
     */
   static void _onData(void* pCustom, AsyncClient* client, void *data, size_t len) {
        auto* custom_client = static_cast<CustomAsyncClient *>(pCustom);
        custom_client->onData(data, len);
    }

    /**
     * @brief Static connect callback wrapper
     * @param _ Client instance pointer
     * @param client TCP client pointer
     */
    static  void _onConnect(void* _, AsyncClient* client) {
        auto* custom_client = static_cast<CustomAsyncClient *>(_);
        custom_client->onConnect();
    }

    /**
     * @brief Static disconnect callback wrapper
     * @param _ Client instance pointer
     * @param client TCP client pointer
     */
    static  void _onDisconnect(void* _, AsyncClient* client) {
        auto* custom_client = static_cast<CustomAsyncClient *>(_);
        custom_client->onDisconnect();
    }

    /**
     * @brief Static error callback wrapper
     * @param _ Client instance pointer
     * @param client TCP client pointer
     * @param error Error code
     */
    static  void _onError(void* _, AsyncClient* client, err_t error) {
        auto* custom_client = static_cast<CustomAsyncClient *>(_);
        custom_client->onError(error);
    }

    /**
     * @brief Handles received data
     * @param data Received data buffer
     * @param len Data length
     */
    FASTRUN void onData(void *data, size_t len);

    /**
     * @brief Handles connection events
     */
    void onConnect();

    /**
     * @brief Handles disconnection events
     */
    void onDisconnect();

    /**
     * @brief Handles error events
     * @param error Error code
     */
    void onError(err_t error);

    /**
     * @brief Handles packet validation
     * @param status Validation status
     * @param packet Validated packet
     */
    void onCheck(CheckStatus status, std::shared_ptr<IPacket> packet);

public:
    /**
     * @brief Constructs a new client
     * @param client Underlying TCP client
     */
    CustomAsyncClient(AsyncClient* client);

    /**
     * @brief Sends a ping packet
     * @param id Ping identifier
     */
    void sendPing(uint32_t id);

    /**
     * @brief Registers a typed packet listener
     * 
     * @tparam PacketType Type of packet to listen for
     * @param listener Callback function
     */
    template<typename PacketType>
    void registerPacketListener(const std::function<bool(std::shared_ptr<PacketType>)> &listener) {
        packetDispatcher->registerCallBack(listener);
    }

    /**
     * @brief Registers a raw data listener
     * @param data_listener Callback function
     */
    void registerDataListener(const std::function<bool(std::vector<uint8_t>)> &data_listener);

    /**
     * @brief Registers a connection listener
     * @param connect_listener Callback function
     */
    void registerConnectListener(const std::function<bool()> &connect_listener);

    /**
     * @brief Registers a disconnection listener
     * @param disconnect_listener Callback function
     */
    void registerDisconnectListener(const std::function<bool()> &disconnect_listener);

    /**
     * @brief Registers an error listener
     * @param error_listener Callback function
     */
    void registerErrorListener(const std::function<bool(err_t error)> &error_listener);

    /**
     * @brief Gets the packet dispatcher
     * @return std::shared_ptr<PacketDispatcher> Packet dispatcher
     */
    FASTRUN std::shared_ptr<PacketDispatcher> getPacketDispatcher();

    /**
     * @brief Sends a packet
     * @param packet Packet to send
     */
    void FASTRUN sendPacket(std::shared_ptr<IPacket> packet);

    /**
     * @brief Gets the underlying TCP client
     * @return AsyncClient* TCP client pointer
     */
    AsyncClient * getClient() const {
        return client;
    }
};

#endif //CUSTOMASYNCCLIENT_H
