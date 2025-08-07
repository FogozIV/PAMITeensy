//
// Created by fogoz on 06/05/2025.
//


#include "network/CustomAsyncClient.h"

#include <Arduino.h>
#include <SD.h>

#include "curves/CurveFactory.h"
#include "target/ContinuousCurveTarget.h"
#include "target/FunctionTarget.h"
#include "utils/Regex.h"
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

    registerDataListener([this](const std::vector<uint8_t> &data) -> bool {
        packet_handler->receiveData(data);
        CheckStatus result = CheckStatus::BAD_CRC;
        std::shared_ptr<IPacket> packet;
        while (result != CheckStatus::WAITING_DATA && result != CheckStatus::WAITING_LENGTH) {
            auto a = packet_handler->checkPacket();
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
        streamSplitter.println(PSTR("Received start flash packet"));
        if (updater.isFlashing()) {
            sendPacket(std::make_shared<AlreadyFlashingPacket>());
            return false;
        }
        if (updater.startFlashMode()) {
            streamSplitter.println(PSTR("Flashing started"));
            sendPacket(std::make_shared<StartFlashPacket>());
            packetDispatcher->registerCallBack<DataPacket>([this](std::shared_ptr<DataPacket> packet) {
                //streamSplitter.println(PSTR("Received data packet"));
                updater.addData(reinterpret_cast<const char *>(packet->getDataRef().data()), packet->getDataRef().size());
                if (!updater.parse()) {
                    sendPacket(std::make_shared<IssueFlashingPacket>());
                    streamSplitter.println(PSTR("Issue flashing"));
                    return true;
                }
                if (updater.isDone()) {
                    sendPacket(std::make_shared<FlashingSoftwarePacket>());
                    streamSplitter.println(PSTR("Flashing software"));
                    updater.callDone();
                    return true;
                }
                //streamSplitter.println(PSTR("Flashing in progress sending ack"));
                sendPacket(std::make_shared<ReceivedDataPacket>(packet->getDataRef().size()));
                return false;
            });

        }else {
            sendPacket(std::make_shared<IssueStartingFlashingPacket>());
        }
        return false;
    });

    packetDispatcher->registerCallBack<FileResponsePacket>([this](std::shared_ptr<FileResponsePacket> packet) {
        std::string filename = f.name();
        sdMutex->lock();
        streamSplitter.println(PSTR("Received file response"));
        if (filename == packet->getFilename()) {
            f.write(packet->getData().data(), packet->getData().size());
        }else {
            filename = packet->getFilename();
            if (f) {
                f.close();
            }
            SD.begin();
            SD.remove(filename.c_str());
            f = SD.open(filename.c_str(), FILE_WRITE_BEGIN);
            streamSplitter.println(f);
            streamSplitter.println(filename.c_str());
            f.write(packet->getData().data(), packet->getData().size());
        }
        sendPacket(std::make_shared<ReceivedDataPacket>(0));
        sdMutex->unlock();
        return false;
    });

    packetDispatcher->registerCallBack<DoneSendingFilesPacket>([this](std::shared_ptr<DoneSendingFilesPacket> packet) {
        streamSplitter.println(PSTR("Received done files"));
        sdMutex->lock();
        if (f) {
            f.close();
        }

        sdMutex->unlock();
        return false;
    });

    packetDispatcher->registerCallBack<RequestFilePacket>([this](std::shared_ptr<RequestFilePacket> packet) {
        streamSplitter.printf("Received packet request file %s \r\n", packet->getRegexexp().c_str());
        sdMutex->lock();
        SD.begin();
        File root = SD.open("/");
        const char* pattern = packet->getRegexexp().c_str();
        re_t regex = re_compile(pattern);
        std::vector<String> matches;
        while (true) {
            File file = root.openNextFile();
            if (!file) break;

            if (!file.isDirectory()) {

                String name = file.name();  // This will include the full LFN if available
                int match_length = 0;
                int match_index = re_matchp(regex, name.c_str(), &match_length);
                if (match_index >= 0) {
                    streamSplitter.printf("Matched at %d with length %d Adding: ", match_index, match_length);
                    streamSplitter.println(name);
                    matches.push_back(name);
                }
            }

            file.close();
        }
        root.close();
        sdMutex->unlock();
        std::shared_ptr<SimpleSemaphore> semaphore = std::make_shared<SimpleSemaphore>();

        int callbackid = packetDispatcher->registerCallBack<ReceivedDataPacket>([this, semaphore](std::shared_ptr<ReceivedDataPacket> packet) {
            semaphore->signal();
            return false;
        });

        scheduler->addTask(0ms, [matches, this, semaphore, callbackid]() {
            sdMutex->lock();
            SD.begin();
            std::vector<uint8_t> data(4096);
            for (String match : matches) {
                    streamSplitter.printf("Handling %s\r\n", match.c_str());
                    File f = SD.open(match.c_str());
                    if (!f) {
                        streamSplitter.println(PSTR("File not found"));
                        continue;
                    }
                    int bytesRead;
                    while ((bytesRead = f.read(data.data(), data.size())) > 0) {
                        streamSplitter.printf("Sending %d bytes\r\n", bytesRead);
                        sendPacket(std::make_shared<FileResponsePacket>(match.c_str(), std::vector<uint8_t>(data.begin(), data.begin() + bytesRead)));
                        semaphore->wait();
                    }
            }
            packetDispatcher->removeCallback(ReceivedDataPacket::getPacketID(), callbackid);
            sendPacket(std::make_shared<DoneSendingFilesPacket>());
            sdMutex->unlock();
        });
        return false;
    });

    packetDispatcher->registerCallBack<SendTrajectoryPacket>([](std::shared_ptr<SendTrajectoryPacket> packet) {
        auto trajVector = packet->getTrajectory();
        base_robot->addTarget(std::make_shared<ContinuousCurveTarget<DynamicQuadRamp>>(base_robot, CurveFactory::getBaseCurve(trajVector), RampData(packet->getAcc(), packet->getMaxspeed(), packet->getMaxspeed(), packet->getDec(), packet->getMaxlateralacc())));
        base_robot->setControlDisabled(false);
        return false;
    });

    packetDispatcher->registerCallBack<DisableControlPacket>([](std::shared_ptr<DisableControlPacket> packet) {
        base_robot->setControlDisabled(packet->getDisabled());
        base_robot->getEventEndOfComputeNotifier()->wait();
        base_robot->getLeftMotor()->setPWM(0);
        base_robot->getRightMotor()->setPWM(0);
        return false;
    });

    packetDispatcher->registerCallBack<ClearTargetPacket>([](std::shared_ptr<ClearTargetPacket> packet) {
        base_robot->clearTarget();
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

void FASTRUN CustomAsyncClient::sendPacket(std::shared_ptr<IPacket> packet) {
    auto a = packet_handler->createPacket(packet);
    client->write(reinterpret_cast<const char *>(a.data()), a.size(), TCP_WRITE_FLAG_COPY);
    client->send();
}

void CustomAsyncClient::sendPing(uint32_t id) {
    PingPacket ping(id);
    auto a = packet_handler->createPacket(ping);
    streamSplitter.printf(PSTR("Sending ping with id: %lu\r\n"), id);
    client->write(reinterpret_cast<const char *>(a.data()), a.size(), TCP_WRITE_FLAG_COPY);
    client->send();
    packetDispatcher->registerCallBack<PongPacket>([id](std::shared_ptr<PongPacket> packet) {
        if (packet->getUniqueID() == id) {
            streamSplitter.printf(PSTR("Received pong packet with id %d\r\n"), id);
        }else {
            streamSplitter.printf(PSTR("Received pong packet with id %d but expected %d\r\n"), packet->getUniqueID(), id);
        }
        return true;
    });
}

