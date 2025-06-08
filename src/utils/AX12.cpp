//
// Created by fogoz on 26/04/2025.
//

#include "utils/AX12.h"
#include <chrono>

using namespace std::chrono;
std::vector<uint8_t> AX12Handler::AX12::sendCommand(std::vector<uint8_t> data) {
    //Empty buffer that could lead to errors

    communicationMutex->lock();
    while (serial.available() > 0) {
        Serial.printf("%02x\r\n", serial.read());
        serial.read();
    }
    data.push_back(computeChecksum(data));
    std::vector<uint8_t> res;
    if (data.size() < 2) {
#if DEBUG_AX12
        Serial.println("CUSTOM_AX12_ERROR_EMPTY_INPUT");
#endif
        return {CUSTOM_AX12_ERROR_EMPTY_INPUT};
    }

    serial.write(0xFF);
    serial.write(0xFF);
    serial.write(id);
    serial.write(data.size());
    for (auto a : data) {
        serial.write(a);
    }
    serial.flush();
    auto data_point = steady_clock::now();
    while (serial.available() <= 4 && steady_clock::now() - data_point < 5ms) {
        Threads::yield();
    }
    if (steady_clock::now() - data_point > 5ms) {
        communicationMutex->unlock();
#if DEBUG_AX12
        Serial.println("CUSTOM_AX12_ERROR_TIMEOUT");
#endif
        return {CUSTOM_AX12_ERROR_TIMEOUT};
    }
    for (int i = 0; i < 2; i++) {
        if (serial.read() != 0xFF) {
            communicationMutex->unlock();
#if DEBUG_AX12
            Serial.println("CUSTOM_AX12_ERROR_HEADER");
#endif
            return {CUSTOM_AX12_ERROR_HEADER};
        }
    }
    uint8_t id = serial.read();
    if (this->id != id) {
#if DEBUG_AX12
        Serial.println("CUSTOM_AX12_ERROR_WRONG_ID");
#endif
        return {CUSTOM_AX12_ERROR_WRONG_ID};
    }


    uint8_t result_size = serial.read();
    if (result_size < 2 ) {
        communicationMutex->unlock();
#if DEBUG_AX12
        Serial.println("CUSTOM_AX12_ERROR_SIZE_TOO_SMALL");
#endif
        return {CUSTOM_AX12_ERROR_SIZE_TOO_SMALL};
    }
    data_point = steady_clock::now();
    while (serial.available() < result_size&& steady_clock::now() - data_point < 10ms) {
        Threads::yield();
    }
    if (steady_clock::now() - data_point > 10ms) {
        communicationMutex->unlock();
#if DEBUG_AX12
        Serial.println("CUSTOM_AX12_ERROR_TIMEOUT_PARAM");
#endif
        return {CUSTOM_AX12_ERROR_TIMEOUT_PARAM};
    }

    for (uint8_t i = 0; i < result_size; i++) {
        res.push_back(static_cast<uint8_t>(serial.read()));
    }
    communicationMutex->unlock();

    if (result_size == res.size()) {
        uint8_t checksum = res.back();
        res.pop_back();
        if (computeChecksum(res) == checksum) {
            res.push_back(checksum);
#if DEBUG_AX12
            Serial.println("CUSTOM_AX12_SUCCESS");
#endif
            return res;
        }
#if DEBUG_AX12
        Serial.println("CUSTOM_AX12_ERROR_CHECKSUM");
#endif
        return {CUSTOM_AX12_ERROR_CHECKSUM};
    }
#if DEBUG_AX12
    Serial.println("CUSTOM_AX12_ERROR_SIZE");
#endif
    return {CUSTOM_AX12_ERROR_SIZE};
}

int AX12Handler::AX12::readAX12(uint8_t address, uint8_t size) {
    std::vector<uint8_t> data = sendCommand({AX12_INSTRUCTION_READ, address, size});
    if (data.size() > 1) {
        int result = 0;
        for (uint8_t i = 0; i < size; i++) {
            result += data[i+1] << ((i) * 8);
        }
        return result;
    }
    Serial.printf("Error AX %d reading address %02x with size %d, the error is %d \r\n", id, address, size, data[0]);
    return -1;
}

int AX12Handler::AX12::writeAX12(uint8_t address, std::vector<uint8_t> data) {
    std::vector<uint8_t> command({AX12_INSTRUCTION_WRITE, address});
    for (const auto& a : data) {
        command.push_back(a);
    }
    std::vector<uint8_t> result = sendCommand(command);
    if (result.size() > 1) {
        return result[0];
    }
    Serial.printf("Error AX %d writting address %d with size %d, the error is %d \r\n", id, address, result.size(), result[0]);
    return -1;
}

int AX12Handler::AX12::writeAX12(uint8_t address, uint8_t data) {
    return writeAX12(address, std::vector<uint8_t>({data}));
}

int AX12Handler::AX12::writeAX12(uint8_t address, uint16_t data) {
    return writeAX12(address, std::vector<uint8_t>({static_cast<uint8_t>(data&0xFF), static_cast<uint8_t>(data>>8)}));
}

int AX12Handler::ping(int id) const {
    std::vector<uint8_t> data = get(id).sendCommand({AX12_INSTRUCTION_PING, id});
    if (data.size() > 1) {
        return data[0];
    }
    return -1;
}

AX12Handler::AX12::AX12(int id, ThreadSafeSerial &serial, std::shared_ptr<std::mutex> m) : id(id), serial(serial), communicationMutex(m) {

}

uint8_t AX12Handler::AX12::computeChecksum(const std::vector<uint8_t>& data) const {
    uint8_t length = data.size() + 1;
    int checksum = id + length;
    for (const auto a : data) {
        checksum += a;
    }
    return (~checksum) & 0xFF;
}

AX12Handler::AX12Handler(ThreadSafeSerial &serial, int baudrate) : serial(serial), baudrate(baudrate) {
    serial.begin(baudrate, SERIAL_8N1 | SERIAL_HALF_DUPLEX);
    communicationMutex = std::make_shared<std::mutex>();
}

void AX12Handler::setBaudrate(int baudrate) {
    this->baudrate = baudrate;
    serial.end();
    serial.begin(baudrate, SERIAL_8N1 | SERIAL_HALF_DUPLEX);
}

AX12Handler::AX12 AX12Handler::get(int id) const {
    return AX12(id, serial, communicationMutex);
}
