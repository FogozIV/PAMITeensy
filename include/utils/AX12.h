//
// Created by fogoz on 26/04/2025.
//

#ifndef AX12_H
#define AX12_H
#include <Arduino.h>
#include <memory>

#include "TeensyThreads.h"

/**
 * @defgroup AX12_Instructions AX12 Instruction Set
 * @{
 */
#define AX12_INSTRUCTION_PING 0x01         ///< Ping instruction
#define AX12_INSTRUCTION_READ 0x02         ///< Read data instruction
#define AX12_INSTRUCTION_WRITE 0x03        ///< Write data instruction
#define AX12_INSTRUCTION_REG_WRITE 0x04    ///< Register write instruction
#define AX12_INSTRUCTION_ACTION 0x05       ///< Action instruction
#define AX12_INSTRUCTION_FACTORY_RESET 0x06 ///< Factory reset instruction
#define AX12_INSTRUCTION_REBOOT 0x07       ///< Reboot instruction
#define AX12_INSTRUCTION_SYNC_WRITE 0x83   ///< Synchronized write instruction
#define AX12_BULK_READ 0x92               ///< Bulk read instruction
/** @} */

#define DEBUG_AX12 false

/**
 * @brief Control table definition for AX12 servos
 * 
 * This macro defines all the control registers available in the AX12 servo,
 * including their address, size, and whether they are writable.
 */
#define AX12_CONTROL_TABLE \
    AX12_CONTROL(0, 2, MODEL_NUMBER, false) \
    AX12_CONTROL(2, 1, FIRMWARE_VERSION, false) \
    AX12_CONTROL(3, 1, ID, true) \
    AX12_CONTROL(4, 1, BAUD_RATE, true) \
    AX12_CONTROL(5, 1, RETURN_DELAY_TIME, true) \
    AX12_CONTROL(6, 2, CW_ANGLE_LIMIT, true) \
    AX12_CONTROL(8, 2, CCW_ANGLE_LIMIT, true) \
    AX12_CONTROL(11, 1, TEMPERATURE_LIMIT, true) \
    AX12_CONTROL(12, 1, MIN_VOLTAGE_LIMIT, true) \
    AX12_CONTROL(13, 1, MAX_VOLTAGE_LIMIT, true) \
    AX12_CONTROL(14, 2, MAX_TORQUE, true) \
    AX12_CONTROL(16, 1, STATUS_RETURN_LEVEL, true) \
    AX12_CONTROL(17, 1, ALARM_LED, true) \
    AX12_CONTROL(18, 1, SHUTDOWN, true)\
    AX12_CONTROL(24, 1, TORQUE_ENABLE, true) \
    AX12_CONTROL(25, 1, LED, true) \
    AX12_CONTROL(26, 1, CW_COMPLIANCE_MARGIN, true) \
    AX12_CONTROL(27, 1, CCW_COMPLIANCE_MARGIN, true) \
    AX12_CONTROL(28, 1, CW_COMPLIANCE_SLOPE, true) \
    AX12_CONTROL(29, 1, CCW_COMPLIANCE_SLOPE, true) \
    AX12_CONTROL(30, 2, GOAL_POSITION, true) \
    AX12_CONTROL(32, 2, MOVING_SPEED, true) \
    AX12_CONTROL(34, 2, TORQUE_LIMIT, true) \
    AX12_CONTROL(36, 2, PRESENT_POSITION, false) \
    AX12_CONTROL(38, 2, PRESENT_SPEED, false) \
    AX12_CONTROL(40, 2, PRESENT_LOAD, false) \
    AX12_CONTROL(42, 2, PRESENT_VOLTAGE, false) \
    AX12_CONTROL(43, 1, PRESENT_TEMPERATURE, false) \
    AX12_CONTROL(44, 1, REGISTERED, false) \
    AX12_CONTROL(46, 1, MOVING, false) \
    AX12_CONTROL(47, 1, LOCK, true) \
    AX12_CONTROL(48, 2, PUNCH, true)

#define AX12_CONTROL(address, size, name, writable) AX12_CONTROL_##writable(address, size, name)

#define AX12_CONTROL_false(address, size, name)\
    int read##name(){\
        return readAX12(address, size);\
    }

#define AX12_CONTROL_true(address, size, name) AX12_CONTROL_true_##size(address, size, name)

#define AX12_CONTROL_true_1(address, size, name) \
    int read##name(){\
    return readAX12(address, size);\
    };\
    int write##name(uint8_t data) {\
        return writeAX12(address, data); \
    }
#define AX12_CONTROL_true_2(address, size, name)  \
    int read##name(){\
    return readAX12(address, size);\
    };\
    int write##name(uint16_t data) {\
    return writeAX12(address, data); \
    }

/**
 * @defgroup AX12_Custom_Errors Custom AX12 Error Codes
 * @{
 */
#define CUSTOM_AX12_ERROR_TIMEOUT 0x01          ///< Communication timeout error
#define CUSTOM_AX12_ERROR_HEADER 0x02           ///< Invalid header error
#define CUSTOM_AX12_ERROR_SIZE_NOT_MATCHING 0x03 ///< Response size mismatch error
#define CUSTOM_AX12_ERROR_CHECKSUM 0x04         ///< Checksum verification failed
#define CUSTOM_AX12_ERROR_SIZE_TOO_SMALL 0x05   ///< Response size too small
#define CUSTOM_AX12_ERROR_SIZE 0x06             ///< Invalid size error
#define CUSTOM_AX12_ERROR_EMPTY_INPUT 0x07      ///< Empty input error
#define CUSTOM_AX12_ERROR_TIMEOUT_PARAM 0x08    ///< Invalid timeout parameter
#define CUSTOM_AX12_ERROR_WRONG_ID 0x09         ///< Wrong servo ID error
/** @} */

/**
 * @brief Error message generator for AX12 servos
 * 
 * This macro generates error messages for standard AX12 error codes
 */
#define AX12_ERRORS_GENERATOR \
    AX12_ERROR("Weird error that should not happend", 128)\
    AX12_ERROR("Instruction Error", 64)\
    AX12_ERROR("Overloard Error", 32)\
    AX12_ERROR("Checksum Error", 16)\
    AX12_ERROR("Range Error", 8)\
    AX12_ERROR("Overheating error", 4)\
    AX12_ERROR("Angle Limit Error", 2)\
    AX12_ERROR("Input Voltage Error", 1)

/**
 * @brief Prints human-readable error messages for AX12 status codes
 * 
 * @param status The status byte from the AX12 servo
 * @param serial Serial interface for output
 */
inline void printAX12Error(uint8_t status, Stream& serial) {
    AX12_ERRORS_GENERATOR
    if (status == 0) {
        serial.println("No error");
    }
}

/**
 * @brief Handler class for managing AX12 servo communications
 * 
 * This class manages the serial communication with AX12 servos and
 * provides a factory for creating AX12 instances.
 */
class AX12Handler {
protected:
    HardwareSerialIMXRT &serial;  ///< Serial interface for communication
    int baudrate;                  ///< Communication baud rate
    std::shared_ptr<std::mutex> communicationMutex;  ///< Mutex for thread-safe communication
public:
    /**
     * @brief Class representing an individual AX12 servo
     * 
     * This class provides methods for reading from and writing to
     * the control table of a specific AX12 servo.
     */
    class AX12 {
        int id;  ///< Servo ID
        HardwareSerialIMXRT& serial;  ///< Serial interface reference
        std::shared_ptr<std::mutex> communicationMutex;  ///< Communication mutex reference

    protected:
        /**
         * @brief Constructs a new AX12 instance
         * 
         * @param id Servo ID
         * @param serial Serial interface for communication
         * @param m Mutex for thread-safe communication
         */
        AX12(int id, HardwareSerialIMXRT& serial, std::shared_ptr<std::mutex> m);

        /**
         * @brief Computes checksum for AX12 packets
         * 
         * @param data Packet data
         * @return uint8_t Computed checksum
         */
        uint8_t computeChecksum(const std::vector<uint8_t>& data) const;

    public:
        AX12(const AX12&) = default;
        AX12(AX12&&) = default;
        AX12& operator=(const AX12&) = default;
        AX12& operator=(AX12&&) = default;
        friend class AX12Handler;

        /**
         * @brief Sends a command to the servo
         * 
         * @param data Command data packet
         * @return std::vector<uint8_t> Response from servo
         */
        std::vector<uint8_t> sendCommand(std::vector<uint8_t> data);

        /**
         * @brief Reads data from the servo's control table
         * 
         * @param address Control table address
         * @param size Number of bytes to read
         * @return int Read value or error code
         */
        int readAX12(uint8_t address, uint8_t size);

        /**
         * @brief Writes data to the servo's control table
         * 
         * @param address Control table address
         * @param data Data to write
         * @return int Status code
         */
        int writeAX12(uint8_t address, std::vector<uint8_t> data);
        int writeAX12(uint8_t address, uint8_t data);
        int writeAX12(uint8_t address, uint16_t data);

        // Control table access methods generated by AX12_CONTROL_TABLE macro
        AX12_CONTROL_TABLE
    };

public:
    /**
     * @brief Constructs a new AX12Handler
     * 
     * @param serial Serial interface for communication
     * @param baudrate Communication baud rate
     */
    AX12Handler(HardwareSerialIMXRT &serial, int baudrate);

    /**
     * @brief Creates an AX12 instance for a specific servo
     * 
     * @param id Servo ID
     * @return AX12 Servo instance
     */
    AX12 get(int id) const;
};

#undef AX12_ERROR
#undef AX12_CONTROL_true_2
#undef AX12_CONTROL_true_1
#undef AX12_CONTROL_true
#undef AX12_CONTROL_false
#undef AX12_CONTROL
#endif //AX12_H
