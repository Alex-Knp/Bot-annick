#ifndef DYNAMIXEL_CONTROL_H
#define DYNAMIXEL_CONTROL_H

#include <iostream>
#include <unistd.h>
#include <include/dynamixel_sdk/dynamixel_sdk.h>

// Define communication parameters
#define DEVICENAME "/dev/ttyUSB0"
#define BAUDRATE 1000000
#define PROTOCOL_VERSION 2.0

// Define Dynamixel parameters
#define ADDR_ID 3
#define ADDR_CONTROL_MODE 11
#define ADDR_TORQUE_ENABLE 24
#define ADDR_GOAL_POSITION 30
#define ADDR_MOVING_SPEED 32
#define ADDR_PRESENT_POSITION 37
#define ADDR_PRESENT_VELOCITY 39
#define ADDR_PRESENT_LOAD 41
#define ADDR_MOVING 49

class DynamixelControl {
private:
    dynamixel::PortHandler *portHandler;
    dynamixel::PacketHandler *packetHandler;

public:
    DynamixelControl(DEVICENAME, BAUDRATE, PROTOCOL_VERSION) {
        portHandler = dynamixel::PortHandler::getPortHandler(devicename);
        packetHandler = dynamixel::PacketHandler::getPacketHandler(protocol_version);

        if (!portHandler->openPort()) {
            std::cerr << "Failed to open the port!" << std::endl;
            // Handle error
        }

        if (!portHandler->setBaudRate(baudrate)) {
            std::cerr << "Failed to set the baudrate!" << std::endl;
            // Handle error
        }
    }

    ~DynamixelControl() {
        portHandler->closePort();
        delete portHandler;
        delete packetHandler;
    }

    bool setID(uint8_t id, uint8_t new_id);
    bool setWheelMode(uint8_t id);
    bool setJointMode(uint8_t id, int cw_limit, int ccw_limit);
    bool setSpeed(uint8_t id, int speed);
    bool setAngle(uint8_t id, int angle);
    bool getTorque(uint8_t id);
    bool torqueEnable(uint8_t id);
    bool torqueDisable(uint8_t id);
    bool isMoving(uint8_t id);
};

#endif // DYNAMIXEL_CONTROL_H
