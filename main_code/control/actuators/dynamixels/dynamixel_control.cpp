#include <iostream>
#include "dynamixel_sdk.h"

// Control table addresses for XL320
#define ADDR_GOAL_POSITION   30
#define ADDR_PRESENT_POSITION   36
#define ADDR_MOVING_SPEED    32
#define ADDR_PRESENT_SPEED   38
#define ADDR_TORQUE_ENABLE  24
#define ADDR_PRESENT_LOAD   40
#define ADDR_ID              7
#define PROTOCOL_VERSION    1.0     // Dynamixel protocol version 1.0

// Default settings
#define BAUDRATE            1000000 // Dynamixel baudrate
#define DEVICENAME          "/dev/ttyUSB0" // Check your device name
#define TORQUE_ENABLE       1       // Value for enabling the torque
#define TORQUE_DISABLE      0       // Value for disabling the torque

// Initialize PortHandler instance
// Set the port path
// Get methods and members of PortHandlerLinux or PortHandlerWindows
dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

// Initialize PacketHandler instance
// Set the protocol version
// Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

bool initialize() {
    // Open port
    if (!portHandler->openPort()) {
        std::cout << "Failed to open the port!" << std::endl;
        return false;
    }

    // Set port baudrate
    if (!portHandler->setBaudRate(BAUDRATE)) {
        std::cout << "Failed to set the baudrate!" << std::endl;
        return false;
    }

    return true;
}

void setID(uint8_t new_id, uint8_t old_id) {
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;  // Communication result

    // Write new ID
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, old_id, ADDR_ID, new_id, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
        std::cout << "Failed to set ID: " << packetHandler->getTxRxResult(dxl_comm_result) << std::endl;
    } else if (dxl_error != 0) {
        std::cout << "Dynamixel error: " << packetHandler->getRxPacketError(dxl_error) << std::endl;
    }
}

void setPosition(int goal_position, uint8_t id) {
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;  // Communication result

    // Switch to position control mode (assuming the control mode address is ADDR_CONTROL_MODE)
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, id, ADDR_CONTROL_MODE, POSITION_CONTROL_MODE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
        std::cout << "Failed to switch to position control mode: " << packetHandler->getTxRxResult(dxl_comm_result) << std::endl;
        return;
    } else if (dxl_error != 0) {
        std::cout << "Dynamixel error: " << packetHandler->getRxPacketError(dxl_error) << std::endl;
        return;
    }

    // Write goal position
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, id, ADDR_GOAL_POSITION, goal_position, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
        std::cout << "Failed to write goal position: " << packetHandler->getTxRxResult(dxl_comm_result) << std::endl;
    } else if (dxl_error != 0) {
        std::cout << "Dynamixel error: " << packetHandler->getRxPacketError(dxl_error) << std::endl;
    }
}

void setSpeed(uint16_t moving_speed, uint8_t id) {
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;  // Communication result

    // Switch to speed control mode (assuming the control mode address is ADDR_CONTROL_MODE)
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, id, ADDR_CONTROL_MODE, SPEED_CONTROL_MODE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
        std::cout << "Failed to switch to speed control mode: " << packetHandler->getTxRxResult(dxl_comm_result) << std::endl;
        return;
    } else if (dxl_error != 0) {
        std::cout << "Dynamixel error: " << packetHandler->getRxPacketError(dxl_error) << std::endl;
        return;
    }

    // Write moving speed
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, id, ADDR_MOVING_SPEED, moving_speed, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
        std::cout << "Failed to write moving speed: " << packetHandler->getTxRxResult(dxl_comm_result) << std::endl;
    } else if (dxl_error != 0) {
        std::cout << "Dynamixel error: " << packetHandler->getRxPacketError(dxl_error) << std::endl;
    }
}

int32_t readPresentPosition(uint8_t id) {
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;  // Communication result
    int32_t present_position = 0;

    // Read present position
    dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, id, ADDR_PRESENT_POSITION, (uint16_t*)&present_position, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
        std::cout << "Failed to read present position: " << packetHandler->getTxRxResult(dxl_comm_result) << std::endl;
        return -1;
    } else if (dxl_error != 0) {
        std::cout << "Dynamixel error: " << packetHandler->getRxPacketError(dxl_error) << std::endl;
        return -1;
    }

    return present_position;
}

int16_t readPresentSpeed(uint8_t id) {
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;  // Communication result
    int16_t present_speed = 0;

    // Read present speed
    dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, id, ADDR_PRESENT_SPEED, (uint16_t*)&present_speed, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
        std::cout << "Failed to read present speed: " << packetHandler->getTxRxResult(dxl_comm_result) << std::endl;
        return -1;
    } else if (dxl_error != 0) {
        std::cout << "Dynamixel error: " << packetHandler->getRxPacketError(dxl_error) << std::endl;
        return -1;
    }

    return present_speed;
}

int16_t readPresentTorque(uint8_t id) {
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;  // Communication result
    int16_t present_torque = 0;

    // Read present torque
    dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, id, ADDR_PRESENT_LOAD, (uint16_t*)&present_torque, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
        std::cout << "Failed to read present torque: " << packetHandler->getTxRxResult(dxl_comm_result) << std::endl;
        return -1;
    } else if (dxl_error != 0) {
        std::cout << "Dynamixel error: " << packetHandler->getRxPacketError(dxl_error) << std::endl;
        return -1;
    }

    return present_torque;
}

void disableTorque(uint8_t id) {
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;  // Communication result

    // Disable Dynamixel Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
        std::cout << "Failed to disable Dynamixel torque: " << packetHandler->getTxRxResult(dxl_comm_result) << std::endl;
    } else if (dxl_error != 0) {
        std::cout << "Dynamixel error: " << packetHandler->getRxPacketError(dxl_error) << std::endl;
    } else {
        std::cout << "Dynamixel torque disabled!" << std::endl;
    }
}

int homing(uint8_t id) {
    // Set moving speed
    setSpeed(100);



    return 0;
}

int main() {
    if (!initialize()) {
        return 1;
    }

    // Example usage:
    // setID(2);
    // setPosition(512);
    // setSpeed(100);
    // int32_t position = readPresentPosition(2);
    // int16_t speed = readPresentSpeed(2);
    // int16_t torque = readPresentTorque(2);
    // disableTorque(2);

    // Your code logic here...

    return 0;
}
