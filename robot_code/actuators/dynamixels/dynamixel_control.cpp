#include "dynamixel_control.hh"

bool DynamixelControl::setID(uint8_t id, uint8_t new_id) {
    uint8_t error = 0;
    packetHandler->write1ByteTxRx(portHandler, id, ADDR_ID, new_id, &error);
    if (error != 0) {
        std::cerr << "Failed to set ID!" << std::endl;
        return false;
    }
    return true;
}

bool DynamixelControl::setWheelMode(uint8_t id) {
    uint8_t error = 0;
    error = torqueDisable(id);
    if(error != 0) {
        std::cerr << "Failed to disable torque!" << std::endl;
        return false;
    }
    packetHandler->write1ByteTxRx(portHandler, id, ADDR_CONTROL_MODE, 1, &error);
    error = torqueEnable(id);
    if(error != 0) {
        std::cerr << "Failed to enable torque!" << std::endl;
        return false;
    }
    if (error != 0) {
        std::cerr << "Failed to set wheel mode!" << std::endl;
        return false;
    }
    
    return true;
}

bool DynamixelControl::setJointMode(uint8_t id) {
    uint8_t error = 0;
    error = torqueDisable(id);
    if(error != 0) {
        std::cerr << "Failed to disable torque!" << std::endl;
        return false;
    }
    packetHandler->write1ByteTxRx(portHandler, id, ADDR_CONTROL_MODE, 2, &error);
    error = torqueEnable(id);
    if(error != 0) {
        std::cerr << "Failed to enable torque!" << std::endl;
        return false;
    }
    if (error != 0) {
        std::cerr << "Failed to set joint mode!" << std::endl;
        return false;
    }
    return true;
}

bool DynamixelControl::setSpeed(uint8_t id, int speed) {
    uint8_t error = 0;
    packetHandler->write2ByteTxRx(portHandler, id, ADDR_MOVING_SPEED, speed, &error);
    if (error != 0) {
        std::cerr << "Failed to set speed!" << std::endl;
        return false;
    }
    return true;
}

bool DynamixelControl::setAngle(uint8_t id, int angle) {
    uint8_t error = 0;
    packetHandler->write2ByteTxRx(portHandler, id, ADDR_GOAL_POSITION, angle, &error);
    if (error != 0) {
        std::cerr << "Failed to set angle!" << std::endl;
        return false;
    }
    return true;
}

bool DynamixelControl::getTorque(uint8_t id, int16_t &present_load) {
    uint8_t error = 0;
    packetHandler->read2ByteTxRx(portHandler, id, ADDR_PRESENT_LOAD, (uint16_t *)&present_load, &error);
    if (error != 0) {
        std::cerr << "Failed to get present load!" << std::endl;
        return false;
    }
    return true;
}

bool DynamixelControl::torqueEnable(uint8_t id) {
    uint8_t error = 0;
    packetHandler->write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, 1, &error);
    if (error != 0) {
        std::cerr << "Failed to enable torque!" << std::endl;
        return false;
    }
    return true;
}

bool DynamixelControl::torqueDisable(uint8_t id) {
    uint8_t error = 0;
    packetHandler->write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, 0, &error);
    if (error != 0) {
        std::cerr << "Failed to disable torque!" << std::endl;
        return false;
    }
    return true;
}

bool DynamixelControl::isMoving(uint8_t id) {
    uint8_t moving = 0;
    uint8_t error = 0;
    packetHandler->read1ByteTxRx(portHandler, id, ADDR_MOVING, &moving, &error);
    if (error != 0) {
        std::cerr << "Failed to check movement!" << std::endl;
        return false;
    }
    return (moving != 0);
}
