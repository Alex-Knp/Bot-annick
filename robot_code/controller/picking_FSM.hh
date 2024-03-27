#pragma once

#include "../main/all_struct.hh"
#include "../Sensors/IR.hh"
#include "../communication/SPI_spidev.hh"
#include "../actuators/dynamixels/dynamixel_main.hh"
#include "../actuators/stepper/Stepper.hh"
#include "../actuators/Servo/Servo.hh"

typedef struct Storage{
    Side left; 
    Side right;
} Storage;

typedef struct Side{
    Fill storage_1;
    Fill storage_2;
    Fill storage_3;
    int is_in_middle;
} Side; 

typedef struct Plant_Manager{
    Storage storage;
    State left_state;
    State right_state;
    int potting_enable;
    Fill type_expected;
} Plant_Manager;

int picking_FSM(Plant_Manager* plant_manager);
int plant_manager_init();