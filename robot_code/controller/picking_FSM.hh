#pragma once

#include "../main/all_struct.hh"
#include "../Sensors/IR.hh"
#include "../communication/SPI_spidev.hh"
#include "../actuators/dynamixels/dynamixel_main.hh"
#include "../actuators/stepper/Stepper.hh"
#include "../actuators/Servo/Servo.hh"


struct Side{
    Fill storage_1;
    Fill storage_2;
    Fill storage_3;
    int is_in_middle;
}; 

typedef struct Storage{
    Side left; 
    Side right;
} Storage;

typedef struct Plant_Manager{
    Storage storage;
    State left_state;
    State right_state;
    int potting_enable;
    Fill type_expected;
    DynStruct* left_dyn;
    DynStruct* right_dyn;
    int fd;
    Angle depot_angle;
} Plant_Manager;

int picking_FSM_left(Plant_Manager* plant_manager);
int init_plant_manager();
Angle drop_slot(Plant_Manager *PM, side side);
Angle depot_slot(Plant_Manager *PM, side side);

float travel_height(Plant_Manager *PM, side side, Angle angle);
int update_fill(Plant_Manager *PM, side side, Angle angle);
int unupdate_fill(Plant_Manager *PM, side side, Angle angle);
