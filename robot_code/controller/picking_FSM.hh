#pragma once

#include "../main/all_struct.hh"
#include "../Sensors/IR.hh"
#include "../communication/SPI_spidev.hh"
#include "../actuators/dynamixels/dynamixel_main.hh"
#include "../actuators/stepper/Stepper.hh"
#include "../actuators/Servo/Servo.hh"
#include "../Sensors/Micro_switch.hh"


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
    DynStruct* dynamixels;
    int fd;
    Angle depot_angle;
    side depot_zone; 
} Plant_Manager;

int picking_FSM(Plant_Manager* plant_manager, side* Side);
int init_plant_manager(Plant_Manager* plant_manager);
int close_plant_manager(Plant_Manager* plant_manager); 
Angle drop_slot(Plant_Manager *PM, side side);

float travel_height(Plant_Manager *PM, side side, Angle angle);
int update_fill(Plant_Manager *PM, side side, Angle angle);

Angle depot_slot(Plant_Manager *PM, side side);
int update_unfill(Plant_Manager *PM, side side, Angle angle);
double grab_height(Plant_Manager *PM, side side, Angle angle);
int is_full(Plant_Manager *PM, side Side); 

void left_gripper_thread(Plant_Manager* plant_manager);
void right_gripper_thread(Plant_Manager* plant_manager);


