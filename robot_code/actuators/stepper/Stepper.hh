#pragma once

//#include "../main/all_struct.hh"
//#include "../main/all_struct.cpp"
//#include "../../communication/SPI_spidev.hh"
//#include "../../communication/SPI_spidev.cpp"
#include "../../Sensors/Micro_switch.hh"

//enum side {left, right};
enum side {left, right};

int stepper_homing(int fd, int stepper_id); 
int stepper_go_to(int fd, int stepper_id, double depth, int speed_coef);
double get_stepper_position(int fd, side stepper_id);
