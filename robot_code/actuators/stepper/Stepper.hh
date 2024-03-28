#pragma once

#include "../../main/all_struct.hh"
#include "../../communication/SPI_spidev.hh"

int stepper_homing(int fd, int stepper_id); 
int stepper_go_to(int fd, side stepper_id, double depth, int speed_coef);
double get_stepper_position(int fd, side stepper_id);
