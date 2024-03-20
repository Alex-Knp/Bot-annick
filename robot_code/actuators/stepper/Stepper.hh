#pragma once

//#include "all_struct.hh"
#include "../../communication/SPI_spidev.hh"
#include "../../communication/SPI_spidev.cpp"

int stepper_homing(int fd, int stepper_id); 
int stepper_ask(int fd, int stepper_id, double depth, int speed_coef); 
