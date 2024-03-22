#pragma once

#include "../main/all_struct.hh"
#include "../communication/SPI_spidev.hh"
#include "../communication/SPI_spidev.cpp"

int get_right_MS(int fd); 
int get_left_MS(int fd);
int get_emergency_stop(int fd);
void print_MS(int fd);

