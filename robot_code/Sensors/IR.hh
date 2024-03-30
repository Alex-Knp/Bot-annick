#pragma once 

#include "../main/all_struct.hh"
#include "../communication/SPI_spidev.hh"

// Structure pour représenter un point (x, y)
struct Point {
    int x;
    double y;
};

int simple_IR_ask(int fd, int IR_id);
double IR_Wall_tracking(int IR_id);
double interpolateLinear(int xi);
int If_pot_IR(int fd);
int If_plant_IR(int fd);
int If_double_pot(int fd);
int wait_for_plant(int fd);
int wait_for_pot(int fd);