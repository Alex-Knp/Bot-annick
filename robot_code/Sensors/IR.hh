#pragma one 

//#include "all_struct.hh"

#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>  
#include "time.h"
#include "../controller/motor_ask.hh"
// #include "../communication/SPI_spidev.hh"
// #include "../communication/SPI_spidev.cpp"

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