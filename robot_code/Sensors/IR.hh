#pragma one 

//#include "all_struct.hh"
#include "../communication/SPI_spidev.hh"
#include "../communication/SPI_spidev.cpp"
#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>  

// Structure pour représenter un point (x, y)
struct Point {
    int x;
    double y;
};

int micro_switch_tracking(int fd, int micro_switch_id);
//double cubicSplineInterpolation(int x);
double interpolateLinear(const std::vector<Point>& points, int xi);