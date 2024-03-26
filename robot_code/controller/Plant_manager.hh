#pragma once

//#include "../main/all_struct.hh"
#include "../actuators/Servo/Servo.hh"

//enum side {left, right};

int init_claw(int fd, side Side);
void take_plant(int fd, side Side);
void store_plant(int fd, side Side, int stock_id);
void drop_plant(int fd, side Side);
