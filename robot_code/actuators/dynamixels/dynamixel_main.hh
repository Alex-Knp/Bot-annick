#include <iostream>
#include <cmath>
#include <chrono>
#include "XL_320_library/XL_320.hpp"
#include "../main/all_struct.hh"

typedef struct DynStruct {
    float gripper_current_angle;
    XL_320 Servo;
} DynStruct ;

int left_dyn_set_angle(DynStruct* dyn, float gripper_goal_angle, float travel_speed);
int home(DynStruct* dyn);
int dyn_init(DynStruct* left_dyn,DynStruct* right_dyn);
int dyn_go_to(DynStruct *left_dyn,DynStruct *right_dyn, side side, Angle angle, int speed);