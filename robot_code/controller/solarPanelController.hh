#include <iostream>
#include <chrono>

#pragma once
#include "../Sensors/IR.cpp"

struct panel_controller_struct {
    float previous_error;
    time_t previous_time;
};

int left_panel_controller(panel_controller_struct *panel_struct,float speed, float distance);
int right_panel_controller(panel_controller_struct *panel_struct,float speed, float distance);