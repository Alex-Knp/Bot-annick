#include "solarPanelController.h"

int left_panel_controller(float speed, float goal_distance){
    // This function is used to control the distance between the robot and the wall using the IR sensor
    // It will input the speed and distance from the wall at which the robot has to travel
    // It will output the speed of each wheel to maintain the distance from the wall

    float current_distance = get_left_IR();

    float error = goal_distance - current_distance;

    float Kp = 0.5;
    float speed_correction = Kp * error;

    float left_speed = speed + speed_correction;
    float right_speed = speed - speed_correction;

    motor_ask(left_speed, right_speed);
}

int right_panel_controller(float speed, float goal_distance){
    // This function is used to control the distance between the robot and the wall using the IR sensor
    // It will input the speed and distance from the wall at which the robot has to travel
    // It will output the speed of each wheel to maintain the distance from the wall

    float current_distance = get_right_IR();

    float error = goal_distance - current_distance;

    float Kp = 0.5;

    float speed_correction = Kp * error;

    float left_speed = speed - speed_correction;
    float right_speed = speed + speed_correction;

    motor_ask(left_speed, right_speed);
}