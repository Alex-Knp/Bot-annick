// Filename: robot_controller.h
#pragma once

#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include <math.h> 
#include <stdio.h>
#include <time.h>


#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <thread>


/* #ifndef MOTOR_ASK_H
#define MOTOR_ASK_H
    extern "C" {
        #include "../../Our_code_C/motor_ask.h"
    }
#endif

#ifndef SPI_H
#define SPI_H
    extern "C" {
        #include "../../Our_code_C/SPI.h"
    }
#endif */

#ifndef MOTOR_ASK
#define MOTOR_ASK
    #include "motor_ask.h"
#endif

#ifndef SPI
#define SPI
    #include "SPI.h"
#endif

#include "lidar.h"

#define PI 3.14159265358979323846

/*
// To delete
struct coordinates
{
    float x;
    float y;
    float theta;
    int   update_by;        // 0 if updated by odometers, 1 if by LIDAR
};
*/


struct odometer_data {
    int32_t left_prev;
    int32_t right_prev;

    uint8_t odo_left[5];
    uint8_t odo_right[5];
    
    int32_t right_tics;
    int32_t left_tics;
};


struct encoder_data {
    uint8_t enc_left[5];
    uint8_t enc_right[5];

    float L_speed;
    float R_speed;
};


struct wheels_speed {
    float left_wheel_speed;
    float right_wheel_speed;
};


// Function prototypes
/**
 * This function updates the position using the odometers
 * \param prev_known_coord: the coordinates of the robot
 * \param odo_data: the number of tics counted by the odometers
*/
void update_position_encoders(struct coordinates *prev_known_coord, struct odometer_data *odo_data);


/**
 * This function gets via SPI the number of tics counted by each odometer
 * \param encoder_data: the structure containing the odometers informations to update
*/
void update_encoders(struct encoder_data *encoder_data);


/**
 * This function calculates both wheel speeds needed to go from where the robot is to its goal position
 * \param wheels_speed: the structure containing the wheel speeds to update
 * \param curr_pos: the current position of the robot
 * \param goal_pos: the goal position of the robot
*/
int position_controller(struct wheels_speed *wheels_speed,struct coordinates *curr_pos, struct coordinates *goal_pos);


/**
*\param omega_in: actual speed of the motor, measured by the motor encoder
*\param omega_ref: motor speed instruction we give to the controller
*\param integral_term: integral of the previous error
*\param previous_time: time of the last controller calculution
*
*\return output_voltage: average motor voltage
*/
float PI_motor_controller(float omega_in, float omega_ref, float* integral_term, time_t previous_time);


int robot_controller();

#endif // ROBOT_CONTROLLER_H