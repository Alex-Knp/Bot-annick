/*
 * Forward declaration (in all_struct.hh) and new (malloc) (in all_struct.cpp) all the struct which are shared by all the codes (position,...)
 * typedef struct for all structure that are defined in other cpp = forward declaration
 * Ex : typdef  Obstacles in all_struct.hh then struct Obstacles {attribute...} in localisation.hh
 * Declaration of obstacles in the struct big_struct
 * All our .hh included in all_struc.cpp !
*/


#pragma once
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>  
#include "../sdk/include/sl_lidar_driver.h"
#include "../sdk/include/sl_lidar.h"
#include <thread>

// forward declaration for localization
typedef struct RobotPosition RobotPosition;
typedef struct lidar_data lidar_data;

// forward declaration for actuators 


/// Main controller structure
typedef struct BigStruct
{
	// Localisation data
	RobotPosition *rob_pos;
	lidar_data *table;
	sl::ILidarDriver* drv;



} BigStruct;

// function prototypes
BigStruct* init_BigStruct();
void free_BigStruct(BigStruct *calib);

