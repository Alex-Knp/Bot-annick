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
#include <thread>
#include <vector>
#include <iostream>
#include <time.h>
#include <mutex>


#include "../sdk/include/sl_lidar_driver.h"
#include "../sdk/include/sl_lidar.h"

// Strategy States
enum{CALIB_STATE, WAIT_INIT_STATE,ON_THE_MOVE,
	GO_TO_POT,GO_TO_PLANT,GO_TO_PANNEL,GO_TO_DROP,GO_TO_END,
	TAKE_PLANT,TAKE_POT,
	TURN, MOVE,START, // Test FSM states
	SOLAR_PANNEL_STATE,DROP_POT_STATE,END_STATE};

//teams
enum {TEAM_YELLOW,TEAM_BLUE};

//side 
enum side {LEFT, RIGHT};

// Plant Manager State
enum State {WAITING_PLANT, GRAB_PLANT, STORE_PLANT, WAITING_CLEAR};

// Fill state of storage
enum Fill {EMPTY, POT, DOUBLE_POT, POT_AND_PLANT, DOUBLE_POT_AND_PLANT, PLANT};

// Storage angle
enum Angle {PICKING, PLANT1, PLANT2, PLANT3, STANDBY};

// Stepper height
enum Height {GROUND, LOW, MIDDLE, HIGH};

// microswitch
enum Micro_switch {Left_ms, Right_ms, emergency};

// forward declaration for localization
typedef struct RobotPosition RobotPosition;
typedef struct lidar_data lidar_data;
typedef struct OpponentsPosition OpponentsPosition;
typedef struct Strategy Strategy;

// forward declaration for actuators
typedef struct Storage Storage;
typedef struct Side Side;
typedef struct Plant_Manager; 


/// Main controller structure
typedef struct BigStruct
{
	// Localisation data
	RobotPosition *rob_pos;
	OpponentsPosition *opp_pos;
	Strategy *strat;
	lidar_data *table;
	sl::ILidarDriver* drv;

	int team_id;
	bool startup; // False until the startup microswitch is pressed

	time_t start_time, current_time, elapsed_time;  // start time mis à zero dans init big struct
	std::mutex time_mutex;


} BigStruct;

// function prototypes
BigStruct* init_BigStruct();
void free_BigStruct(BigStruct *calib);

