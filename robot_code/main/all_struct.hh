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
enum State {WAITING_PLANT, GRAB_PLANT, STORE_PLANT, WAITING_CLEAR, CHECKING_INSTRUCTION, DEPOT_PLANT};

// Fill state of storage
enum Fill {EMPTY, POT, DOUBLE_POT, POT_AND_PLANT, DOUBLE_POT_AND_PLANT, PLANT};

// Storage angle
enum Angle {PICKING, PLANT1, PLANT2, PLANT3, STANDBY, DEPOT_1, DEPOT_2, DEPOT_3};

// Stepper height
enum Height {GROUND, LOW, MIDDLE, HIGH};

// microswitch
enum Micro_switch {Left_ms, Right_ms, emergency};

// forward declaration for localization
typedef struct lidar_data lidar_data;

// forward declaration for actuators
typedef struct Storage Storage;
typedef struct Side Side;
typedef struct odometer_data;
typedef struct Plant_Manager Plant_Manager; 


typedef struct Path_planning Path_planning;


typedef struct RobotPosition
{
    float x;
    float y;
    float theta;
    int   update_by;        // 0 if updated by odometers, 1 if by LIDAR
}RobotPosition;
//extern pthread_mutex_t mutex;

typedef struct OpponentsPosition
{
	double* x; ///< x position of opponents [m]
	double* y; ///< y position of opponents [m]

	int nb_opp; ///< number of opponents

} OpponentsPosition;

typedef struct Strategy
{
    int state;
    int next_state;
    double goal_x;
    double goal_y;
    double goal_theta;
    bool goal_reached;
    int next_pot;
	int drop_zone;
    int count_pot;
	int count_plant;

} Strategy;
/// Main controller structure
typedef struct BigStruct
{
	// Localisation data
	RobotPosition *rob_pos;
	OpponentsPosition *opp_pos;
	Strategy *strat;
	lidar_data *table;
	sl::ILidarDriver* drv;
	Path_planning *path;

	int team_id;
	bool startup; // False until the startup microswitch is pressed

	time_t start_time, current_time, elapsed_time;  // start time mis à zero dans init big struct
	std::mutex time_mutex;

	int* pot_list;
	int* plant_list;
	
	// Communication data
	int fd1;   // correspond au spi_init_1
	int fd0;   // correspond au spi_init_0
	odometer_data *odo_data;



	// Variables communicants avec la FSM de la pince
	// Enable : FSM Strat vers FSM Pince
	// Done : FSM Pince vers FSM Strat
	bool grabbing_pot_enable;
	bool grabbing_plant_enable;
	bool grabbing_pot_done;
	bool grabbing_plant_done;
	bool dropping_enable;
	bool dropping_done;


} BigStruct;

// function prototypes
BigStruct* init_BigStruct();
void free_BigStruct(BigStruct *calib);

