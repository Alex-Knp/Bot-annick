#pragma once

#include "all_struct.hh"



typedef struct RobotPosition
{
	double x; ///< x position [m]
	double y; ///< y position [m]
	double theta; ///< robot orientation [rad]

	double last_t; ///< last time odometry was updated

} RobotPosition;

typedef struct table_data
{
	float beaconsx[3];        		// beacons's coordinates on the game bord
    float beaconsy[3];    
    float theta_lidar_calibration;  // calibration du lidar (dans notre cas c'est pi/2)

}table_data;

