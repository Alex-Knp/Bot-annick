#pragma once

#include "../main/all_struct.hh"
#include <iostream>
#include <thread>
#include <mutex>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>
#include <cmath>
#include <pthread.h> 
#include <fstream>


//#include "sl_lidar.h"
//#include "sl_lidar_driver.h"

#define PI 3.14159265358979323846

using namespace sl;

ILidarDriver* connectLidar();


void scanLidar(BigStruct *all_struct);

void disconnectLidar(ILidarDriver* lidar);

struct Cluster {
    std::vector<sl_lidar_response_measurement_node_hq_t> points;
};

struct Beacon {
    float distance;
    float angle;
};

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
	double x[2]; ///< x position of opponents [m]
	double y[2]; ///< y position of opponents [m]

	int nb_opp; ///< number of opponents

} OpponentsPosition;

typedef struct lidar_data
{
	float beaconsx[3];        		// beacons's coordinates on the game bord
    float beaconsy[3];    
    float theta_lidar_calibration;  // calibration du lidar (dans notre cas c'est pi/2)

}lidar_data;

extern sl_lidar_response_measurement_node_hq_t* closest_point;


/**
* Calculate the distance between two points
* \param p1 First point which is a sl_lidar_response_measurement_node_hq_t with distance and angle as attributes
* \param p2 Second point which is a sl_lidar_response_measurement_node_hq_t with distance and angle as attributes
* \return The distance between the two points as a double
*/
double distance_btw_points(sl_lidar_response_measurement_node_hq_t p1, sl_lidar_response_measurement_node_hq_t p2);

/**
* Compute the clusters, one parameter can be adapted : the minimum distance between two points to be in the same cluster
* \param clusters Vector of clusters which is empty at the beginning
* \param nodes Array of points which is a sl_lidar_response_measurement_node_hq_t with distance and angle as attributes
* \param count Number of points in the array 
* \internal
* min_dist : minimum distance between two points to be in the same cluster (default value: 30)
*/
void getClusters(std::vector<Cluster>& clusters, sl_lidar_response_measurement_node_hq_t* nodes, size_t count);

/**
* Add beacon to the vector of beacons
* \param beacons Vector of beacons which is empty at the beginning
* \param tab Vector of clusters
*/
void getBeacon(std::vector<Beacon>& beacons, std::vector<Cluster> tab);

/**
* Evaluate if a cluster is a beacon
* \param clust Cluster which is a vector of sl_lidar_response_measurement_node_hq_t with distance and angle as attributes
* \param diam Diameter of the beacon 
* \internal
* diam_error : max error between the real diameter of the beacon and the diameter of the cluster (default value: 20)
* min_points : minimum number of points in the cluster to be a beacon
* max_points : maximum number of points in the cluster to be a beacon
* \return True if the cluster is a beacon, false otherwise
*/
bool isAbeacon(Cluster clust, double diam);

/**
* Compute the final beacons
* \param beacons Vector of beacons candidates for the final beacons
* \param final_beacons Vector of final beacons which is empty at the beginning
* \internal
* error : Max error threshold accepted when comparing distance btw beacons(default value: 50)
*/
void getFinalBeacon(std::vector<Beacon>& beacons, std::vector<Beacon>& final_beacons);
/**
* Compute the distance between two beacons
* \param beacon1 
* \param beacon2
* \return The distance between the two beacons as a double
*/
double DistBtwBeacon(Beacon beacon1, Beacon beacon2);

void lidar_update_position(RobotPosition &coord,lidar_data &data, std::vector<Beacon> &final_beacons,std::vector<double> &x_tab, std::vector<double> &y_tab, std::vector<double> &x_ref_tab, std::vector<double> &y_ref_tab);
bool comparerParAngle(const Beacon& a, const Beacon& b, float alpha);
void newsort(std::vector<Beacon>& final_beacons);

void fill_points(double x, double y, double x_ref, double y_ref, std::vector<double> &x_tab, std::vector<double> &y_tab, std::vector<double> &x_ref_tab, std::vector<double> &y_ref_tab);


void getObstacles(BigStruct* all_struct, std::vector<Cluster> tab,RobotPosition* coord);

bool isObstacle(Cluster clust,RobotPosition* coord);