#include "Path_planning.hh"
#include <cmath>
#include <vector>


void Path_planing_update(){

//  Global Variables 

double X_robot = 0.0;   // Robot position [m]
double Y_robot = 0.0;   // Robot position [m]

double V_max  = 0.0;    // Max speed [rad/s]

double x_lim = 0.0;     // limite map x [m]
double y_lim = 0.0;     // limite map y [m]

double *F = (double *)calloc(2, sizeof(double));    // Field vector [rad/s]


/////////--------  Attractive field  -------/////////

// Attractive field variables

double trigger_decel = 0.1;     // distance [m] from goal at which the robot starts to decelerate

double X_goal = 0.0;            // Goal position [m]
double Y_goal = 0.0;            // Goal position [m]

double rho_goal = sqrt(pow(X_robot - X_goal, 2) + pow(Y_robot - Y_goal, 2));


// compute attractive field

// if the robot is far from the goal (rho_goal > trigger_decel), the robot moves at maximum speed
if (rho_goal > trigger_decel) {     
    F[0] = V_max * (X_goal - X_robot) / rho_goal;
    F[1] = V_max * (Y_goal - Y_robot) / rho_goal;
}

// if the robot is close to the goal (rho_goal <= trigger_decel), the robot decelerates
else {
    F[0] = V_max * (X_goal - X_robot) / trigger_decel;
    F[1] = V_max * (Y_goal - Y_robot) / trigger_decel;
}


/////////--------   Repulsive Wall  -------/////////

// attention à considérer les pannnneux qui dépenses 

// Variables declaration

double x_lim = 0.0;         // map limit x [m]
double y_lim = 0.0;         // map limit y [m]
double trigger_wall = 0.1;  // distance [m] from the wall impact the trajectory
double rho_0 = 0.3;
double rho;

// compute repulsive field
if (X_robot < trigger_wall) {
    rho = X_robot;
    if (rho < 0.1) {
        rho = 0.1; }
    F[0] += (1/rho - 1/rho_0) / pow(rho, 3) * (X_robot);
}
if (x_lim-X_robot < trigger_wall) {
    rho = x_lim-X_robot;
    if (rho < 0.1) {
        rho = 0.1; }
    F[0] += (1/rho - 1/rho_0) / pow(rho, 3) * (X_robot-x_lim);
}
if (Y_robot < rho_0) {
    rho = Y_robot;
    if (rho < 0.1) {
        rho = 0.1; }
    F[1] += (1/rho - 1/rho_0) / pow(rho, 3) * (Y_robot);
}
if (y_lim-Y_robot < rho_0) {
    rho = y_lim-Y_robot;
    if (rho < 0.1) {
        rho = 0.1; }
    F[1] += (1/rho - 1/rho_0) / pow(rho, 3) * (Y_robot-y_lim);
}


/////////--------   Plants avoidance  -------/////////

// Variables declaration
std::vector<Plant> plants = {
        {0.500 + 1, 0.0 + 1.5},
        {-0.500 + 1, 0.0 + 1.5},
        {0.300 + 1, 0.500 + 1.5},
        {0.300 + 1, -0.500 + 1.5},
        {-0.300 + 1, 0.500 + 1.5},
        {-0.300 + 1, -0.500 + 1.5}
    };

double trigger_plant = 0.1; // distance [m] from the plant at which the robot starts to avoid it
double slope = 100;         // slope of the repulsive field [rad/s/m]
double rho_plant;

// compute repulsive field
for (int i = 0; i < plants.size(); i++) {
    rho_plant = sqrt(pow(X_robot - plants[i].x, 2) + pow(Y_robot - plants[i].y, 2));
    if (rho < trigger_plant) {
        // linear repulsive field
        F[0] += slope * (X_robot - plants[i].x) / rho_plant;
        F[1] += slope * (Y_robot - plants[i].y) / rho_plant;
    }
}

/////////--------   Pots avoidance  -------/////////


/////////--------   Opponent avoidance  -------/////////


/////////--------   Obstacle avoidance  -------/////////



}