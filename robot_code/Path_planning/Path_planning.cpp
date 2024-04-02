#include "Path_planning.hh"
#include <cmath>
#include <vector>


void Path_planning_update(BigStruct* all_struct){

    //  Global Variables 

    double k_rep;
    double rho;
    double K = 0.1;         // Speed factor
    double x_robot = 0.0;   // Robot position [m]
    double y_robot = 0.0;   // Robot position [m]

    double x_obs;           // Obstacle position [m]
    double y_obs;

    double x_lim = 2.0;     // limite map x [m]
    double y_lim = 3.0;     // limite map y [m]

    double *F = (double *)calloc(2, sizeof(double));    // Field vector [rad/s]
    double *Frep = (double *)calloc(2, sizeof(double));    // Field vector [rad/s]

    double v_max = all_struct->path->v_max;


    /////////--------  Attractive field  -------/////////

    // Attractive field variables

    double trigger_decel = 0.1;     // distance [m] from goal at which the robot starts to decelerate

    double X_goal = all_struct->strat->goal_x;            // Goal position [m]
    double Y_goal = all_struct->strat->goal_y;            // Goal position [m]

    double rho_goal = sqrt(pow(x_robot - X_goal, 2) + pow(y_robot - Y_goal, 2));


    // compute attractive field

    // if the robot is far from the goal (rho_goal > trigger_decel), the robot moves at maximum speed
    if (rho_goal > trigger_decel) {     
        F[0] = v_max * (X_goal - x_robot) / rho_goal;
        F[1] = v_max * (Y_goal - y_robot) / rho_goal;
    }

    // if the robot is close to the goal (rho_goal <= trigger_decel), the robot decelerates
    else {
        F[0] = v_max * (X_goal - x_robot) / trigger_decel;
        F[1] = v_max * (Y_goal - y_robot) / trigger_decel;
    }


    /////////--------   Repulsive Wall  -------/////////

    // attention à considérer les pannnneux qui dépenses 

    // Variables declaration

    double trigger_wall = 0.5;  // distance [m] from the wall impact the trajectory
    double rho_0 = 0.3;
    

    if (x_robot < trigger_wall) {
        rho = x_robot;
        if (rho < 0.1) {
            rho = 0.1; }
        Frep[0] += (1/rho - 1/rho_0) / pow(rho, 3) * (x_robot);
    }
    if (x_lim-x_robot < trigger_wall) {
        rho = x_lim-x_robot;
        if (rho < 0.1) {
            rho = 0.1; }
        Frep[0] += (1/rho - 1/rho_0) / pow(rho, 3) * (x_robot-x_lim);
    }
    if (y_robot < rho_0) {
        rho = y_robot;
        if (rho < 0.1) {
            rho = 0.1; }
        Frep[1] += (1/rho - 1/rho_0) / pow(rho, 3) * (y_robot);
    }
    if (y_lim-y_robot < rho_0) {
        rho = y_lim-y_robot;
        if (rho < 0.1) {
            rho = 0.1; }
        Frep[1] += (1/rho - 1/rho_0) / pow(rho, 3) * (y_robot-y_lim);
    }


    /////////--------   Plants avoidance  -------/////////

    // Variables declaration

    double radius = 0.125;

    struct Plant {
            double x;
            double y;
        };

    std::vector<Plant> plants = {
            {0.500 + 1, 0.0 + 1.5},
            {-0.500 + 1, 0.0 + 1.5},
            {0.300 + 1, 0.500 + 1.5},
            {0.300 + 1, -0.500 + 1.5},
            {-0.300 + 1, 0.500 + 1.5},
            {-0.300 + 1, -0.500 + 1.5}
        };

    k_rep = 1;

    for (int i = 0; i < 6; i++) {

            /*if (i == 1) {
                rho_0 = 0.2;
                k_rep = 0.4; }
            else if (i == 4 or i == 5) {
                rho_0 = 0.3;
                k_rep = 1; }
            else {
                rho_0 = 0.3;
                k_rep = 1; }*/

        x_obs = plants[i].x;
        y_obs = plants[i].y;

        rho = sqrt(pow(x_obs-x_robot, 2) + pow(y_obs-x_robot, 2)) - radius;  // Minimal distance from the obstacle

        if (rho < rho_0) {
            Frep[0] += k_rep * (1/rho - 1/rho_0) / pow(rho, 3) * (x_robot-x_obs);
            Frep[1] += Frep[0] / (x_robot-x_obs) * (y_robot-y_obs);
        }
    }


    /////////--------   Pots avoidance  -------/////////


    /////////--------   Opponent avoidance  -------/////////

    k_rep = 200;
    rho_0 = 0.5;

    int N = 3;

    for (int i = 0; i < N; i++) {

        x_obs = all_struct->opp_pos->x[i];
        y_obs = all_struct->opp_pos->y[i];

        rho = sqrt(pow(x_obs-x_robot, 2) + pow(y_obs-y_robot, 2)); 

        if (rho < rho_0) {
            Frep[0] += k_rep * (1/rho - 1/rho_0) / pow(rho, 3) * (x_robot-x_obs);
            Frep[1] += Frep[1] / (x_robot-x_obs) * (y_robot-y_obs);
        }
    }



    /////////--------   Obstacle avoidance  -------/////////


    ////////---------    PAMI's avoidance   -------////////

    k_rep = 0.5;
    rho_0 = 0.3;

    if (sqrt(pow(1.05 - y_robot, 2) + pow(x_robot - 0.15, 2)) < rho_0) {
        y_obs = 1.05;
        x_obs = 0.15;
        rho = sqrt(pow(x_obs-x_robot, 2) + pow(y_obs-y_robot, 2));
        if (rho < 0.1) {
            rho = 0.1; }
        Frep[1] += k_rep * (1/rho - 1/rho_0) / pow(rho, 3) * (y_robot-y_obs);
        Frep[0] += k_rep * (1/rho - 1/rho_0) / pow(rho, 3) * (x_robot-x_obs);
    }
    else if (sqrt(pow(1.95 - y_robot, 2) + pow(x_robot - 0.15, 2)) < rho_0) {
        x_obs = 0.15;
        y_obs = 1.95;
        rho = sqrt(pow(x_obs-x_robot, 2) + pow(y_obs-y_robot, 2));
        if (rho < 0.1) {
            rho = 0.1; }
        Frep[1] += k_rep * (1/rho - 1/rho_0) / pow(rho, 3) * (y_robot-y_obs);
        Frep[0] += k_rep * (1/rho - 1/rho_0) / pow(rho, 3) * (x_robot-x_obs);
    }
    else if (x_robot - 0.15 < rho_0 && 1.05 < y_robot < 1.95) {
        rho = x_robot - 0.15;
        x_obs = 0.15;
        if (rho < 0.1) {
            rho = 0.1; }
        Frep[0] += k_rep * (1/rho - 1/rho_0) / pow(rho, 3) * (x_robot-x_obs);
    }


    ////////--------- Repulsive field cancellation -------////////


    if (rho_goal < 0.3) {
        Frep[0] = 0;
        Frep[1] = 0;
    }

    F[0] += Frep[0];
    F[1] += Frep[1];


    ////////--------- Robot's speed setting -------////////


    all_struct->path->norm = sqrt(pow(F[0] * K, 2) + pow(F[1] * K, 2));
    all_struct->path->theta = atan2(F[1], F[0]);

    if (all_struct->path->theta > 2*M_PI) { all_struct->path->theta -= 2*M_PI; }
    else if (all_struct->path->theta < 0) { all_struct->path->theta += 2*M_PI; }


    ////////--------- Speed limitation -----------////////


    if (all_struct->path->norm > v_max) {
        all_struct->path->norm = v_max;
    }


    ////////-----------    Print    -------------////////

    
/*     printf("Fx = %f\t", F[0]*K);
    printf("Fy = %f\t", F[1]*K);
    printf("Frep x = %f\t", Frep[0]*K);
    printf("Frep y = %f\n", Frep[1]*K);
    printf("Speed ref = %f\t", cvs->path->field->norm);
    printf("Position x = %f\t", x);
    printf("Position y = %f\t", y);
    printf("Angle goal = %f\n", cvs->path->field->theta);
    printf("Angle theta = %f\n", cvs->rob_pos->theta); */


    ////////---------     free      -------////////


    free(Frep);
    free(F);
}