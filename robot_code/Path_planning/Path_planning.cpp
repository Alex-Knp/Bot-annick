#include "Path_planning.hh"
#include "../localization/localisation.hh"
#include "../controller/motor_ask.hh"
#include "../Strategy/strategy.hh"
#include <cmath>
#include <vector>
#include "../controller/motor_ask.hh"


void Path_planning_update(BigStruct* all_struct){

    //  Global Variables 

    double k_rep;
    double k_att = 150;
    double K = 1;         // Speed factor

    double rho;
    double rho_0;

    double x_robot = all_struct->rob_pos->x;   // Robot position [m]
    double y_robot = all_struct->rob_pos->y;   // Robot position [m]

    double x_obs;           // Obstacle position [m]
    double y_obs;          

    double x_lim = 2.0;     // limite map x [m]
    double y_lim = 3.0;     // limite map y [m]

    double *F = (double *)calloc(2, sizeof(double));       // Field vector [rad/s]
    double *Frep = (double *)calloc(2, sizeof(double));    // Field vector [rad/s]

    double v_max = all_struct->path->v_max;

    double radius_robot = 0.15;  // TO FIND IMPORTANT, DISTANCE BETWEEN ORIGIN OF THE ROBOT AND THE FICTIVE CIRCLE CENTERED AT ORIGIN WHICH IS THE REPULSIVE BOARDER


    /////////--------  Attractive field  -------/////////

    // Attractive field variables

    double trigger_decel = 0.1;     // distance [m] from goal at which the robot starts to decelerate

    double x_goal = all_struct->strat->goal_x/1000;            // Goal position [m]
    double y_goal = all_struct->strat->goal_y/1000;            // Goal position [m]

    double rho_goal = sqrt(pow(x_robot - x_goal, 2) + pow(y_robot - y_goal, 2));


    // compute attractive field

    // if the robot is far from the goal (rho_goal > trigger_decel), the robot moves at maximum speed
    if (rho_goal > trigger_decel) {     
        F[0] = v_max * (x_goal - x_robot) / rho_goal * k_att;
        F[1] = v_max * (y_goal - y_robot) / rho_goal * k_att;
    }

    // if the robot is close to the goal (rho_goal <= trigger_decel), the robot decelerates
    else {
        F[0] = v_max * (x_goal - x_robot) / trigger_decel * k_att;
        F[1] = v_max * (y_goal - y_robot) / trigger_decel * k_att;
    }


    /////////--------   Repulsive Wall  -------/////////

    // attention à considérer les pannnneux qui dépenses 

    // Variables declaration

    rho_0 = 0.05;  // [m] : distance from which the repulsive field is triggered
    k_rep = 0.001; // Magnitude of the repulsive field
    

    rho = x_robot - radius_robot;
    if (rho < rho_0) {
        Frep[0] += (1/rho - 1/rho_0) / pow(rho, 3) * (x_robot) * k_rep;
    }
    rho = x_lim-x_robot - radius_robot;
    if (rho < rho_0) {
        Frep[0] += (1/rho - 1/rho_0) / pow(rho, 3) * (x_robot-x_lim) * k_rep;
    }
    rho = y_robot - radius_robot;
    if (rho < rho_0) {
        Frep[1] += (1/rho - 1/rho_0) / pow(rho, 3) * (y_robot) * k_rep;
    }
    rho = y_lim-y_robot - radius_robot;
    if (rho < rho_0) {
        Frep[1] += (1/rho - 1/rho_0) / pow(rho, 3) * (y_robot-y_lim) * k_rep;
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

    rho_0 = 0.05;  // [m] : distance from which the repulsive field is triggered
    k_rep = 0.001; // Magnitude of the repulsive field

    for (int i = 0; i < 6; i++) {

        x_obs = plants[i].x;
        y_obs = plants[i].y;

        rho = sqrt(pow(x_obs-x_robot, 2) + pow(y_obs-x_robot, 2)) - radius - radius_robot;  // Minimal distance from the obstacle

        if (rho < rho_0 && all_struct->path->active_zone[i] == 1) {
            Frep[0] += k_rep * (1/rho - 1/rho_0) / pow(rho, 3) * (x_robot-x_obs);
            Frep[1] += Frep[0] / (x_robot-x_obs) * (y_robot-y_obs);
        }
    }


    /////////--------   Opponent avoidance  -------/////////

    rho_0 = 0.05;  // [m] : distance from which the repulsive field is triggered
    k_rep = 0.001; // Magnitude of the repulsive field

    int N = 3;

    for (int i = 0; i < N; i++) {

        x_obs = all_struct->opp_pos->x[i]/1000;
        y_obs = all_struct->opp_pos->y[i]/1000;

        rho = sqrt(pow(x_obs-x_robot, 2) + pow(y_obs-y_robot, 2)) - radius_robot; 

        if (rho < rho_0) {
            Frep[0] += k_rep * (1/rho - 1/rho_0) / pow(rho, 3) * (x_robot-x_obs);
            Frep[1] += Frep[1] / (x_robot-x_obs) * (y_robot-y_obs);
        }
    }


    ////////---------    PAMI's avoidance   -------////////

    rho_0 = 0.05;  // [m] : distance from which the repulsive field is triggered
    k_rep = 0.001; // Magnitude of the repulsive field

    y_obs = 1.05;
    x_obs = 0.15;
    rho = sqrt(pow(x_obs-x_robot, 2) + pow(y_obs-y_robot, 2)) - radius_robot;
    if (rho < rho_0) {
        Frep[1] += k_rep * (1/rho - 1/rho_0) / pow(rho, 3) * (y_robot-y_obs);
        Frep[0] += k_rep * (1/rho - 1/rho_0) / pow(rho, 3) * (x_robot-x_obs);
    }
    x_obs = 0.15;
    y_obs = 1.95;
    rho = sqrt(pow(x_obs-x_robot, 2) + pow(y_obs-y_robot, 2)) - radius_robot;
    if (rho < rho_0) {
        Frep[1] += k_rep * (1/rho - 1/rho_0) / pow(rho, 3) * (y_robot-y_obs);
        Frep[0] += k_rep * (1/rho - 1/rho_0) / pow(rho, 3) * (x_robot-x_obs);
    }
    x_obs = 0.15;
    rho = x_robot - x_obs - radius_robot;
    if (rho < rho_0 && (1.05 < y_robot) < 1.95) {
        Frep[0] += k_rep * (1/rho - 1/rho_0) / pow(rho, 3) * (x_robot-x_obs);
    }


    ////////--------- Repulsive field cancellation -------////////


    if (rho_goal < 0.1) {
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
    printf("Speed ref = %f\t", all_struct->path->field->norm);
    printf("Position x = %f\t", x);
    printf("Position y = %f\t", y);
    printf("Angle goal = %f\n", all_struct->path->field->theta);
    printf("Angle theta = %f\n", all_struct->rob_pos->theta); */

    speed_regulation(all_struct,all_struct->strat->goal_x,all_struct->strat->goal_y,all_struct->path->theta,all_struct->path->norm);
    ////////---------     free      -------////////


    free(Frep);
    free(F);
}



void speed_regulation(BigStruct* all_struct, double x_goal, double y_goal,double wanted_theta, double wanted_speed) {

    float stationnary_error_angle = M_PI/4 ;          //the error angles for which the robot only turns and do not drive forward
    //double x_goal = all_struct->strat->goal_x;
    //double y_goal = all_struct->strat->goal_y;

    float current_theta = all_struct->rob_pos->theta;
    //float wanted_theta = all_struct->path->theta;
    //float wanted_speed = all_struct->path->norm;
    float turning_speed = wanted_speed / 1.5;       //[rad/s] the speed at which the robot turns


    double x = all_struct->rob_pos->x;
    double y = all_struct->rob_pos->y;

    double rho = sqrt(pow(x-x_goal/1000, 2) + pow(y-y_goal/1000, 2));  // TODO : find where to get the info


	float angle_error = wanted_theta - current_theta;

    if (angle_error > M_PI) {
        angle_error -= 2*M_PI;
    } else if (angle_error < -M_PI) {
        angle_error += 2*M_PI;
    }

	float linear_speed = 0.0;
    float rotational_speed = 0.0;


    if(abs(angle_error)>stationnary_error_angle){
        linear_speed = 0.0;
        rotational_speed = turning_speed * angle_error / abs(angle_error);
    } else {

        linear_speed = wanted_speed*cos(angle_error/stationnary_error_angle*M_PI/2.0);
        rotational_speed = turning_speed*angle_error/stationnary_error_angle;
    }

	float L_wheel_speed = linear_speed - rotational_speed;
	float R_wheel_speed = linear_speed + rotational_speed;


    if (rho < 0.02) {
        L_wheel_speed = 0;
        R_wheel_speed = 0;
        all_struct->strat->goal_reached = true;
    }
    else {
        all_struct->strat->goal_reached = false;
    }
    if (all_struct->strat->goal_reached==true){
        L_wheel_speed = 0;
        R_wheel_speed = 0;
    }

	motor_ask(L_wheel_speed, R_wheel_speed, all_struct);
}

        //all_struct->strat->goal_reached = true;

    /*else {
        all_struct->strat->goal_reached = false;
    }*/

