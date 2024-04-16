#include "test_path_planning.hh"
#include "../Sensors/odometry.hh"
#include "../Strategy/strategy.hh"
#include "../Path_planning/Path_planning.hh"
#include "../Sensors/Micro_switch.hh"
#include "../controller/motor_ask.hh"
#include <fstream>


void test_path_planning(BigStruct* all_struct) {

    printf("enter test path planning\n");

    while (true) {
        update_position_encoders(all_struct, all_struct->odo_data);
        all_struct->strat->goal_y = 2600;
        all_struct->strat->goal_x = 1600;
        Path_planning_update(all_struct);

        if (*ctrl_c_pressed){ 
            motor_ask(0,0, all_struct);
            break;
        }

        /*double x_goal = all_struct->strat->goal_x/1000;            // Goal position [m]
        double y_goal = all_struct->strat->goal_y/1000;            // Goal position [m]
        double x_robot = all_struct->rob_pos->x;                   // Robot position [m]
        double y_robot = all_struct->rob_pos->y;                   // Robot position [m]
        double rho_goal = sqrt(pow(x_robot - x_goal, 2) + pow(y_robot - y_goal, 2));

        if (rho_goal > 0.3 && all_struct->path->delta_time > 1000) { 
            //intermediate_goal_update(all_struct); 
            all_struct->path->delta_time = 0;
        } else {
            all_struct->path->delta_time += all_struct->current_time - all_struct->path->previous_time;
            all_struct->path->previous_time = all_struct->current_time; 
        } */
    }
}