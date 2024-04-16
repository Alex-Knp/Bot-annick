#include "main_controller.hh"
#include "../Sensors/odometry.hh"
#include "../Strategy/strategy.hh"
#include "../Path_planning/Path_planning.hh"
#include "../Sensors/Micro_switch.hh"
#include "../controller/motor_ask.hh"
#include <fstream>


void main_controller(BigStruct* all_struct){


    while(all_struct->strat->state != END_STATE){

        all_struct->startup = !get_MS(all_struct->fd1, Left_ms);

        printf(" startup = %d\n",all_struct->startup);
        printf(" State = %d\n", all_struct->strat->state);
        printf(" Goal_X = %f\n", all_struct->strat->goal_x);
        printf(" Goal_Y = %f\n", all_struct->strat->goal_y);
        printf("\n");
        printf("Position : x = %f\t y = %f,theta = %f\n", all_struct->rob_pos->x, all_struct->rob_pos->y,all_struct->rob_pos->theta*180/M_PI);
        printf("\n"); 

        update_position_encoders(all_struct, all_struct->odo_data);

        main_strategy(all_struct);


        // Bypass Drop pot
        if (all_struct->strat->state == DROP_POT_STATE){
            if(opponent_detection(all_struct)){
                motor_ask(0.0,0.0, all_struct);
            }
            else if(all_struct->dropping_done){
                back_up(all_struct);
            }
            else if(get_MS(all_struct->fd1, Right_ms) && get_MS(all_struct->fd1, Left_ms)){
                motor_ask(0,0, all_struct);
                all_struct->dropping_done = true;//Temporary
            }
            else {
                calib_drop_zone(all_struct);
            }
        } 
        // Bypass plant V2 ( merci pas de cou tu khalass)
        else if (all_struct->strat->state == TAKE_PLANT){
            if(opponent_detection(all_struct)){
                motor_ask(0.0,0.0, all_struct);
                printf("stopped\n");
            }
            else{
                calib_plant_zone(all_struct);
            }
        }
        //Bypass Solar pannel
        else if(all_struct->strat->state == SOLAR_PANNEL_STATE){
            if(opponent_detection(all_struct)){
                motor_ask(0.0,0.0, all_struct);
                printf("stopped\n");
            }
            else{
                printf("calib\n");
                calib_solar_pannel(all_struct);
            }
        }
        // path planning
        else if (all_struct->strat->state != WAIT_INIT_STATE && all_struct->strat->state != CALIB_STATE){
            printf("enter PP\n");
            Path_planning_update(all_struct);

            double x_goal = all_struct->strat->goal_x/1000;            // Goal position [m]
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
            }
        }
        if (*ctrl_c_pressed){ 
            motor_ask(0,0, all_struct);
            break;
        }
    }
}