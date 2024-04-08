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
        //printf(" startup = %d\n",all_struct->startup);
/*         printf(" State = %d\n", all_struct->strat->state);
        printf(" Goal_X = %f\n", all_struct->strat->goal_x);
        printf(" Goal_Y = %f\n", all_struct->strat->goal_y);
        printf("\n");
        printf("Position : x = %f\t y = %f\n", all_struct->rob_pos->x, all_struct->rob_pos->y);
        printf("\n");  */

        update_position_encoders(all_struct, all_struct->odo_data);
        main_strategy(all_struct);

        // Bypass Drop pot
        if (all_struct->strat->state == DROP_POT_STATE){
            if(all_struct->dropping_done){
                back_up(all_struct);
            }
            else if(get_MS(all_struct->fd1, Right_ms) && get_MS(all_struct->fd1, Left_ms)){
                motor_ask(0,0, all_struct);
                all_struct->dropping_done = true;//Temporary
            }
            else{
                calib_drop_zone(all_struct);
            }
        } 
        // Bypass Plant grab
        else if (all_struct->strat->state == TAKE_PLANT){
            double angle;
            if(all_struct->strat->first_time_plant){
                angle = atan2(all_struct->strat->goal_y - all_struct->rob_pos->y, all_struct->strat->goal_x - all_struct->rob_pos->x);
                if (angle < 0){
                    angle += 2*M_PI;
                }
                first_time_in = 0;
            }
            calib_plant_zone(all_struct,angle);

        }
        else if (all_struct->strat->state != WAIT_INIT_STATE && all_struct->strat->state != CALIB_STATE){
            Path_planning_update(all_struct);
        }
        if (*ctrl_c_pressed){ 
            motor_ask(0,0, all_struct);
            break;
        }
    }
}