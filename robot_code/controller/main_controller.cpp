#include "main_controller.hh"
#include "../Sensors/odometry.hh"

void main_controller(BigStruct* all_struct){
    while(all_struct->strat->state != END_STATE){
        update_position_encoders(all_struct);
        main_strategy(all_struct);
        Path_planning_update(all_struct);

    }
}