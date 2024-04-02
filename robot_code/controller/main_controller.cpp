#include "main_controller.hh"
#include "../Sensors/odometry.hh"
#include "../Strategy/strategy.hh"
#include "../Path_planning/Path_planning.hh"

void main_controller(BigStruct* all_struct){
    
    while(all_struct->strat->state != END_STATE){
        update_position_encoders(all_struct, all_struct->odo_data);
        main_strategy(all_struct);
        Path_planning_update(all_struct);
    }
}