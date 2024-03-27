#include "strategy.hh"

void main_strategy(BigStruct* all_struct){
    //  Global Variables 
    if(all_struct->time > 80.0){
        all_struct->strat->state = GO_TO_END;
    }
    if(all_struct->time > 90.0){
        all_struct->strat->state = END_STATE;
    }

    switch(all_struct->strat->state)

    {
        case CALIB_STATE:
            if(verification_beacon(all_struct)){
                all_struct->strat->state = WAIT_INIT_STATE;
            }
            break;
        case WAIT_INIT_STATE:
            if(all_struct->team_id == TEAM_BLUE && !all_struct->startup){
                all_struct->strat->goal_x = 225.0;
                all_struct->strat->goal_y = 225.0;
                all_struct->strat->goal_theta = 90.0;// degree
            }
            else if(all_struct->team_id == TEAM_YELLOW && !all_struct->startup){
                all_struct->strat->goal_x = 225.0;
                all_struct->strat->goal_y = 2775.0;
                all_struct->strat->goal_theta = 270.0;// degree
            }
            break;
        case ON_THE_MOVE:
            // Robot is moving
            break;

        case GO_TO_POT:

        case GO_TO_PLANT:

        case GO_TO_PANNEL:

        case GO_TO_DROP:
            // Go to the pot
            break;
        case GO_TO_END:
            // Go to the end
            break;
        case TAKE_PLANT:
            // Take the plant
            break;

        case TAKE_POT:
            // Take the pot
            break;
        case SOLAR_PANNEL_STATE:

        case DROP_POT_STATE:

        case END_STATE:
            // End of the game
            break;


    }

}

bool verification_beacon(BigStruct* all_struct){
    if (all_struct->rob_pos->x == 0.0 && all_struct->rob_pos->y == 0.0 && all_struct->rob_pos->theta == 0.0){
        return false;
    }
    return true;
}