#include "strategy.hh"

void main_strategy(BigStruct* all_struct){
    //  Global Variables 
    if(all_struct->elapsed_time > 80.0){
        all_struct->strat->state = GO_TO_END;
    }
    if(all_struct->elapsed_time > 90.0){
        all_struct->strat->state = END_STATE;
    }

    switch(all_struct->strat->state)

    {
        case CALIB_STATE:
            if ((verification_beacon(all_struct) && init_pince(all_struct)) ){
                all_struct->strat->state = WAIT_INIT_STATE;
            }
            else if(all_struct->startup){
                all_struct->strat->state = GO_TO_POT;
            }
            break;

        case WAIT_INIT_STATE:
            if(all_struct->startup){
                
                all_struct->strat->state = GO_TO_POT;
            }
            break;
        case ON_THE_MOVE:
            // Robot is moving
            if (all_struct->strat->goal_reached){
                all_struct->strat->state = all_struct->strat->next_state;
                all_struct->strat->goal_reached = false;
            }
            break;

        case GO_TO_POT:
            // Go to the pot
            pot_zone_select(all_struct, all_struct->strat->next_pot);
            if (all_struct->strat->count_pot > 5)
            {
                all_struct->strat->state = GO_TO_END;
            }
            else {
                all_struct->strat->state = ON_THE_MOVE;
                all_struct->strat->next_state = TAKE_POT;
            }
            break;

        case GO_TO_PLANT:

            plant_zone_select(all_struct, all_struct->strat->next_plant);
            if (all_struct->strat->count_plant > 5)
            {
                all_struct->strat->state = GO_TO_END;
            }
            else {
                all_struct->strat->state = ON_THE_MOVE;
                all_struct->strat->next_state = TAKE_PLANT;
            }

            break;

        case GO_TO_PANNEL:

        case GO_TO_DROP:
            drop_zone_select(all_struct,all_struct->strat->next_drop);
            all_struct->strat->state = ON_THE_MOVE;
            all_struct->strat->next_state = DROP_POT_STATE;
            break;


        case GO_TO_END: // Vérifier les coords
            if(all_struct->team_id == TEAM_BLUE){
                all_struct->strat->goal_x = 225.0;
                all_struct->strat->goal_y = 225.0;
                all_struct->strat->goal_theta = 90.0;// degree
                all_struct->strat->state = ON_THE_MOVE;
                all_struct->strat->next_state = END_STATE;
            }
            else if(all_struct->team_id == TEAM_YELLOW){
                all_struct->strat->goal_x = 225.0;
                all_struct->strat->goal_y = 225.0;
                all_struct->strat->goal_theta = 270.0;// degree
                all_struct->strat->state = ON_THE_MOVE;
                all_struct->strat->next_state = END_STATE;
            }
            break;
        case TAKE_PLANT:
            /*if(all_struct->camera->pot_detected < 3){
                go to another pot zone
            }*/
            // all_struct->grabbing_plant_enable = true;
            // if(all_struct->grabbing_plant_done){
            //     all_struct->grabbing_plant_enable = false;
            //     all_struct->strat->state = GO_TO_DROP;
            // }
            sleep(2);
            all_struct->strat->state = GO_TO_DROP;
            break;
        case TAKE_POT:
            /*if(all_struct->camera->pot_detected < 3){
                go to another pot zone
            }*/
            // all_struct->grabbing_pot_enable = true;
            // if(all_struct->grabbing_pot_done){
            //     all_struct->grabbing_pot_enable = false;
            //     all_struct->strat->state = GO_TO_PLANT;
            // }
            all_struct->strat->state = GO_TO_PLANT;
            sleep(2);
            break;
        case SOLAR_PANNEL_STATE:

        case DROP_POT_STATE:
            // all_struct->dropping_enable = true;
            // if(all_struct->grabbing_plant_done){
            //     all_struct->grabbing_plant_enable = false;
            //     all_struct->strat->state = GO_TO_POT;
            // }
            sleep(2);

            all_struct->strat->state = GO_TO_POT;

        case END_STATE:
            // shutdown actuators
            break;

    }

}

bool verification_beacon(BigStruct* all_struct){
    if(all_struct->beacon_ok){
        return true;
    }
    return false;
}

bool init_pince(BigStruct* all_struct){
    if (all_struct->pince_ok){
        return true;
    }
    return false;
}

void pot_zone_select(BigStruct* all_struct, int num_pot){

    if(num_pot == 0){
        all_struct->strat->goal_x = 612.5;
        all_struct->strat->goal_y= 285; 
    }
    else if(num_pot == 1){
        all_struct->strat->goal_x = 1387.5;
        all_struct->strat->goal_y = 285;
    }
    else if(num_pot == 2){
        all_struct->strat->goal_x = 1387.5;
        all_struct->strat->goal_y = 2715;
    }
    else if(num_pot == 3){
        all_struct->strat->goal_x = 612.5;
        all_struct->strat->goal_y = 2715;
    }
    else if(num_pot == 4){
        all_struct->strat->goal_x = 285;
        all_struct->strat->goal_y = 2000;
    }
    else if(num_pot == 5){
        all_struct->strat->goal_x = 285;
        all_struct->strat->goal_y = 1000;
    }

    all_struct->strat->count_pot++;
    printf("befor countpot\n");
    all_struct->strat->next_pot = all_struct->pot_list[all_struct->strat->count_pot];
    printf("after countpot\n");
} 

void plant_zone_select(BigStruct* all_struct, int num_plant){

    if(num_plant == 0){
        all_struct->strat->goal_x = 700;
        all_struct->strat->goal_y= 1000;
    }
    else if(num_plant == 1){
        all_struct->strat->goal_x = 1300;
        all_struct->strat->goal_y = 1000;
    }
    else if(num_plant == 2){
        all_struct->strat->goal_x = 1500;
        all_struct->strat->goal_y = 1500;
    }
    else if(num_plant == 3){
        all_struct->strat->goal_x = 1300;
        all_struct->strat->goal_y = 2000;
    }
    else if(num_plant == 4){
        all_struct->strat->goal_x = 700;
        all_struct->strat->goal_y = 2000;
    }
    else if(num_plant == 5){
        all_struct->strat->goal_x = 500;
        all_struct->strat->goal_y = 1500;
    }

    all_struct->strat->count_plant++;
    all_struct->strat->next_plant = all_struct->plant_list[all_struct->strat->count_plant];          
} 

void drop_zone_select(BigStruct* all_struct,int num_drop_zone){
    if(num_drop_zone == 0){
        all_struct->strat->goal_x = 612.5;
        all_struct->strat->goal_y = 450;
    }
    else if(num_drop_zone == 1){
        all_struct->strat->goal_x = 1387.5;
        all_struct->strat->goal_y = 450;
    }
    else if(num_drop_zone == 2){
        all_struct->strat->goal_x = 1550;
        all_struct->strat->goal_y = 762.5;
    }
    else if(num_drop_zone == 3){
        all_struct->strat->goal_x = 1800;
        all_struct->strat->goal_y = 2237.5;
    }
    else if(num_drop_zone == 4){
        all_struct->strat->goal_x = 1387.5;
        all_struct->strat->goal_y = 2550;
    }
    else if(num_drop_zone == 5){
        all_struct->strat->goal_x = 612.5;
        all_struct->strat->goal_y = 2550;
    }
    all_struct->strat->count_drop++;
    all_struct->strat->count_drop = all_struct->strat->count_drop % 3;
    all_struct->strat->next_drop = all_struct->drop_zone_list[all_struct->strat->count_drop];
}