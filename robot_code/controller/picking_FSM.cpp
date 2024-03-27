#include "picking_FSM.hh"

int picking_FSM_left(Plant_Manager* plant_manager){

    // Variable
    State next_state;
    

    // TRUC A INITIALISER DANS INIT_PLANT_MANAGER
    DynStruct* left_dyn; 
    DynStruct* right_dyn;
    int fd;

    switch(plant_manager->left_state){

        case WAITNG_CLEAR:



        case WAITING_PLANT:

            // Dynamixel ROTATE
            dyn_go_to(left_dyn, right_dyn, LEFT, PICKING, 100); 

            // Stepper DOWN
            stepper_go_to(fd, LEFT, 5, 40); 

            // Wait for potting mode ON
            while(1){
                if(plant_manager->potting_enable){
                    break;
                }
                usleep(5000);
            }

            // Waiting for pot / plant
            if(plant_manager->type_expected == POT){
                wait_for_pot(fd);
            }
            else if(plant_manager->type_expected == PLANT){
                wait_for_plant(fd);
            }

            // Next state
            next_state = GRAB_PLANT;


        case GRAB_PLANT:
            


        case STORE_PLANT:

        default :
            printf("an error has occurred in plan manager left FSM, verfify the State the attribution\n");
    }

    plant_manager->left_state = next_state;
}