#include "picking_FSM.hh"

int init_plant_manager(Plant_Manager* plant_manager){
    int fd = spi_init_1();

    stepper_homing(fd, LEFT);
    stepper_homing(fd, RIGHT);
    plant_manager->storage.left.storage_1 = EMPTY;
    plant_manager->storage.left.storage_2 = EMPTY;
    plant_manager->storage.left.storage_3 = EMPTY;
    plant_manager->storage.right.storage_1 = EMPTY;
    plant_manager->storage.right.storage_2 = EMPTY;
    plant_manager->storage.right.storage_3 = EMPTY;
    plant_manager->storage.left.is_in_middle = 1;
    plant_manager->storage.right.is_in_middle = 0;
    plant_manager->left_state = WAITING_PLANT;
    plant_manager->right_state = WAITING_CLEAR;
    plant_manager->potting_enable = 0;
    plant_manager->type_expected = POT;
    plant_manager->fd = fd; 


    XL_320 left_Servo;
    XL_320 right_Servo;
    left_Servo.verbose = false;
    right_Servo.verbose = false;
    left_Servo.is_id(0x05);
    right_Servo.is_id(0x04);

    plant_manager->left_dyn = new DynStruct;
    plant_manager->right_dyn = new DynStruct;
    plant_manager->left_dyn->Servo = left_Servo;
    plant_manager->right_dyn->Servo = right_Servo;
    left_dyn_init(plant_manager->left_dyn);
    right_dyn_init(plant_manager->right_dyn);
    sleep(2);

    return 0;
}

int picking_FSM_left(Plant_Manager* plant_manager){

    // Variable
    State next_state;
    float depth;
    Angle angle;
    float offset_without_plant = 2.7;
    int fd = plant_manager->fd;

    switch(plant_manager->left_state){

        case WAITING_CLEAR:

            // Dynamixel rotate 
            dyn_go_to(plant_manager->left_dyn, plant_manager->right_dyn, LEFT, STANDBY, 100);

            // Stepper DOWN
            depth = travel_height(plant_manager, LEFT, PLANT1) + offset_without_plant;
            stepper_go_to(fd, LEFT, depth, 15);

            // Wait for potting mode ON
            while(1){
                if(plant_manager->potting_enable){
                    break;
                }
                usleep(5000);
            }

            // Wait for clear space 
            while(1){
                if(plant_manager->storage.right.is_in_middle == 0){
                    break;
                }
                usleep(5000);
            }

            next_state = WAITING_PLANT;

        case WAITING_PLANT:

            printf("in waiting plant state \n");

            plant_manager->storage.left.is_in_middle = 1;

            printf("bite \n");
            // Open claw
            open_claw(fd, LEFT);

            // Dynamixel ROTATE
            dyn_go_to(plant_manager->left_dyn, plant_manager->right_dyn, LEFT, PICKING, 100); 
            printf("bite \n");

            // Stepper DOWN
            stepper_go_to(fd, LEFT, 4.7, 15); 

            // Wait for potting mode ON
            while(1){
                if(plant_manager->potting_enable){
                    break;
                }
                usleep(5000);
            }

            printf("potting mode is on \n");
            printf("waiting for pot or plant \n");

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

            printf("in grab plant state \n");
            
            // Stepper DOWN
            stepper_go_to(fd, LEFT, 12.8, 15);

            // Wait for stepper to arrive
            while(1){
                if(get_stepper_position(fd, LEFT) > 12.4){break;}
            }

            // Close claw
            close_claw(fd, LEFT);

            // Sleep ???

            next_state = STORE_PLANT;



        case STORE_PLANT:

            angle = drop_slot(plant_manager, LEFT); 
            depth = travel_height(plant_manager, LEFT, angle);

            // Stepper UP
            stepper_go_to(fd, LEFT, depth, 15); 

            // Wait for stepper 
            while(1){
                if(get_stepper_position(fd, LEFT) < depth + 0.2){break;}
            }

            plant_manager->storage.left.is_in_middle = 0;

            // Dynamixel rotate
            dyn_go_to(plant_manager->left_dyn, plant_manager->right_dyn, LEFT, angle, 100);

            // Sleep ? 
            usleep(20000);

            // Open Claw
            open_claw(fd, LEFT);

            //UPDATE STRUCT
            update_fill(plant_manager, LEFT, angle);

            // Stepper UP
            depth = travel_height(plant_manager, LEFT, angle) + offset_without_plant;
            stepper_go_to(fd, LEFT, depth, 15);

            // Wait for stepper
            while(1){
                if(get_stepper_position(fd, LEFT) < depth + 0.2){break;}
            }

            if(plant_manager->storage.right.is_in_middle == 0){
                plant_manager->storage.left.is_in_middle = 1;
                next_state = WAITING_PLANT;
            }
            else{
                next_state = WAITING_CLEAR;




        default :
            printf("an error has occurred in plan manager left FSM, verfify the State the attribution\n");
    }

    plant_manager->left_state = next_state;

    return 0;
    }
}



//function to compute the slot in which the plant/pot will be dropped
Angle drop_slot(Plant_Manager *PM, side side){
    Angle DEFAULT = PICKING;

    if(side == LEFT){
        if(PM->type_expected == POT){
            //verify there are no plants in the storage
            if(PM->storage.left.storage_3 == PLANT || PM->storage.left.storage_3 == POT_AND_PLANT || PM->storage.left.storage_3 == DOUBLE_POT_AND_PLANT){
                printf("Error: left emplacement 3 contains a plant");
                return DEFAULT;
            }
            if(PM->storage.left.storage_2 == PLANT || PM->storage.left.storage_2 == POT_AND_PLANT || PM->storage.left.storage_2 == DOUBLE_POT_AND_PLANT){
                printf("Error: left emplacement 2 contains a plant");
                return DEFAULT;
            }
            if(PM->storage.left.storage_1 == PLANT || PM->storage.left.storage_1 == POT_AND_PLANT || PM->storage.left.storage_1 == DOUBLE_POT_AND_PLANT){
                printf("Error: left emplacement 1 contains a plant");
                return DEFAULT;
            }            

            if(PM->storage.left.storage_3 == EMPTY){ return PLANT3; }
            if(PM->storage.left.storage_2 == EMPTY){ return PLANT2; }
            if(PM->storage.left.storage_1 == EMPTY){ return PLANT1; }
            //double potting
            if(PM->storage.left.storage_3 == POT){ return PLANT3; }
            if(PM->storage.left.storage_2 == POT){ return PLANT2; }
            if(PM->storage.left.storage_1 == POT){ return PLANT1; }
            printf("Error: left pot storage is full");
            return DEFAULT;
        } else if (PM->type_expected == PLANT){
            if(PM->storage.left.storage_3 == POT || PM->storage.left.storage_3 == DOUBLE_POT || PM->storage.left.storage_3 == EMPTY){ return PLANT3; }
            if(PM->storage.left.storage_2 == POT || PM->storage.left.storage_2 == DOUBLE_POT || PM->storage.left.storage_2 == EMPTY){ return PLANT2; }
            if(PM->storage.left.storage_1 == POT || PM->storage.left.storage_1 == DOUBLE_POT || PM->storage.left.storage_1 == EMPTY){ return PLANT1; }
            printf("Error: left plant storage is full");
            return DEFAULT;
        } else { 
            printf("Error: wrong expected type");
            return DEFAULT;
        }

    } else {
        if(PM->type_expected == POT){
            //verify there are no plants in the storage
            if(PM->storage.right.storage_3 == PLANT || PM->storage.right.storage_3 == POT_AND_PLANT || PM->storage.right.storage_3 == DOUBLE_POT_AND_PLANT){
                printf("Error: right emplacement 3 contains a plant");
                return DEFAULT;
            }
            if(PM->storage.right.storage_2 == PLANT || PM->storage.right.storage_2 == POT_AND_PLANT || PM->storage.right.storage_2 == DOUBLE_POT_AND_PLANT){
                printf("Error: right emplacement 2 contains a plant");
                return DEFAULT;
            }
            if(PM->storage.right.storage_1 == PLANT || PM->storage.right.storage_1 == POT_AND_PLANT || PM->storage.right.storage_1 == DOUBLE_POT_AND_PLANT){
                printf("Error: right emplacement 1 contains a plant");
                return DEFAULT;
            }            

            if(PM->storage.right.storage_3 == EMPTY){ return PLANT3; }
            if(PM->storage.right.storage_2 == EMPTY){ return PLANT2; }
            if(PM->storage.right.storage_1 == EMPTY){ return PLANT1; }
            //double potting
            if(PM->storage.right.storage_3 == POT){ return PLANT3; }
            if(PM->storage.right.storage_2 == POT){ return PLANT2; }
            if(PM->storage.right.storage_1 == POT){ return PLANT1; }
            printf("Error: right pot storage is full");
            return DEFAULT;
        } else if (PM->type_expected == PLANT){
            if(PM->storage.right.storage_3 == POT || PM->storage.right.storage_3 == DOUBLE_POT || PM->storage.right.storage_3 == EMPTY){ return PLANT3; }
            if(PM->storage.right.storage_2 == POT || PM->storage.right.storage_2 == DOUBLE_POT || PM->storage.right.storage_2 == EMPTY){ return PLANT2; }
            if(PM->storage.right.storage_1 == POT || PM->storage.right.storage_1 == DOUBLE_POT || PM->storage.right.storage_1 == EMPTY){ return PLANT1; }
            printf("Error: right plant storage is full");
            return DEFAULT;
        } else {
            printf("Error: wrong expected type");
            return DEFAULT;
        }
    }
}

float travel_height(Plant_Manager *PM, side side, Angle angle){
    float empty = 4.7;
    float pot = 1.1;
    float double_pot = 0.3;
    float plant = 4.5;
    float pot_and_plant = -2.3;
    float double_pot_and_plant = 0.5;

    float DEFAULT = double_pot_and_plant;

    if(side == LEFT){
        switch (angle)
        {
        case PLANT1:
            if(PM->storage.left.storage_1 == EMPTY){ return empty; }
            if(PM->storage.left.storage_1 == POT){ return pot; }
            if(PM->storage.left.storage_1 == DOUBLE_POT){ return double_pot; }
            if(PM->storage.left.storage_1 == PLANT){ return plant; }
            if(PM->storage.left.storage_1 == POT_AND_PLANT){ return pot_and_plant; }
            if(PM->storage.left.storage_1 == DOUBLE_POT_AND_PLANT){ return double_pot_and_plant; }
            break;
        case PLANT2:
            if(PM->storage.left.storage_2 == POT){ return pot; }
            if(PM->storage.left.storage_2 == DOUBLE_POT){ return double_pot; }
            if(PM->storage.left.storage_2 == PLANT){ return plant; }
            if(PM->storage.left.storage_2 == POT_AND_PLANT){ return pot_and_plant; }
            if(PM->storage.left.storage_2 == DOUBLE_POT_AND_PLANT){ return double_pot_and_plant; }
            break;
        case PLANT3:
            if(PM->storage.left.storage_3 == POT){ return pot; }
            if(PM->storage.left.storage_3 == DOUBLE_POT){ return double_pot; }
            if(PM->storage.left.storage_3 == PLANT){ return plant; }
            if(PM->storage.left.storage_3 == POT_AND_PLANT){ return pot_and_plant; }
            if(PM->storage.left.storage_3 == DOUBLE_POT_AND_PLANT){ return double_pot_and_plant; }
            break;
        default:
            return DEFAULT;
            break;
        }
    } else {
        switch (angle)
        {
        case PLANT1:
            if(PM->storage.right.storage_1 == EMPTY){ return empty; }
            if(PM->storage.right.storage_1 == POT){ return pot; }
            if(PM->storage.right.storage_1 == DOUBLE_POT){ return double_pot; }
            if(PM->storage.right.storage_1 == PLANT){ return plant; }
            if(PM->storage.right.storage_1 == POT_AND_PLANT){ return pot_and_plant; }
            if(PM->storage.right.storage_1 == DOUBLE_POT_AND_PLANT){ return double_pot_and_plant; }
            break;
        case PLANT2:
            if(PM->storage.right.storage_2 == POT){ return pot; }
            if(PM->storage.right.storage_2 == DOUBLE_POT){ return double_pot; }
            if(PM->storage.right.storage_2 == PLANT){ return plant; }
            if(PM->storage.right.storage_2 == POT_AND_PLANT){ return pot_and_plant; }
            if(PM->storage.right.storage_2 == DOUBLE_POT_AND_PLANT){ return double_pot_and_plant; }
            break;
        case PLANT3:
            if(PM->storage.right.storage_3 == POT){ return pot; }
            if(PM->storage.right.storage_3 == DOUBLE_POT){ return double_pot; }
            if(PM->storage.right.storage_3 == PLANT){ return plant; }
            if(PM->storage.right.storage_3 == POT_AND_PLANT){ return pot_and_plant; }
            if(PM->storage.right.storage_3 == DOUBLE_POT_AND_PLANT){ return double_pot_and_plant; }
            break;
        default:
            return DEFAULT;
            break;
        }
    }
}

int update_fill(Plant_Manager *PM, side side, Angle angle){
    if(side == LEFT){
        Fill previous_fill;
        if(angle == PLANT1){    previous_fill = PM->storage.left.storage_1; }
        else if(angle == PLANT2){    previous_fill = PM->storage.left.storage_2; }
        else if(angle == PLANT3){    previous_fill = PM->storage.left.storage_3; }
        else {
            printf("Error: wrong angle");
            return -1;
        }
        Fill new_fill;
        if(PM->type_expected == POT){
            if(previous_fill == EMPTY){ new_fill = POT; }
            else if(previous_fill == POT){ new_fill = DOUBLE_POT; }
            else {
                printf("Error: wrong previous fill");
                return -1;
            }
        } else if (PM->type_expected == PLANT) {
            if(previous_fill == EMPTY){ new_fill = PLANT; }
            else if(previous_fill == POT){ new_fill = POT_AND_PLANT; }
            else if(previous_fill == DOUBLE_POT){ new_fill = DOUBLE_POT_AND_PLANT; }
            else {
                printf("Error: wrong previous fill");
                return -1;
            }
        } else {
            printf("Error: wrong expected type");
            return -1;
        }
        if(angle == PLANT1){    PM->storage.left.storage_1 = new_fill; }
        else if(angle == PLANT2){    PM->storage.left.storage_2 = new_fill; }
        else if(angle == PLANT3){    PM->storage.left.storage_3 = new_fill; }
        else {
            printf("Error: wrong angle");
            return -1;
        }
        return 0;
    } else {
        Fill previous_fill;
        if(angle == PLANT1){    previous_fill = PM->storage.right.storage_1; }
        else if(angle == PLANT2){    previous_fill = PM->storage.right.storage_2; }
        else if(angle == PLANT3){    previous_fill = PM->storage.right.storage_3; }
        else {
            printf("Error: wrong angle");
            return -1;
        }
        Fill new_fill;
        if(PM->type_expected == POT){
            if(previous_fill == EMPTY){ new_fill = POT; }
            else if(previous_fill == POT){ new_fill = DOUBLE_POT; }
            else {
                printf("Error: wrong previous fill");
                return -1;
            }
        } else if (PM->type_expected == PLANT) {
            if(previous_fill == EMPTY){ new_fill = PLANT; }
            else if(previous_fill == POT){ new_fill = POT_AND_PLANT; }
            else if(previous_fill == DOUBLE_POT){ new_fill = DOUBLE_POT_AND_PLANT; }
            else {
                printf("Error: wrong previous fill");
                return -1;
            }
        } else {
            printf("Error: wrong expected type");
            return -1;
        }
        if(angle == PLANT1){    PM->storage.right.storage_1 = new_fill; }
        else if(angle == PLANT2){    PM->storage.right.storage_2 = new_fill; }
        else if(angle == PLANT3){    PM->storage.right.storage_3 = new_fill; }
        else {
            printf("Error: wrong angle");
            return -1;
        }
        return 0;
    }
}

int main(){
    Plant_Manager* plant_manager = new Plant_Manager;
    init_plant_manager(plant_manager);
    plant_manager->potting_enable = 1;
    while(1){
        picking_FSM_left(plant_manager);
    }
    return 0;
}