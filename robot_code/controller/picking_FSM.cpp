#include "picking_FSM.hh"

// command de fils de pute

/*
g++ -o picking picking_FSM.cpp ../main/all_struct.cpp ../communication/SPI_spidev.cpp ../actuators/stepper/Stepper.cpp ../actuators/dynamixels/dynamixel_main.cpp ..//actuators/dynamixels/XL_320_library/XL_320.cpp ../Sensors/IR.cpp ../actuators/Servo/Servo.cpp ../Sensors/Micro_Switch.cpp -lgpiod
*/

int picking_FSM(Plant_Manager* plant_manager, side* Side) {
    // Variable initialization
    State next_state;
    State state; 
    float depth;
    Angle angle;
    Angle take_angle;
    Angle depot_angle;
    float offset_without_plant = 2.9;
    int fd = plant_manager->fd;

    if(*Side == LEFT){
        state = plant_manager->left_state; 
    }
    else{state = plant_manager->right_state;}

    switch(state) {

        //------------------- CHECKING FOR ENABLE ---------------//
        case CHECKING_INSTRUCTION: 

            printf("---------- %d IN CHECKING STATE -------------\n", *Side); 
            // close claw 
            //close_claw(fd, Side); 

            if (plant_manager->potting_enable == 0) {
                // Stepper UP
                //stepper_go_to(fd, Side, 1.5, 60);
                usleep(100000);

                next_state = CHECKING_INSTRUCTION;
                break;
            }

            else if(plant_manager->potting_enable == 1) {

                if(is_full(plant_manager, RIGHT) == 1 && is_full(plant_manager, LEFT)){
                plant_manager->type_expected = PLANT; 
                printf("ALLLLLLL FULLLLLLLLLL\n"); 
                next_state = CHECKING_INSTRUCTION; 
                break; 
                }

                else if(is_full(plant_manager, *Side)){
                next_state = CHECKING_INSTRUCTION; 
                usleep(100000);
                break; 
                }
                
                else if(*Side == LEFT && plant_manager->storage.right.is_in_middle == 0){
                    next_state == WAITING_PLANT; 
                }

                else if(*Side == RIGHT && plant_manager->storage.left.is_in_middle == 0){
                    next_state == WAITING_PLANT; 
                }

                else {
                    next_state = WAITING_CLEAR;
                    usleep(50000);
                    break; 
                }
            } else if (plant_manager->potting_enable == 2) {

                next_state = UNSTORE_PLANT;
                break;

            } else {
                printf("Error: wrong potting enable value\n");
                next_state = CHECKING_INSTRUCTION;
                break;
            }

        //-------------------- WAITING FOR MIDDLE TO BE FREE -----------//
        case WAITING_CLEAR:

            printf("---------- %d in waiting clear state -----------\n", *Side);
            // close claw 
            close_claw(fd, *Side);
            // Dynamixel rotate 
            dyn_go_to(plant_manager->dynamixels, *Side, STANDBY, 100);
            // Stepper DOWN
            depth = travel_height(plant_manager, *Side, PLANT1) + offset_without_plant;
            stepper_go_to(fd, *Side, depth, 60);

            // Wait for clear space 
            if(*Side == LEFT){
                while(1) {
                    if (plant_manager->storage.right.is_in_middle == 0) {
                        plant_manager->storage.left.is_in_middle = 1;
                        break;
                    }
                    usleep(5000);
                }
            }
            else{
                while(1) {
                    if (plant_manager->storage.left.is_in_middle == 0) {
                        plant_manager->storage.right.is_in_middle = 1;
                        break;
                    }
                    usleep(5000);
                }
            }

            if(is_full(plant_manager, *Side) == 1){
                next_state = CHECKING_INSTRUCTION; 
                break; 

            }
            
            next_state = WAITING_PLANT;
            break;
            
        //-------------------- WAITING FOR PLANT TO BE IN -----------//
        case WAITING_PLANT:

            printf("---------- %d in waiting plant state ----------\n", *Side);
            close_claw(fd, *Side); 
            // Dynamixel ROTATE
            dyn_go_to(plant_manager->dynamixels, *Side, PICKING, 100);

            // Stepper DOWN
            stepper_go_to(fd, *Side, 12.8, 60);
            while(1) {
                if(get_stepper_position(fd, Side) > 5.0){
                    open_claw(fd, *Side); 
                    break;
                }
            }
            while(1) {
                if(get_stepper_position(fd, Side) > 10.0){
                    open_large_claw(fd, *Side); 
                    break;
                }
            }
            // Waiting for pot / plant
            if (plant_manager->type_expected == POT) {
                wait_for_pot(fd);
            } else if (plant_manager->type_expected == PLANT) {
                wait_for_plant(fd);
            }

            // Next state

            if(is_full(plant_manager, *Side) == 1){
                next_state = CHECKING_INSTRUCTION; 
                break; 

            }

            next_state = GRAB_PLANT;
            break; 

        //-------------------- GRAB THE PLANT -----------//
        case GRAB_PLANT:

            printf("----------- %d in grab plant state ----------\n", *Side);
            // Wait for stepper to arrive
            while(1) {
                if (get_stepper_position(fd, Side) > 12.75) {
                    break;
                }
            }
            if(If_double_pot(fd) && plant_manager->type_expected == POT){
                next_state = STORE_DOUBLE_POT;
                break;
            }
            else{
                // Close claw
                close_claw(fd, *Side);
                next_state = STORE_PLANT;
                break;
            }

        //-------------------- STORE PLANT IN FREE STORAGE-----------//
        case STORE_PLANT:

            printf("---------- %d In store plant state ----------\n", *Side);
            angle = drop_slot(plant_manager, *Side); 
            depth = travel_height(plant_manager, *Side, angle);


            // Stepper UP
            stepper_go_to(fd, *Side, depth, 60);


            // Wait for stepper 
            while(1) {
                printf("Side = %d", *Side);
                printf("get stepper pos = %f", get_stepper_position(fd, Side)); 
                if (get_stepper_position(fd, Side) < depth + 0.05) {
                    break;
                }
            }

            if(*Side == LEFT){
                plant_manager->storage.left.is_in_middle = 0;
            }
            else{
                plant_manager->storage.right.is_in_middle = 0;
            }
            
            // Dynamixel rotate
            dyn_go_to(plant_manager->dynamixels, *Side, angle, 100);
            
            //stepper LOW
            stepper_go_to(fd, *Side, depth + 1, 60); 

            while(1) {
                if (get_stepper_position(fd, Side) > depth + 1 - 0.05) {
                    break;
                }
            }

            // Open Claw
            open_claw(fd, *Side);
            usleep(300000); 
            //UPDATE STRUCT
            update_fill(plant_manager, *Side, angle);
            // Stepper UP
            depth = travel_height(plant_manager, *Side, angle) + offset_without_plant;
            stepper_go_to(fd, *Side, depth, 60);
            // Wait for stepper
            while(1) {
                if (get_stepper_position(fd, Side) < depth + 0.05) {
                    close_claw(fd, *Side);
                    break;
                }
            }

            if(*Side == LEFT){
                if (plant_manager->storage.right.is_in_middle == 0 && plant_manager->potting_enable == 1) {
                    plant_manager->storage.left.is_in_middle = 1;
                    next_state = WAITING_PLANT;
                } 
                else if(plant_manager->potting_enable == 1){
                    next_state = WAITING_CLEAR; 
                }
                else {
                    next_state = CHECKING_INSTRUCTION;
                }
            }
            else{
                if (plant_manager->storage.left.is_in_middle == 0 && plant_manager->potting_enable == 1) {
                    plant_manager->storage.right.is_in_middle = 1;
                    next_state = WAITING_PLANT;
                }  
                else if(plant_manager->potting_enable == 1){
                    next_state = WAITING_CLEAR; 
                }else {
                    next_state = CHECKING_INSTRUCTION;
                }
            }
            break;


        //-------------------- STORE DOUBLE POT IN FREE STORAGE-----------//
        case STORE_DOUBLE_POT :

            printf("---------- In store plant state ----------\n");
            angle = drop_slot(plant_manager, *Side); 
            depth = 3;

            // Stepper UP
            stepper_go_to(fd, *Side, depth, 60);

            //catch double pot
            while(1) {
                if(get_stepper_position(fd, Side) < 10.7){
                    close_claw(fd, *Side); 
                    break; 
                }
            }

            // Wait for stepper 
            while(1) {
                if (get_stepper_position(fd, Side) < depth + 0.05) {
                    break;
                }
            }

            if(*Side == LEFT){
                plant_manager->storage.left.is_in_middle = 0;
            }
            else{
                plant_manager->storage.right.is_in_middle = 0;
            }
            
            // Dynamixel rotate
            dyn_go_to(plant_manager->dynamixels, *Side, angle, 100);

            //stepper LOW
            stepper_go_to(fd, *Side, depth+1, 60); 

            while(1) {
                if (get_stepper_position(fd, Side) > depth + 1 - 0.05) {
                    break;
                }
            }

            // Open Claw
            open_claw(fd, *Side);
            usleep(300000); 

            //UPDATE STRUCT
            update_fill(plant_manager, *Side, angle);

            // Stepper UP
            depth = travel_height(plant_manager, *Side, angle) + offset_without_plant;
            stepper_go_to(fd, *Side, depth, 60);
            // Wait for stepper
            while(1) {
                if (get_stepper_position(fd, Side) < depth + 0.05) {
                    close_claw(fd, *Side);
                    break;
                }
            }

            if(*Side == LEFT){
                if (plant_manager->storage.right.is_in_middle == 0 && plant_manager->potting_enable == 1) {
                    plant_manager->storage.left.is_in_middle = 1;
                    next_state = WAITING_PLANT;
                } 
                else if(plant_manager->potting_enable == 1){
                    next_state = WAITING_CLEAR; 
                }
                else {
                    next_state = CHECKING_INSTRUCTION;
                }
            }
            else{
                if (plant_manager->storage.left.is_in_middle == 0 && plant_manager->potting_enable == 1) {
                    plant_manager->storage.right.is_in_middle = 1;
                    next_state = WAITING_PLANT;
                }  
                else if(plant_manager->potting_enable == 1){
                    next_state = WAITING_CLEAR; 
                }else {
                    next_state = CHECKING_INSTRUCTION;
                }
            }
            break;


        //-------------------- UNSTORE PLANT -----------//
        case UNSTORE_PLANT:

            printf("---------- unstore plant state ----------\n");
            take_angle = depot_slot(plant_manager, *Side);
            depot_angle = plant_manager->depot_angle;

            // dyn go to plant to take
            dyn_go_to(plant_manager->dynamixels, *Side, take_angle, 100);

            // checking if all sides empty 
            if (take_angle == STANDBY) {
                printf("Error: no plant to depot\n");
                next_state = CHECKING_INSTRUCTION;
                plant_manager->depot_angle = DEPOT_1;
                plant_manager->potting_enable = 0;
                dyn_go_to(plant_manager->dynamixels, *Side, STANDBY, 100); 
                break;
            }
            // Open claw
            open_claw(fd, *Side);

            // Stepper DOWN to taking pot
            depth = grab_height(plant_manager, *Side, take_angle);
            stepper_go_to(fd, *Side, depth, 15); 

            //checking when stepper arrived
            while(1) {
                if (get_stepper_position(fd, Side) > depth - 0.05) {
                    break;
                }
            }

            // Close claw
            close_claw(fd, *Side);
            usleep(10000);

            // Update fill pot
            update_unfill(plant_manager, *Side, take_angle);

            // Stepper UP
            depth = travel_height(plant_manager, *Side, PLANT1); 
            stepper_go_to(fd, *Side, depth-2, 20);

            // Checking if stepper arrived
            while(1) {
                if (get_stepper_position(fd, Side) < depth + 0.05 - 2) {
                    break;
                }
            }

            if(*Side == plant_manager->depot_zone && depot_angle != DEPOT_1){
                next_state = DEPOT_PLANT_SIDE;
            }
            else{next_state = DEPOT_PLANT_FRONT;}
            break; 
            

        case DEPOT_PLANT_FRONT : 
            // dyn go to front
            dyn_go_to(plant_manager->dynamixels, *Side, FRONT, 100);
            
            // Extend Claw 
            extend_pento(fd, *Side);
            usleep(200000); 

            printf("DEPOT ANGLE = %d \n", plant_manager->depot_angle); 
            // trun to depot angle
            dyn_go_to(plant_manager->dynamixels, *Side, depot_angle, 100);

            // Check front Micro_Switches
            while(1){
                if(get_MS(fd, Right_ms) == 1 && get_MS(fd, Left_ms) == 1){
                    break;
                }
            }
            usleep(100000);

            // Open claw
            open_claw(fd, *Side);
            sleep(1);

            // dyn go to front
            dyn_go_to(plant_manager->dynamixels, *Side, FRONT, 100); 

            // Retract pento
            retract_pento(fd, *Side);
            sleep(3);

            // Stepper UP
            stepper_go_to(fd, *Side, 1, 20);

            // Checking if stepper arrived
            while(1) {
                if (get_stepper_position(fd, Side) < 1.5 + 0.05) {
                    break;
                }
            }

            if(plant_manager->depot_zone == *Side){
                plant_manager->depot_angle = DEPOT_SIDE_1; 
            }
            else{plant_manager->depot_angle = (Angle) (depot_angle + 1);} 

            next_state = UNSTORE_PLANT;
            break;
        

        case DEPOT_PLANT_SIDE :

            // dyn go to front
            dyn_go_to(plant_manager->dynamixels, *Side, plant_manager->depot_angle, 100);
            
            // Extend Claw 
            extend_pento(fd, *Side);
            usleep(200000); 

            // Check front Micro_Switches
            while(1){
                if(get_MS(fd, Right_ms) == 1 && get_MS(fd, Left_ms) == 1){
                    break;
                }
            }
            usleep(100000);

            // Open claw
            open_claw(fd, *Side);
            sleep(1);

            // Retract pento
            retract_pento(fd, *Side);
            sleep(3);

            // Stepper UP
            stepper_go_to(fd, *Side, 1, 20);

            // Checking if stepper arrived
            while(1) {
                if (get_stepper_position(fd, Side) < 1.5 + 0.05) {
                    break;
                }
            }

            plant_manager->depot_angle = (Angle) (depot_angle + 1); 
            next_state = UNSTORE_PLANT;
            break;

    }


    if(*Side == LEFT){
        plant_manager->left_state = next_state;
    }
    else {plant_manager->right_state = next_state; }

    return 0;
}



int init_plant_manager(Plant_Manager* plant_manager){
    
    int fd = spi_init_1();

    open_claw(fd, LEFT);
    open_claw(fd, RIGHT); 
    retract_pento(fd, LEFT); 
    retract_pento(fd, RIGHT);
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
    plant_manager->depot_angle = DEPOT_1;
    plant_manager->depot_zone = LEFT; 

    return 0;
}

int close_plant_manager(Plant_Manager* plant_manager){
    free(plant_manager->dynamixels); 
    free(plant_manager); 
    return 1; 
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
    float pot = 0.5;
    float double_pot = 0.2;
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
            if(PM->storage.left.storage_2 == EMPTY){ return empty; }
            if(PM->storage.left.storage_2 == POT){ return pot; }
            if(PM->storage.left.storage_2 == DOUBLE_POT){ return double_pot; }
            if(PM->storage.left.storage_2 == PLANT){ return plant; }
            if(PM->storage.left.storage_2 == POT_AND_PLANT){ return pot_and_plant; }
            if(PM->storage.left.storage_2 == DOUBLE_POT_AND_PLANT){ return double_pot_and_plant; }
            break;
        case PLANT3:
            if(PM->storage.left.storage_3 == EMPTY){ return empty; }
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
            if(PM->storage.right.storage_2 == EMPTY){ return empty; }
            if(PM->storage.right.storage_2 == POT){ return pot; }
            if(PM->storage.right.storage_2 == DOUBLE_POT){ return double_pot; }
            if(PM->storage.right.storage_2 == PLANT){ return plant; }
            if(PM->storage.right.storage_2 == POT_AND_PLANT){ return pot_and_plant; }
            if(PM->storage.right.storage_2 == DOUBLE_POT_AND_PLANT){ return double_pot_and_plant; }
            break;
        case PLANT3:
            if(PM->storage.right.storage_3 == EMPTY){ return empty; }   
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
    return DEFAULT;
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

int update_unfill(Plant_Manager *PM, side side, Angle angle){
    if(side == LEFT){
        if(angle == PLANT1){
            if(PM->storage.left.storage_1 == POT_AND_PLANT){PM->storage.left.storage_1 = EMPTY;}
            else if(PM->storage.left.storage_1 == DOUBLE_POT_AND_PLANT){PM->storage.left.storage_1 = POT;}
            else{printf("error, wrong argument for update_unfill function");}
            return 1; 
        }
        if(angle == PLANT2){
            if(PM->storage.left.storage_2 == POT_AND_PLANT){PM->storage.left.storage_2 = EMPTY;}
            else if(PM->storage.left.storage_2 == DOUBLE_POT_AND_PLANT){PM->storage.left.storage_2 = POT;}
            else{printf("error, wrong argument for update_unfill function");}
            return 1; 
        }
        if(angle == PLANT3){
            if(PM->storage.left.storage_3 == POT_AND_PLANT){PM->storage.left.storage_3 = EMPTY;}
            else if(PM->storage.left.storage_3 == DOUBLE_POT_AND_PLANT){PM->storage.left.storage_3 = POT;}
            else{printf("error, wrong argument for update_unfill function");}
            return 1; 
        }
        printf("error, wrong argument for update_unfill function");
        return 1;
    }
    if(side == RIGHT){
        if(angle == PLANT1){
            if(PM->storage.right.storage_1 == POT_AND_PLANT){PM->storage.right.storage_1 = EMPTY;}
            else if(PM->storage.right.storage_1 == DOUBLE_POT_AND_PLANT){PM->storage.right.storage_1 = POT;}
            else{printf("error, wrong argument for update_unfill function");}
            return 1; 
        }
        if(angle == PLANT2){
            if(PM->storage.right.storage_2 == POT_AND_PLANT){PM->storage.right.storage_2 = EMPTY;}
            else if(PM->storage.right.storage_2 == DOUBLE_POT_AND_PLANT){PM->storage.right.storage_2 = POT;}
            else{printf("error, wrong argument for update_unfill function");}
            return 1; 
        }
        if(angle == PLANT3){
            if(PM->storage.right.storage_3 == POT_AND_PLANT){PM->storage.right.storage_3 = EMPTY;}
            else if(PM->storage.right.storage_3 == DOUBLE_POT_AND_PLANT){PM->storage.right.storage_3 = POT;}
            else{printf("error, wrong argument for update_unfill function");}
            return 1; 
        }
        printf("error, wrong argument for update_unfill function");
        return 1; 
    }
    printf("error, wrong argument for update_unfill function");
    return 1;
}

Angle depot_slot(Plant_Manager *PM, side side){
    if(side == LEFT){
        if(PM->storage.left.storage_1 == POT_AND_PLANT || PM->storage.left.storage_1 == DOUBLE_POT_AND_PLANT){ return PLANT1; }
        if(PM->storage.left.storage_2 == POT_AND_PLANT || PM->storage.left.storage_2 == DOUBLE_POT_AND_PLANT){ return PLANT2; }
        if(PM->storage.left.storage_3 == POT_AND_PLANT || PM->storage.left.storage_3 == DOUBLE_POT_AND_PLANT){ return PLANT3; }
        return STANDBY;
    } else {
        if(PM->storage.right.storage_1 == POT_AND_PLANT || PM->storage.right.storage_1 == DOUBLE_POT_AND_PLANT){ return PLANT1; }
        if(PM->storage.right.storage_2 == POT_AND_PLANT || PM->storage.right.storage_2 == DOUBLE_POT_AND_PLANT){ return PLANT2; }
        if(PM->storage.right.storage_3 == POT_AND_PLANT || PM->storage.right.storage_3 == DOUBLE_POT_AND_PLANT){ return PLANT3; }
        return STANDBY;
    }
}

double grab_height(Plant_Manager *PM, side side, Angle angle){
    double Double_pot_height = 7.0; 
    double Simple_pot_height = 7.5; 

    if(side == LEFT){
        if(angle == PLANT1){
            if(PM->storage.left.storage_1 == DOUBLE_POT_AND_PLANT){return Double_pot_height;}
            if(PM->storage.left.storage_1 == POT_AND_PLANT){return Simple_pot_height;}
            printf("error computing grabing height : No pot+plant in this slot, returning depth 1 \n"); 
            return 1.0; 
        }
        if(angle == PLANT2){
            if(PM->storage.left.storage_2 == DOUBLE_POT_AND_PLANT){return Double_pot_height;}
            if(PM->storage.left.storage_2 == POT_AND_PLANT){return Simple_pot_height;}
            printf("error computing grabing height : No pot+plant in this slot, returning depth 1 \n"); 
            return 1.0; 
        }
        if(angle == PLANT3){
            if(PM->storage.left.storage_3 == DOUBLE_POT_AND_PLANT){return Double_pot_height;}
            if(PM->storage.left.storage_3 == POT_AND_PLANT){return Simple_pot_height;}
            printf("error computing grabing height : No pot+plant in this slot, returning depth 1 \n"); 
            return 1.0; 
        }
        printf("error computing grabing height : angle isn't a slot angle \n");
        return 1.0;
    }

    if(side == RIGHT){
        if(angle == PLANT1){
            if(PM->storage.right.storage_1 == DOUBLE_POT_AND_PLANT){return Double_pot_height;}
            if(PM->storage.right.storage_1 == POT_AND_PLANT){return Simple_pot_height;}
            printf("error computing grabing height : No pot+plant in this slot, returning depth 1 \n"); 
            return 1.0; 
        }
        if(angle == PLANT2){
            if(PM->storage.right.storage_2 == DOUBLE_POT_AND_PLANT){return Double_pot_height;}
            if(PM->storage.right.storage_2 == POT_AND_PLANT){return Simple_pot_height;}
            printf("error computing grabing height : No pot+plant in this slot, returning depth 1 \n"); 
            return 1.0; 
        }
        if(angle == PLANT3){
            if(PM->storage.right.storage_3 == DOUBLE_POT_AND_PLANT){return Double_pot_height;}
            if(PM->storage.right.storage_3 == POT_AND_PLANT){return Simple_pot_height;}
            printf("error computing grabing height : No pot+plant in this slot, returning depth 1 \n"); 
            return 1.0; 
        }
        printf("error computing grabing height : angle isn't a slot angle \n");
        return 1.0;
    }

    return 0.0;
}

int is_full(Plant_Manager *PM, side Side){

    if(Side == LEFT){
        if(PM->type_expected == POT){
            if(PM->storage.left.storage_1 == POT){
                return 1; 
            }
        }
        if(PM->type_expected == PLANT){
            if(PM->storage.left.storage_1 == POT_AND_PLANT){
                return 1;
            }
        }     
    }
    if(Side == RIGHT){
        if(PM->type_expected == POT){
            if(PM->storage.right.storage_1 == POT){
                return 1; 
            }
        }
        if(PM->type_expected == PLANT){
            if(PM->storage.right.storage_1 == POT_AND_PLANT){
                return 1;
            }
        }     
    }
    
    return 0; 
}

int main(){
    Plant_Manager *plant_manager = new Plant_Manager;
    init_plant_manager(plant_manager);

    DynStruct *dyn = new DynStruct;
    XL_320 *Servo = new XL_320;
    plant_manager->dynamixels = dyn;
    plant_manager->dynamixels->Servo = Servo;
    dynamixel_init(plant_manager->dynamixels);
    

    while(0){
        dyn_go_to(plant_manager->dynamixels, LEFT, STANDBY, 100);
        sleep(1);
        dyn_go_to(plant_manager->dynamixels, LEFT, PICKING, 100);
        sleep(1);
        dyn_go_to(plant_manager->dynamixels, LEFT, PLANT1, 100);
        sleep(1);
        dyn_go_to(plant_manager->dynamixels, LEFT, PLANT2, 100);
        sleep(1);
        dyn_go_to(plant_manager->dynamixels, LEFT, PLANT3, 100);
        sleep(1);
        dyn_go_to(plant_manager->dynamixels, LEFT, STANDBY, 100);

        //same for the right dynamixel
        dyn_go_to( plant_manager->dynamixels, RIGHT, STANDBY, 100);
        sleep(1);
        dyn_go_to( plant_manager->dynamixels, RIGHT, PICKING, 100);
        sleep(1);
        dyn_go_to( plant_manager->dynamixels, RIGHT, PLANT1, 100);
        sleep(1);
        dyn_go_to( plant_manager->dynamixels, RIGHT, PLANT2, 100);
        sleep(1);
        dyn_go_to( plant_manager->dynamixels, RIGHT, PLANT3, 100);
        sleep(1);
        dyn_go_to( plant_manager->dynamixels, RIGHT, STANDBY, 100);
    }
    


    std::thread left_gripper(left_gripper_thread, std::ref(plant_manager));
    std::thread right_gripper(right_gripper_thread,std::ref(plant_manager));
    plant_manager->potting_enable = 1;

    left_gripper.join();
    right_gripper.join();

    sleep(10);

    return 0;
}

void left_gripper_thread(Plant_Manager* plant_manager){
    side* Side = new side; 
    *Side = LEFT; 
    while(!get_MS(plant_manager->fd,emergency)){
        picking_FSM(plant_manager, Side);
        /*if (*ctrl_c_pressed){ 
            break;
        }*/
    }
}

void right_gripper_thread(Plant_Manager* plant_manager){
    side* Side = new side; 
    *Side = RIGHT;
    while(!get_MS(plant_manager->fd,emergency)){
        picking_FSM(plant_manager, Side);
        /*if (*ctrl_c_pressed){ 
            break;
        }*/
    }
}
