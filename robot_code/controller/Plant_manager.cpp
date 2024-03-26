#include "Plant_manager.hh"

int init_claw(int fd){

    //retract and close claw
    close_claw(fd, left);
   
    // Homing of the stepper
    int result = stepper_homing(fd, left);

    // Homing of the dynamixel
    
    // Go to start position 
    stepper_go_to(fd, left, 2, 20);
    sleep(1); 
    return 1;

}

void take_plant(int fd, side Side){

    // Turn dynamixel above plant


    // Down stepper to plant
    stepper_go_to(fd, Side, 12, 5);

    while(1){
        double test = get_stepper_position(fd, Side);
        if(test > 5){
            break;
        }
        printf("stepper hight = %f \n", test);
    }
    printf("go through");

    // Open the claw
    open_claw(fd, Side);

    while(1){
        if(get_stepper_position(fd, Side) == 12){
            break;
        }
    }

    // Close the claw
    close_claw(fd, Side);

    // Move the stepper storage hieght
    stepper_go_to(fd, Side, 2, 20);

    while(1){
        if(get_stepper_position(fd, Side) < 2.5 ){
            break; 
        }
    }

    // Turn dynamixel to storage angle

    // Stepper decent 
    stepper_go_to(fd, Side, 4, 20);

    while(1){
        if(get_stepper_position(fd, Side) < 3.8 ){
            break; 
        }
    }
    open_claw(fd, Side); 

    stepper_go_to(fd, Side, 2, 20);
    while(1){
        if(get_stepper_position(fd, Side) > 2.2 ){
            break; 
        }
    }

    close_claw(fd, Side); 
    


    //open_claw(fd, Side);
}

int main(){
    //stepper_homing(left);
    //sleep(4);
    int fd = spi_init_1();
    init_claw(fd);
    //take_plant(fd, left);
    //take_pot(fd, left);
    //stepper_go_to(fd, left, 3, 20);
    // sleep(1);
    // open_claw(fd, left);
    close(fd);
    return 0;
}
