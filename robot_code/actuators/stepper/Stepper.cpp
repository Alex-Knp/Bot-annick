#include "Stepper.hh"
#include <time.h>
#include <stdint.h>
#include <math.h>

int stepper_homing(int fd, int stepper_id) {
    // initialise le stepper "stepper_id" (id 0 = stepper gauche; id 1 = stepper droit)
    
    int micro_switch;
    uint8_t response[5];
    uint8_t Homing_enable[5] = {0xf0, 0x00, 0x00, 0x00, 0x00};
    uint8_t Reset_message[5] = {0xf0, 0x00, 0x00, 0x00, 0x00};
    uint8_t Reset_position[5] = {0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t UnReset_message[5] = {0xf0, 0x00, 0x00, 0x00, 0x00};
    uint8_t Clear_com[5] = {0x00, 0x00, 0x00, 0x00, 0x00};

    // DATA INITIALIZATION
    if(stepper_id == 0) {    // left stepper data
        micro_switch = 4;
        Homing_enable[2] = 0x01;
        Reset_message[3] = 0x01;
        Reset_position[0] = 0xf5;

    } else if(stepper_id == 1) {    //right stepper data
        micro_switch = 3;
        Homing_enable[2] = 0x10;
        Reset_message[3] = 0x10;
        Reset_position[0] = 0xf6;
    } else {
        printf("error : id should be 0 (left stepper) or 1 (right stepper)\n");
        return -1;}

    // RESET COMM
    spi_transfer(fd, Clear_com, 5);

    // HOMING ENABLE
    spi_transfer(fd, Homing_enable, 5);

    while(true){

        // MICRO-SWITCH TRACKING
        uint8_t MS_message[5] = {0x7f, 0x00, 0x00, 0x00, 0x00};
        spi_transfer(fd, MS_message, 5);
        //printf("Micro-switch tracking : %u, %u, %u, %u \n", response[1], response[2], response[3], response[4]);

        if(MS_message[micro_switch] == 1){

            // RESET STEPPER
            spi_transfer(fd, Reset_message, 5);

            // RESET TARGET POSITION
            spi_transfer(fd, Reset_position, 5);

            // UNRESET STEPPER
            spi_transfer(fd, UnReset_message, 5);

            return 1;
        }
    }
}

int stepper_go_to(int fd, int stepper_id, double depth, int speed_coef){
        
    unsigned char stepper_adress; 
    uint8_t control_message[5];
    uint8_t response[5];


    if(stepper_id == 0){        // left stepper
        control_message[0] = 0xf5;}

    else if(stepper_id == 1){   // right
        control_message[0] = 0xf6;}

    else{
        printf("Erreur: l'identifiant du stepper doit être 0 ou 1.\n");
        return -1;}

    //conpute of number of step 
    double step = depth*3200*5;
    int step_reel = static_cast<int>(std::floor(step)); 

    control_message[2] = (step_reel >> 16) & 0xff; 
    control_message[3] = (step_reel >> 8) & 0xff; 
    control_message[4] = step_reel & 0xff; 
    control_message[1] = speed_coef & 0xff;

    spi_transfer(fd, control_message, 5);

    return 1; 
}

int main() {

    printf("Homing of right stepper\n");
    int fd = spi_init_1();

    if(stepper_homing(fd, 0) != 1){
        printf("an error occured during homing of right stepper \n"); 
        return 0; }

    //stepper_ask(fd, 1, 3, 3);

    close(fd);


    printf("no error occured durin homing of right stepper \n");
    return 0;}

