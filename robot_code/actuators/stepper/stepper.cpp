#include <stepper.hh>


int stepper_homing(int stepper_id) {
    // initialise le stepper "stepper_id" (id 0 = stepper gauche; id 1 = stepper droit)
    
    int micro_switch;
    uint8_t response[5];
    uint8_t Homing_enable[5];
    uint8_t Reset_signal[5];
    uint8_t zero_message[5] = {0x00, 0x00, 0x00, 0x00, 0x00};

    if(stepper_id == 0) {    // left stepper data
        stepper_adress = 0xf5;
        micro_switch = 3;
        uint8_t Homing_enable[] = {0x00, 0x01, 0x00, 0x00};
        uint8_t Reset_signal[]  = {0x00, 0x00, 0x01, 0x00};

    } else if(stepper_id == 1) {    //right stepper data
        stepper_adress = 0xf6;
        micro_switch = 4;
        uint8_t Homing_enable[] = {0x00, 0x00, 0x10, 0x00};
        uint8_t Reset_signal[]  = {0x00, 0x00, 0x00, 0x10};

    } else {
        printf("Erreur: l'identifiant du stepper doit être 0 ou 1.\n");
        return -1;
    }  

    //clear comunication
    spi_ask(0xf0, zero_message, response, 4);
    /*if(response[1] != 0x0f || response[2] != 0x0f || response[3] != 0x0f || response[4] != 0x0f){
        printf("Spi communicaiton failed (Clear communication) \n");
        return -1;} // fin de la procédure*/
    delay(1000);

    spi_ask(0xf0, Homing_enable, response, 4);

    delay(1000);

    while(true){
        // recupère l'info des micro-switch
        spi_ask(0x7f, zero_message, response, 4);
        printf("Micro-switch tracking : %u, %u, %u, %u \n", response[1], response[2], response[3], response[4]);

        // vérifie l'activation des micro-switch
        if(response[micro_switch] == 1){

            // reset l'instance stepper à 0
            spi_ask(0xf0, Reset_signal, response, 4);

            delay(500);

            // set la target position du stepper à 0
            spi_ask(stepper_adress, zero_message, response, 4);
            
            delay(500);

            // set la target position du stepper à 0
            spi_ask(0xf0, zero_message, response, 4);

            return 1; // fin de la procédure
        }
        delay(10);
    }
}

int stepper_ask(double depth, int speed_coef, int stepper_id){
        
        unsigned char stepper_adress; 
        uint8_t control_message[4];
        uint8_t response[5];


        if(stepper_id == 0){        // left stepper
            stepper_adress = 0xf5;}

        else if(stepper_id == 1){   // right
            stepper_adress = 0xf6;}

        else{
            printf("Erreur: l'identifiant du stepper doit être 0 ou 1.\n");
            return -1;}

        //conpute of number of step 
        double step = depth*3200*5;
        int step_reel = static_cast<int>(std::floor(step)); 

        control_message[1] = (step_reel >> 16) & 0xff; 
        control_message[2] = (step_reel >> 8) & 0xff; 
        control_message[3] = step_reel & 0xff; 
        control_message[0] = speed_coef & 0xff;
        spi_ask(stepper_adress, control_message, response, 4);
  
    return 1; 
}


int main() {

    printf("Homing of right stepper\n");
    spi_init_1();

    int test; 

    if(stepper_homing(1) != 1){
        printf("an error occured during homing of right stepper \n"); 
        return 0; }

    stepper_ask(1.0, 1, 0);
    /*delay(2000); 
    stepper_ask(50.0, 2, 1);
    delay(2000); 
    stepper_ask(50.0, 3, 1);
    delay(2000); 
    stepper_ask(50.0, 4, 1);
    delay(2000); 
    stepper_ask(50.0, 5, 1);
    delay(2000); 
    stepper_ask(50.0, 6, 1);
    delay(2000); 
    stepper_ask(50.0, 7, 1);
    delay(2000); 
    stepper_ask(50.0, 8, 1);
    delay(2000); 
    stepper_ask(50.0, 9, 1);*/



    printf("no error occured durin homing of right stepper \n");
    return 0;}

