#include "Servo.hh"

// 2 : pento left, 1 : claw left, 4: pento right, 3: claw right
// 1_000 : Close ; 91_000: close

int open_claw(int fd, side servo_id) {
    // open the claw of the robot
    int id; 
    int position = 0;

    if(servo_id == left) {
        int position = 0;
        int id = 1;
  
    } else if(servo_id == right) {
        int position = 0;
        int id = 3;
        
    } else {
        printf("error : id should be left (left claw) or right (right servo)\n");
        return -1;
    }

    servo_ask(fd, id, position);

    return 1;
}

int close_claw(int fd, side servo_id) {
    // close the claw of the robot
    int id; 
    int position = 1;

    if(servo_id == left) {
        int position = 1;
        int id = 1;
        
    } else if(servo_id == right) {
        int position = 1;
        int id = 3;
        
    } else {
        printf("error : id should be left (left claw) or right (right servo)\n");
        return -1;
    }

    servo_ask(fd, id, position);

    return 1;
}

int extend_pento(int fd, side servo_id) {
    // extend the pento of the robot
    int id; 
    int position = 0;

    if(servo_id == left) {
        int position = 0;
        int id = 2;
        
    } else if(servo_id == right) {
        int position = 0;
        int id = 4;
        
    } else {
        printf("error : id should be left (left pento) or right (right pento)\n");
        return -1;
    }

    servo_ask(fd, id, position);

    return 1;
}

int retract_pento(int fd, side servo_id) {
    // retract the pento of the robot
    int id; 
    int position = 1;

    if(servo_id == left) {
        int position = 1;
        int id = 2;
        
    } else if(servo_id == right) {
        int position = 1;
        int id = 4;
        
    } else {
        printf("error : id should be left (left pento) or right (right pento)\n");
        return -1;}

    servo_ask(fd, id, position);

    return 1;
}

void servo_ask(int fd, int servo_id, int position) {
    // ask the position of the servo

    uint8_t Servo_message[5] = {0xf0, 0x00, 0x00, 0x00, 0x00};

    switch(servo_id) {
        case 1:
            Servo_message[0] = 0xf1;
            break;
        case 2:
            Servo_message[0] = 0xf2;
            break;
        case 3:
            Servo_message[0] = 0xf3;
            break;
        case 4:
            Servo_message[0] = 0xf4;
            break;
        default:
            printf("error : id should be between 1 and 8\n");
            return;
    }

    
    Servo_message[1] = (position >> 24) & 0xff;
    Servo_message[2] = (position >> 16) & 0xff; 
    Servo_message[3] = (position >>  8) & 0xff; 
    Servo_message[4] = position & 0xff;

    printf("Servo message : %u, %u, %u, %u, %u\n", Servo_message[0], Servo_message[1], Servo_message[2], Servo_message[3], Servo_message[4]);

    spi_transfer(fd, Servo_message, 5);
}

int main(){
    printf("Ask the position of the servo\n");
    int fd = spi_init_1();

    int position = 5000; 
    while(1){
        servo_ask(fd, 2, position);
        printf("asked position : %d\n", position);
        usleep(3000000);    // 1s
        position = position - 1000;
    }

    close(fd);
    return 0;
}