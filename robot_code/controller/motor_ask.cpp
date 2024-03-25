
#include "motor_ask.hh"


void motor_ask(float left_speed ,float right_speed){
    if(left_speed>40){
        left_speed = 40;
    }
    if(left_speed<-40){
        left_speed = -40;
    }
    if(right_speed>40){
        right_speed = 40;
    }
    if(right_speed<-40){
        right_speed = -40;
    }

    uint8_t byteBuffer[4];
    int left_speed_int = (int)((left_speed+40)*800);
    int right_speed_int = (int)((right_speed+40)*800);

    int fd0 = spi_init_0();

    intsToBytes(left_speed_int,right_speed_int,byteBuffer);
    spi_transfer(fd0, byteBuffer,4);
    close(fd0);
}

void intsToBytes(int left_speed, int right_speed, uint8_t bytes[4]) {
    bytes[1] = (left_speed >> 8) & 0xFF; // Bits de poids fort de value1
    bytes[0] = left_speed & 0xFF;        // Bits de poids faible de value1
    bytes[3] = (right_speed >> 8) & 0xFF; // Bits de poids fort de value2
    bytes[2] = right_speed & 0xFF;        // Bits de poids faible de value2
}

// int main(){
//     printf("motor_ask test start");
//     motor_ask(20.0,-10.0);
    
// }

