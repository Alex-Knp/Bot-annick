
#include "motor_ask.hh"


void motor_ask(float left_speed ,float right_speed, BigStruct* all_struct){
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


    intsToBytes(left_speed_int,right_speed_int,byteBuffer);
    spi_transfer(all_struct->fd0, byteBuffer,4);
}

void intsToBytes(int left_speed, int right_speed, uint8_t bytes[4]) {
    bytes[1] = (left_speed >> 8) & 0xFF; // Bits de poids fort de value1
    bytes[0] = left_speed & 0xFF;        // Bits de poids faible de value1
    bytes[3] = (right_speed >> 8) & 0xFF; // Bits de poids fort de value2
    bytes[2] = right_speed & 0xFF;        // Bits de poids faible de value2
}

 /* int main(){
    
    BigStruct *all_struct = init_BigStruct();
    printf("fd0 after init = %d\n", all_struct->fd0);
    printf("motor_ask test start\n");
    motor_ask(10.0,-10.0, all_struct);
    printf("fd0 second = %d\n", all_struct->fd0);
    sleep(2);
    motor_ask(-10.0,10.0, all_struct);
    printf("third fd0 = %d\n", all_struct->fd0);
    sleep(2);
    motor_ask(0,0,all_struct);

    close(all_struct->fd0);
    free_BigStruct(all_struct);

  
 }  */

