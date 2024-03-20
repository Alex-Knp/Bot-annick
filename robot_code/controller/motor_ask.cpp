
#include "motor_ask.hh"
#include "../communication/SPI_spidev.hh"


int button_ask() {
    uint8_t TEST_MESSAGE[4] = {0x00, 0x00, 0x00, 0x00};
    uint8_t* resp = new uint8_t;
    int fd1 = spi_init_1();

    spi_transfer(fd1, TEST_MESSAGE, 4);
    close(fd1);
    
    return resp[2];
}

void motor_ask(float left_speed ,float right_speed){
    uint8_t byteBuffer[4];
    int left_speed_int = (int)((left_speed+30)*250);
    int right_speed_int = (int)((right_speed+30)*250);

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

int main(){
    printf("motor_ask test start");
    motor_ask(25,-25);
    
}

