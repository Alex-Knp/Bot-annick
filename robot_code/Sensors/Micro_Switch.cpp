#include "Micro_switch.hh"

void print_MS(int fd){

    uint8_t MS_message[5] = {0x7f, 0x00, 0x00, 0x00, 0x00};
    spi_transfer(fd, MS_message, 5);
    printf("MS_message: %u, %u, %u, %u, %u\n", MS_message[0], MS_message[1], MS_message[2], MS_message[3], MS_message[4]);
}

int get_right_MS(int fd){
    // Function to get the value of the right micro switch
    // fd : file descriptor of the SPI communication
    // return : 1 if the micro switch is pressed, 0 otherwise

    uint8_t MS_message[5] = {0x7f, 0x00, 0x00, 0x00, 0x00};
    spi_transfer(fd, MS_message, 5);

    return MS_message[1]%2;
}

int get_left_MS(int fd){
    // Function to get the value of the left micro switch
    // fd : file descriptor of the SPI communication
    // return : 1 if the micro switch is pressed, 0 otherwise

    uint8_t MS_message[5] = {0x7f, 0x00, 0x00, 0x00, 0x00};
    spi_transfer(fd, MS_message, 5);


    return (MS_message[1]>100);
}

int get_emergency_stop(int fd){
    // Function to get the value of the emergency stop
    // fd : file descriptor of the SPI communication
    // return : 1 if the emergency stop is pressed, 0 otherwise

    uint8_t MS_message[5] = {0x7f, 0x00, 0x00, 0x00, 0x00};
    spi_transfer(fd, MS_message, 5);

    return abs(MS_message[2]-1);

}

int main(){
    int fd = spi_init_1();
    while(1){
        int result = get_emergency_stop(fd);
        printf("Right micro switch: %d\n", result);
    }
    
    return 0;
}