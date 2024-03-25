#include "Micro_switch.hh"

void print_MS(int fd){

    uint8_t MS_message[5] = {0x7f, 0x00, 0x00, 0x00, 0x00};
    spi_transfer(fd, MS_message, 5);
    printf("MS_message: %u, %u, %u, %u, %u\n", MS_message[0], MS_message[1], MS_message[2], MS_message[3], MS_message[4]);
}

int get_MS(int fd, Micro_switch MS){
    // Function to get the value of the selected micro switch
    // fd : file descriptor of the SPI communication
    // return : 1 if the micro switch is pressed, 0 otherwise
    // MS : the micro switch to get the value from (left, right, emergency_stop)

    uint8_t MS_message[5] = {0x7f, 0x00, 0x00, 0x00, 0x00};
    spi_transfer(fd, MS_message, 5);

    switch (MS){ 
        case left:
            return MS_message[1]>100;
        case right:
            return MS_message[1]%2;
        case emergency:
            return abs(MS_message[2]-1);
        default:
            return -1;
    }
    return MS_message[1]%2;
}

int main(){
    int fd = spi_init_1();
    while(1){
        int result = get_MS(fd, right);
        printf("Right micro switch: %d\n", result);
    }
    
    return 0;
}