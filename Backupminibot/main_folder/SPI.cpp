#include "SPI.h"

#ifndef SPI_C 
#define SPI_C

// Function to perform SPI initialisation
void spi_init() {
     if (wiringPiSetup() == -1 || wiringPiSPISetup(SPI_CHANNEL, SPI_SPEED) == -1) {
        printf("Unable to initialize WiringPi or SPI. Make sure you have WiringPi installed.\n");
    }
}

// Function to perform SPI communication
void spi_ask(unsigned char addr, uint8_t* msg, uint8_t* response, int msg_len) {
    uint8_t to_send[msg_len + 1];
    unsigned char received[msg_len + 1];

    to_send[0] = addr;
    for (int i = 0; i < msg_len; i++) {
        to_send[i + 1] = msg[i];
    }

    wiringPiSPIDataRW(SPI_CHANNEL, to_send, msg_len + 1);

    for (int i = 0; i < msg_len + 1; i++) {
        response[i] = to_send[i];
    }
}

#endif //SPI_C