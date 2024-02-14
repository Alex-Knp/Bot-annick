#pragma once

#ifndef SPI_H
#define SPI_H

#include <stdio.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <stdint.h>

// SPI channel (CE1)
#define SPI_CHANNEL 1

// Maximum SPI speed
#define SPI_SPEED 500000

void spi_init();

void spi_ask(unsigned char addr, uint8_t* msg, uint8_t* response, int msg_len);


#endif // SPI_H