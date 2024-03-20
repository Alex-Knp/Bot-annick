
#pragma once

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>


void spi_transfer(int fd, uint8_t *data, int length);
int spi_init_0();
int spi_init_1();
void intsToBytes(int left_speed, int right_speed, uint8_t bytes[4]);