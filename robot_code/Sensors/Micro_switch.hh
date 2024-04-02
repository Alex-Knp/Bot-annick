#pragma once

#include "../main/all_struct.hh"
#include "../communication/SPI_spidev.hh"

int get_MS(int fd, Micro_switch MS);
void print_MS(int fd);

