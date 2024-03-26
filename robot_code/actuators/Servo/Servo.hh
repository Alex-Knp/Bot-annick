#pragma once

//#include "../../main/all_struct.hh"
//#include "../../communication/SPI_spidev.hh"
//#include "../../communication/SPI_spidev.cpp"
//#include "../../main/all_struct.cpp"
#include "../stepper/Stepper.hh"

int open_claw(int fd, side servo_id);
int close_claw(int fd, side servo_id);
int extend_pento(int fd, side servo_id);
int retract_pento(int fd, side servo_id);
void servo_ask(int fd, int id, int position);