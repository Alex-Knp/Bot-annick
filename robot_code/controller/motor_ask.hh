#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>
#include <time.h> 
#include "../communication/SPI_spidev.hh"
#include "../main/all_struct.hh"

///   PIN DEFINITION   ///

void motor_ask(float dcL ,float dcR, BigStruct *all_struct);
void intsToBytes(int left_speed, int right_speed, uint8_t bytes[4]);

