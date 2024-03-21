#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>
#include <time.h>  

///   PIN DEFINITION   ///

void motor_ask(float dcL ,float dcR);
void intsToBytes(int left_speed, int right_speed, uint8_t bytes[4]);

