#pragma once

#include "../main/all_struct.hh"
#include "../communication/SPI_spidev.hh"
#include "../localization/localisation.hh"
#include <stdint.h>
#include <stdio.h>
#include <cmath>
#define PI 3.14159265358979323846


typedef struct odometer_data {
    int32_t left_prev;
    int32_t right_prev;
} odometer_data;

void update_position_encoders(BigStruct *all_struct, struct odometer_data *odo_data);
int odo_init(BigStruct *all_struct,odometer_data *odo_data);