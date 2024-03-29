#pragma once

#include "../main/all_struct.hh"
#include <iostream>
#include <thread>
#include <mutex>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>
#include <cmath>
#include <pthread.h> 
#include <fstream>


void main_strategy(BigStruct* all_struct);

bool verification_beacon(BigStruct* all_struct);

void pot_zone_select(BigStruct* all_struct, int num_pot);
void plant_zone_select(BigStruct* all_struct, int num_plant);
void drop_zone_select(BigStruct* all_struct,int num_drop_zone);