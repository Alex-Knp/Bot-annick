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

typedef struct Strategy
{
    int state;
    int next_state;
    double goal_x;
    double goal_y;
    double goal_theta;
    bool goal_reached;
    int next_pot;
    int count_pot;

} Strategy;

void main_strategy(BigStruct* all_struct);

bool verification_beacon(BigStruct* all_struct);