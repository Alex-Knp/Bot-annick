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
// Variables déclaration 
//double 

typedef struct Path_planning
{
	double norm;  ///< current norm of the force vector field
	double theta; ///< current angle of the force vector field
	double v_max;    ///< max speed
	int *active_zone;

} Path_planning;

void Path_planning_update(BigStruct* all_struct);
void speed_regulation(BigStruct* all_struct,double wanted_theta, double wanted_speed);

