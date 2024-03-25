#pragma once

#ifndef MOTOR_ASK_H
#define MOTOR_ASK_H

//#include <wiringPi.h>
#include <stdio.h>
#include <stdlib.h>
//#include <softPwm.h>


///   PIN DEFINITION   ///
#define GPIO_BT 26
#define L 12
#define R 13
#define dL 5
#define dR 6
#define freq 20000

void init_motors();
void Motor_clear();
void motor_ask(float dcL ,float dcR);
int button_ask();
int motors_test(); 

#endif // MOTOR_ASK_H