#include "motor_ask.h"

#ifndef MOTOR_ASK_C
#define MOTOR_ASK_C
/*
#include <wiringPi.h>
#include <stdio.h>
#include <stdlib.h>
#include <softPwm.h>


///   PIN DEFINITION   ///
#define GPIO_BT 26
#define L 12
#define R 13
#define dL 5
#define dR 6
#define freq 20000
*/
///   PWM bus initialisation   ///
int PWM1 = -1;
int PWM2 = -1;

///   PIN initialisation  ///
/* void init_motors() {
    if (wiringPiSetupGpio() == -1) {
        fprintf(stderr, "Failed to initialize wiringPi.\n");
        exit(1);
    }
    pinMode(GPIO_BT, INPUT);
    pinMode(L, OUTPUT); 
    pinMode(dL, OUTPUT); 
    pinMode(R, OUTPUT);
    pinMode(dR, OUTPUT);

    PWM1 = softPwmCreate(L, 0, 100); 
    PWM2 = softPwmCreate(R, 0, 100);
}


void Motor_clear(){
    if (PWM1 != -1){
        softPwmStop(L); 
        PWM1 = -1; 
    }
    if (PWM2 != -1){
        softPwmStop(R); 
        PWM2 = -1; 
    }

    digitalWrite(L, LOW); 
    digitalWrite(dL, LOW);
    digitalWrite(dR, LOW); 
    digitalWrite(R, LOW); 
}

void motor_ask(float dcL ,float dcR){
    
    if(dcL < 0){digitalWrite(dL, LOW);}
    else{digitalWrite(dL, HIGH);}

    softPwmWrite(L, abs(dcL)); 

    if(dcR < 0){digitalWrite(dR, LOW);}
    else{digitalWrite(dR, HIGH);}

    softPwmWrite(R, abs(dcR)); 
}

int button_ask() {
    return digitalRead(GPIO_BT);
}

int motors_test() {
    init_motors();
    motor_ask(50, 50);
    delay(600);

    Motor_clear(); 
    return -1;
} */

/*int main() {
    printf("button Test\n");
    printf("Press button\n");
    printf("%d\n", motors_test());
    return 0;
}*/

#endif // MOTOR_ASK_C