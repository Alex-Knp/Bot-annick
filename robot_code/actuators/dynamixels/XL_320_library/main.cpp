#include "XL_320.hpp"

using namespace std;

int main(int argc, char **argv)
{

    XL_320 Servo;
    Servo.verbose = true;
    Servo.is_id(0x05);

    // Servo.factory_reset();
    // Servo.is_id(0x01);
    // Servo.setID(5);
    // Servo.is_id(0x05);

    // --- Timing optimization

    /*
     * Do this only once per Servo, the result is stored in the EEPROM
     */

    Servo.setTorqueEnable(0);
    Servo.setReturnDelayTime(100);
    Servo.setStatusReturnLevel(1);
    Servo.setTorqueEnable(0);

    // --- Commands

    // // Test ping
    Servo.ping();

    // // Test LED colors
    // for (int i=0; i<8; i++) {
    //     Servo.setLED(i);
    //     usleep(1000000);
    // }

    Servo.setTorqueEnable(0);
    Servo.setControlMode(2);
    Servo.setSpeed(1023);
    Servo.setGoalPosition(500);
    Servo.setTorqueEnable(1);
    //sleep(1);
    //Servo.setSpeed(0);
    //Servo.setTorqueEnable(0);

    return 0;
}