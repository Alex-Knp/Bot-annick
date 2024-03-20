#include "XL_320.hpp"
#include <gpiod.h>
#include <stdio.h>
#include <stdlib.h>

using namespace std;

int main(int argc, char **argv)
{

    XL_320 Servo;
    Servo.verbose = true;
    Servo.factory_reset();

    // --- Timing optimization

    /*
     * Do this only once per Servo, the result is stored in the EEPROM
     */

    Servo.setTorqueEnable(0);
    Servo.setReturnDelayTime(100);
    Servo.setStatusReturnLevel(1);
    Servo.setTorqueEnable(0);

    // --- Commands
    printf("before ping\n");
    // Test ping
    //Servo.ping();
    printf("after ping\n");

    // Test LED colors
    for (int i=0; i<8; i++) {
        Servo.setLED(i);
        usleep(100000);
    }
    printf("after LED\n");
    Servo.setControlMode(2);

    Servo.setTorqueEnable(1);
    Servo.setGoalPosition(512);
    usleep(1000000);
    Servo.setGoalPosition(1023);
    usleep(1000000);
    Servo.setGoalPosition(512);
    usleep(1000000);
    Servo.setGoalPosition(0);
    Servo.setTorqueEnable(0);

    gpiod_chip *chip = gpiod_chip_open("/dev/gpiochip4");
    if (chip == nullptr) {
        std::cerr << "Failed to open GPIO chip" << std::endl;
        return 1;
    }

    printf("start\n");
    // Get GPIO line
    gpiod_line *line = gpiod_chip_get_line(chip, 4); // Change GPIO number as needed
    if (line == nullptr) {
        std::cerr << "Failed to get GPIO line" << std::endl;
        gpiod_chip_close(chip);
        return 1;
    }

    // Request output mode for the line
    int ret = gpiod_line_request_output(line, "example", 0);
    if (ret < 0) {
        std::cerr << "Failed to request output for GPIO line" << std::endl;
        gpiod_line_release(line);
        gpiod_chip_close(chip);
        return 1;
    }

    // Drive GPIO high
    ret = gpiod_line_set_value(line, 1);
    if (ret < 0) {
        std::cerr << "Failed to set GPIO value" << std::endl;
        gpiod_line_release(line);
        gpiod_chip_close(chip);
        return 1;
    }

    // Wait for 10 seconds
    sleep(10);

    // Drive GPIO low
    ret = gpiod_line_set_value(line, 0);
    if (ret < 0) {
        std::cerr << "Failed to set GPIO value" << std::endl;
        gpiod_line_release(line);
        gpiod_chip_close(chip);
        return 1;
    }

    printf("end\n");

    // Release resources
    gpiod_line_release(line);
    gpiod_chip_close(chip);

    return 0;
}
