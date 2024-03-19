#include "XL_320.cpp"

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

    // Servo.setTorqueEnable(0);
    // Servo.setReturnDelayTime(100);
    // Servo.setStatusReturnLevel(1);
    // Servo.setTorqueEnable(0);

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

        struct gpiod_chip *chip;
    struct gpiod_line *line;
    // Open GPIO chip
    chip = gpiod_chip_open("/dev/gpiochip0");

    // Get the line
    line = gpiod_chip_get_line(chip, 4); // GPIO 4
    printf("high\n");

    // Set the line state to HIGH
    gpiod_line_set_value(line, 1);

    // Sleep for a short duration
    usleep(10000000);

    // Set the line state to LOW
    gpiod_line_set_value(line, 0);
    printf("low\n");
    // Release the chip
    gpiod_line_release(line);
    gpiod_chip_close(chip);

    return 0;
}
