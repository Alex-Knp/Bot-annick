#include <gpiod.h>
#include <iostream>
#include <unistd.h>

int main() {
    // Open GPIO chip
    gpiod_chip *chip = gpiod_chip_open("/dev/gpiochip0");
    if (chip == nullptr) {
        std::cerr << "Failed to open GPIO chip" << std::endl;
        return 1;
    }

    // Get GPIO line
    gpiod_line *line = gpiod_chip_get_line(chip, 27); // Change GPIO number as needed
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

    // Release resources
    gpiod_line_release(line);
    gpiod_chip_close(chip);

    return 0;
}
