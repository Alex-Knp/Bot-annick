#include <gpiod.h>
#include <iostream>
#include <unistd.h>

int main() {
    // Open GPIO chip
    gpiod_chip *  chip = gpiod_chip_open_by_name("gpiochip4");
    if(chip == NULL){
        chip = gpiod_chip_open_by_name("gpiochip0");
        printf("c'était pas le 4\n");
        }
    if(chip == NULL)
    {
        printf("Unable to access GPIO\n");
        return -1;
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
