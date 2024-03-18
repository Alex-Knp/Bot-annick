/* 
#include <gpiod.h>
#include <stdio.h>
#include <unistd.h>

int main() {
	struct gpiod_chip *chip;
	struct gpiod_line *line;
	int req, value;

	chip = gpiod_chip_open("/dev/gpiochip0");
	if (!chip){
		perror("Failed to open GPIO chip");
		return -1;
	}

	line = gpiod_chip_get_line(chip, 3);
	if (!line) {
		gpiod_chip_close(chip);
		return -1;
	}

	req = gpiod_line_request_input(line, "gpio_state");
	if (req) {
		gpiod_chip_close(chip);
		return -1;
	}

	value = gpiod_line_get_value(line);

	printf("GPIO value is: %d\n", value);

	gpiod_chip_close(chip);
} */

#include "SPI.hh"

// Fonction pour initialiser SPI sur channel 0
struct gpiod_chip *spi_init_0(const char *spi_device) {
    struct gpiod_chip *chip = gpiod_chip_open(spi_device);
    if (!chip) {
        perror("Failed to open SPI device");
        return NULL;
    }

    // Initialiser la ligne GPIO pour le Chip Enable (CE)
    struct gpiod_line *ce_line = gpiod_chip_get_line(chip, CE0);
    if (!ce_line) {
        perror("Failed to get CE GPIO line");
        gpiod_chip_close(chip);
        return NULL;
    }
    if (gpiod_line_request_output(ce_line, "CE", 1) < 0) {
        perror("Failed to request CE GPIO line");
        gpiod_chip_close(chip);
        return NULL;
    }

    return chip;
}

// Fonction pour initialiser SPI sur channel 1

struct gpiod_chip *spi_init_1(const char *spi_device) {
    struct gpiod_chip *chip = gpiod_chip_open(spi_device);
    if (!chip) {
        perror("Failed to open SPI device");
        return NULL;
    }

    // Initialiser la ligne GPIO pour le Chip Enable (CE)
    struct gpiod_line *ce_line = gpiod_chip_get_line(chip, CE1);
    if (!ce_line) {
        perror("Failed to get CE GPIO line");
        gpiod_chip_close(chip);
        return NULL;
    }
    if (gpiod_line_request_output(ce_line, "CE", 1) < 0) {
        perror("Failed to request CE GPIO line");
        gpiod_chip_close(chip);
        return NULL;
    }

    return chip;
}

// Fonction pour transférer les données SPI
int spi_ask(struct gpiod_chip *chip, unsigned char *tx_data, unsigned char *rx_data, size_t len) {
    // Configuration de la ligne GPIO pour la communication SPI
    struct gpiod_line *mosi_line = gpiod_chip_get_line(chip, MOSI_GPIO);
    struct gpiod_line *miso_line = gpiod_chip_get_line(chip, MISO_GPIO);
    struct gpiod_line *sck_line = gpiod_chip_get_line(chip, SCK_GPIO);

    if (!mosi_line || !miso_line || !sck_line) {
        perror("Failed to get SPI GPIO lines");
        return -1;
    }
    if (gpiod_line_request_output(mosi_line, "MOSI", 0) < 0 || gpiod_line_request_input(miso_line, "MISO") < 0 || gpiod_line_request_output(sck_line, "SCK", 0) < 0) {
        perror("Failed to request SPI GPIO lines");
        return -1;
    }

    // Activer le Chip Enable (CE)
    struct gpiod_line *ce_line = gpiod_chip_get_line(chip, CE0);
    if (!ce_line) {
        perror("Failed to get CE GPIO line");
        return -1;
    }
    if (gpiod_line_set_value(ce_line, 0) < 0) {
        perror("Failed to set CE GPIO line");
        return -1;
    }

    // Transférer les données SPI
    for (size_t i = 0; i < len; ++i) {
        for (int j = 7; j >= 0; --j) {
            // Mettre à jour la ligne MOSI
            if (gpiod_line_set_value(mosi_line, (tx_data[i] >> j) & 0x01) < 0) {
                perror("Failed to set MOSI GPIO line");
                return -1;
            }

            // Lire la valeur sur la ligne MISO
            int miso_val = gpiod_line_get_value(miso_line);
            if (miso_val < 0) {
                perror("Failed to read MISO GPIO line");
                return -1;
            }

            // Lancer le signal d'horloge
            if (gpiod_line_set_value(sck_line, 1) < 0 || gpiod_line_set_value(sck_line, 0) < 0) {
                perror("Failed to toggle SCK GPIO line");
                return -1;
            }

            // Stocker la valeur lue (LSB-first)
            rx_data[i] |= (miso_val << j);
        }
    }

    // Désactiver le Chip Enable (CE)
    if (gpiod_line_set_value(ce_line, 1) < 0) {
        perror("Failed to set CE GPIO line");
        return -1;
    }

	gpiod_line_release(mosi_line);
	gpiod_line_release(miso_line);
	gpiod_line_release(sck_line);
	gpiod_line_release(ce_line);

    return 0;
}

int spi_sent(struct gpiod_chip *chip, unsigned char *tx_data) {
    // Configuration de la ligne GPIO pour la communication SPI
    struct gpiod_line *mosi_line = gpiod_chip_get_line(chip, MOSI_GPIO);
    struct gpiod_line *miso_line = gpiod_chip_get_line(chip, MISO_GPIO);
    struct gpiod_line *sck_line = gpiod_chip_get_line(chip, SCK_GPIO);
    if (!mosi_line || !miso_line || !sck_line) {
        perror("Failed to get SPI GPIO lines");
        return -1;
    }
    if (gpiod_line_request_output(mosi_line, "MOSI", 0) < 0 || gpiod_line_request_input(miso_line, "MISO") < 0 || gpiod_line_request_output(sck_line, "SCK", 0) < 0) {
        perror("Failed to request SPI GPIO lines");
        return -1;
    }

    // Activer le Chip Enable (CE)
    struct gpiod_line *ce_line = gpiod_chip_get_line(chip, CE0);
    if (!ce_line) {
        perror("Failed to get CE GPIO line");
        return -1;
    }
    if (gpiod_line_set_value(ce_line, 0) < 0) {
        perror("Failed to set CE GPIO line");
        return -1;
    }

    // Transférer les données SPI
    for (size_t i = 1; i < 5; ++i) {   //on envoie que 4 bytes à la teensy
        for (int j = 7; j >= 0; --j) {
            // Mettre à jour la ligne MOSI
            if (gpiod_line_set_value(mosi_line, (tx_data[i] >> j) & 0x01) < 0) {
                perror("Failed to set MOSI GPIO line");
                return -1;
            }

            // Lire la valeur sur la ligne MISO
            int miso_val = gpiod_line_get_value(miso_line);
            if (miso_val < 0) {
                perror("Failed to read MISO GPIO line");
                return -1;
            }

            // Lancer le signal d'horloge
            if (gpiod_line_set_value(sck_line, 1) < 0 || gpiod_line_set_value(sck_line, 0) < 0) {
                perror("Failed to toggle SCK GPIO line");
                return -1;
            }
        }
    }

    // Désactiver le Chip Enable (CE)
    if (gpiod_line_set_value(ce_line, 1) < 0) {
        perror("Failed to set CE GPIO line");
        return -1;
    }

	gpiod_line_release(mosi_line);
	gpiod_line_release(miso_line);
	gpiod_line_release(sck_line);
	gpiod_line_release(ce_line);

    return 0;
}

int main() {
    printf("SPI Test\n");

    // Initialiser SPI
    struct gpiod_chip *spi_chip0 = spi_init_0("/dev/spidev0.0");
	struct gpiod_chip *spi_chip1 = spi_init_1("/dev/spidev0.1");

    if (!spi_chip0) { fprintf(stderr, "SPI initialization0 failed\n"); return 1; }
	if (!spi_chip1) { fprintf(stderr, "SPI initialization1 failed\n"); return 1; }

    unsigned char tx_data1[] = {0x00, 0x00, 0x00, 0x00, 0x00};
    size_t len = sizeof(tx_data1) / sizeof(tx_data1[0]);
    unsigned char resp[len];

    // Transférer les données SPI via channel 1
    if (spi_ask(spi_chip1, tx_data1, resp, len) < 0) {
        fprintf(stderr, "SPI data transfer failed\n");
        gpiod_chip_close(spi_chip1);
        return 1;
    }

    printf("Received data from de0: ");
    for (size_t i = 0; i < len; ++i) {
        printf("%02x ", resp[i]);
    }
    printf("\n");
	// Transférer les données SPI via channel 0 (teensy) seulement 32 bits sans adresse
	unsigned char tx_data0[] = {0x00, 0x00, 0x00, 0x00};
	if (spi_sent(spi_chip0, tx_data0) < 0) {
		fprintf(stderr, "SPI data transfer failed\n");
		gpiod_chip_close(spi_chip0);
		return 1;
	}


    gpiod_chip_close(spi_chip0);
	gpiod_chip_close(spi_chip1);

    return 0;
}



