#pragma once

#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <gpiod.h>

#define CE0 7  // Numéro de la broche GPIO pour le Chip Enable (CE)
#define CE1 8
#define MOSI_GPIO 10
#define MISO_GPIO 9
#define SCK_GPIO 11

struct gpiod_chip *spi_init_0(const char *spi_device);
struct gpiod_chip *spi_init_1(const char *spi_device);
int spi_ask(struct gpiod_chip *chip, unsigned char *tx_data, unsigned char *rx_data, size_t len);
int spi_sent(struct gpiod_chip *chip, unsigned char *tx_data);

