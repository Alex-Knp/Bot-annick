
#include "I2C.hh"
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <stdint.h>

int i2c_bus = -1;

int init_i2c(const char *device_path, int slave_address) {
    // Ouverture du bus I2C
    i2c_bus = open(device_path, O_RDWR);
    if (i2c_bus < 0) {
        perror("Failed to open the I2C bus");
        return -1;
    }

    // Configuration de l'adresse esclave
    if (ioctl(i2c_bus, I2C_SLAVE, slave_address) < 0) {
        perror("Failed to set the slave address");
        close(i2c_bus);
        return -1;
    }

    return 0; // Initialisation réussie
}

int send_i2c(const uint8_t *data, size_t size) {
    if (write(i2c_bus, data, size) != (ssize_t)size) {
        perror("Failed to write to the I2C bus");
        return -1;
    }
    return 0; // Envoi réussi
}

int receive_i2c(uint8_t *buffer, size_t size) {
    ssize_t bytes_received = read(i2c_bus, buffer, size);
    if (bytes_received != (ssize_t)size) {
        perror("Failed to read from the I2C bus");
        return -1;
    }
    return 0; // Réception réussie
}

int main() {
    if (init_i2c("/dev/i2c-1", 0x04) != 0) {                             // 0x04 ici c'est l'adresse du device
        fprintf(stderr, "Erreur lors de l'initialisation du bus I2C\n");
        return 1;
    }

    // Exemple d'envoi de bytes
    uint8_t data_to_send[] = {0x00, 0x01, 0x02}; // Données à envoyer
    if (send_i2c(data_to_send, sizeof(data_to_send)) != 0) {
        fprintf(stderr, "Erreur lors de l'envoi de données via I2C\n");
        close(i2c_bus);
        return 1;
    }

    // Exemple de réception de bytes
    uint8_t received_data[4]; // Buffer pour stocker les données reçues
    if (receive_i2c(received_data, sizeof(received_data)) != 0) {
        fprintf(stderr, "Erreur lors de la réception de données via I2C\n");
        close(i2c_bus);
        return 1;
    }

    // Fermeture du bus I2C
    close(i2c_bus);
    return 0;
}














































/*   ici on écrit dans la led et on lit dans le button

 int main(int argc, char *argv[]) {
    int i2c_bus;
    uint8_t data[2];

    // Opening the bus
    i2c_bus = open("/dev/i2c-1", O_RDWR);

    if (i2c_bus < 0) {
        perror("Failed to open the i2c bus");
        close(i2c_bus);
        return -1;
    }

    //Setting the slave address
    if (ioctl(i2c_bus, I2C_SLAVE, 0x04) < 0) {      // 0x04 est l'adresse du slave
        perror("Failed to set the slave address");
        close(i2c_bus);
        return -1;
    }

    //Access the bus

    //Set the gpioa direction
    data[0] = 0x00; // adresse du registre (dans ce cas ci c'est le zero)
    data[1] = 0x00; // valeur à écrire dans le registre

    if (write(i2c_bus, data, 2) != 2) {
        perror("Failed to write to the i2c bus");
        close(i2c_bus);
        return -1;
    }

    //Set the LED's state
	data[0] = 0x14;          // addresse de la LED
	if(argc > 1)
		data[1] = atoi(argv[1]) > 0;
	else
		data[1] = 0x0;
	
	if(write(i2c_bus, data, sizeof(data)) != 2) {
		perror("Error writing to I2C bus");
		close(i2c_bus);
		return -1;
	}

	// Let's read the button's state 
	// Frist write out the address 
	data[0] = 0x12;
	if(write(i2c_bus, data, 1) != 1) {
		perror("Error writing to I2C bus");
		close(i2c_bus);
		return -1;
	}

	// Let's read the value back 
	if(read(i2c_bus, data, 1) != 1) {
		perror("Error reading from I2C bus");
		close(i2c_bus);
		return -1;
	}

	printf("Button is %s\n", (data[0] & (1<<1)) ? "Pressed" : "Not pressed");
	close(i2c_bus);
	return 0;


} */