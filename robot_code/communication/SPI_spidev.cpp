#include "SPI_spidev.hh"

uint8_t mode = 0;
uint32_t speed = 500000;
uint8_t bits = 8;  // amount of bits that we want to write or read on every spi access

/**
 * @brief function makes a SPI transfer
 *
 * @param fd		File descriptor to SPI bus
 * @param data		Data array with output (write) data
 *					will be overwritten with the received input data
 *	@param length	Length of the data array
 */
void spi_transfer(int fd, uint8_t *data, int length) {
	struct spi_ioc_transfer spi[length];
	int i;

	/* Setup transfer struct */
	for (i=0; i<length; i++) {
		memset(&spi[i], 0, sizeof(struct spi_ioc_transfer));
		spi[i].tx_buf = (unsigned long) (data+i);
		spi[i].rx_buf = (unsigned long) (data+i);
		spi[i].len = 1;
		spi[i].speed_hz = speed;
		spi[i].bits_per_word = bits;
	}

	/* Let's do the transfer */
	if(ioctl(fd, SPI_IOC_MESSAGE(length), spi) < 0) {
		perror("Error transfering data over SPI bus");
		close(fd);
		exit(-1);
	}
}

int spi_init_0(){
	int fd;

	/* Open the SPI bus file descriptor */
	fd = open("/dev/spidev0.0", O_RDWR);
	if(fd < 0) {
		perror("Error opening SPI Bus");
		return -1;
	}

	/* Setup of the SPI Bus */
	if(ioctl(fd, SPI_IOC_WR_MODE, &mode) < 0) {
		perror("Error setting the SPI mode");
		close(fd);
		return -1;
	}
	if(ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
		perror("Error setting the SPI speed");
		close(fd);
		return -1;
	}
	if(ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0) {
		perror("Error setting the SPI wordlength");
		close(fd);
		return -1;
	}
	return fd;
}

int spi_init_1(){
	int fd;

	/* Open the SPI bus file descriptor */
	fd = open("/dev/spidev0.1", O_RDWR);
	if(fd < 0) {
		perror("Error opening SPI Bus");
		return -1;
	}

	/* Setup of the SPI Bus */
	if(ioctl(fd, SPI_IOC_WR_MODE, &mode) < 0) {
		perror("Error setting the SPI mode");
		close(fd);
		return -1;
	}
	if(ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
		perror("Error setting the SPI speed");
		close(fd);
		return -1;
	}
	if(ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0) {
		perror("Error setting the SPI wordlength");
		close(fd);
		return -1;
	}
	return fd;
}

int main() {


	// teensy test
    int fd0 = spi_init_0();
	uint8_t data[4] = {0x00, 0x00, 0x00, 0x00};    // adresse
	spi_transfer(fd0, data, 4);

	close(fd0);

	// de0 test
	int fd1 = spi_init_1();

	uint8_t adr1 = 0x00;
	uint8_t data1[5] = {adr1, 0x00, 0x00, 0x00, 0x00};    // adresse
	spi_transfer(fd1, data1, 5);
	
	return 0;
}