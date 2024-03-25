#include "SPI_spidev.hh"

uint8_t mode = 0;
uint32_t speed = 5000000;
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

// int main() {
// 	odo_calib();
// // 	float left_speed = 35;
// // 	float right_speed = -35;
// // 	uint8_t byteBuffer[4];
// //     int left_speed_int = (int)((left_speed+30)*250);
// //     int right_speed_int = (int)((right_speed+30)*250);

// //     intsToBytes(left_speed_int,right_speed_int,byteBuffer); 
// // 	// teensy test
// //     int fd0 = spi_init_0();
// // 	uint8_t data[4] = {0x00, 0x0A, 0x00, 0x0A};    

// // 	spi_transfer(fd0, byteBuffer, 4);

// // 	close(fd0); 
// // 	/*  
// // 	// de0 test
// // 	int fd1 = spi_init_1();

// // 	uint8_t adr1 = 0xE0;
// // 	uint8_t data1[5] = {adr1, 0x55, 0x45, 0xa8, 0x77};   
// // 	spi_transfer(fd1, data1, 5);*/
	
// // 	for (int i = 0; i < 4; i++) {
// //         printf("%02X ", data[i]);
// //     }
// // 	printf("\n");
	
// 	return 0;
// }

// int odo_calib(){
// 	int fd0 = spi_init_1();
// 	uint8_t left_init[5] = {0x10, 0x0A, 0x00, 0x0A, 0x00};    
// 	uint8_t right_init[5] = {0x20, 0x0A, 0x00, 0x0A, 0x00};
// 	spi_transfer(fd0, left_init, 5);
// 	spi_transfer(fd0, right_init, 5);

// 	float left_init_value = left_init[1] << 24 | left_init[2] << 16 | left_init[3] << 8 | left_init[4];
// 	float right_init_value = right_init[1] << 24 | right_init[2] << 16 | right_init[3] << 8 | right_init[4];
// 	printf("left_init: %f, right_init: %f\n", left_init_value, right_init_value);

// 	for(int i = 0; i < 100; i++){
// 		uint8_t left[5] = {0x10, 0x00, 0x00, 0x00, 0x00};    
// 		uint8_t right[5] = {0x20, 0x00, 0x00, 0x00, 0x00};
// 		spi_transfer(fd0, left, 5);
// 		spi_transfer(fd0, right, 5);
// 		float left_value = (left[1] << 24 | left[2] << 16 | left[3] << 8 | left[4])-left_init_value;
// 		float right_value = (right[1] << 24 | right[2] << 16 | right[3] << 8 | right[4])-right_init_value;
// 		printf("left: %f, right: %f, sum:%f\n", left_value, right_value, left_value - right_value);
// 		printf("%x %x %x %x %x\n", left[0], left[1], left[2], left[3], left[4]);
// 		printf("%x %x %x %x %x\n", right[0], right[1], right[2], right[3], right[4]);
// 		sleep(1);
// 	}

// 	close(fd0); 
// 	return 0;
// }