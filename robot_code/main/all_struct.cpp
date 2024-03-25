#include "all_struct.hh"

#include "../localization/localisation.hh"
#include "../controller/motor_ask.hh"
#include "../communication/I2C.hh"
#include "../communication/SPI_spidev.hh"
#include "../communication/UART.hh"

/*! \brief initialize all the structure that we need
 * 
 * \param[in] inputs inputs of the controller
 * \param[in] outputs outputs of the controller
 * \return controller main structure
 */

BigStruct* init_BigStruct(){

	BigStruct* all_struct = new BigStruct;


	// Localisation data
    all_struct->rob_pos = new RobotPosition;

	// localization

	all_struct->table = new lidar_data;
  	all_struct->table->beaconsx[0] = 1950;
  	all_struct->table->beaconsx[1] = 1000;
	all_struct->table->beaconsx[2] = 50;

  	all_struct->table->beaconsy[0] = -94;
  	all_struct->table->beaconsy[1] = 3094;
	all_struct->table->beaconsy[2] = -94;

  	all_struct->table->theta_lidar_calibration = M_PI/2; 

	// robot position
	all_struct->rob_pos = (RobotPosition*) malloc(sizeof(RobotPosition));

	all_struct->rob_pos->x = 0.0;
	all_struct->rob_pos->y = 0.0;
	all_struct->rob_pos->theta = 0.0;

	// lidar driver
	all_struct->drv = connectLidar();



	return all_struct;
}

void free_BigStruct(BigStruct *all_struct)
{	
	free(all_struct->table);
	free(all_struct->rob_pos);
	disconnectLidar(all_struct->drv);

	free(all_struct);
}