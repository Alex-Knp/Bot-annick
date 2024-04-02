#include "all_struct.hh"

#include "../localization/localisation.hh"
#include "../Path_planning/Path_planning.hh"
#include "../controller/motor_ask.hh"
#include "../communication/I2C.hh"
#include "../Strategy/strategy.hh"
#include "../communication/SPI_spidev.hh"
#include "../communication/UART.hh"
#include "../actuators/Servo/Servo.hh"
#include "../actuators/stepper/Stepper.hh"
#include "../Sensors/IR.hh"
#include "../Sensors/Micro_switch.hh"

/*! \brief initialize all the structure that we need
 * 
 * \param[in] inputs inputs of the controller
 * \param[in] outputs outputs of the controller
 * \return controller main structure
 */

BigStruct* init_BigStruct(){

	BigStruct* all_struct = new BigStruct;

	// start up
	all_struct->startup = false;

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

	all_struct->rob_pos->x = 1.486;
	all_struct->rob_pos->y = 1.314;
	all_struct->rob_pos->theta = 345.125*(M_PI/180);

	// Opponents position
 	all_struct->opp_pos = (OpponentsPosition*) malloc(sizeof(OpponentsPosition));
	all_struct->opp_pos->x = (double*) malloc(20*sizeof(double));
	all_struct->opp_pos->y = (double*) malloc(20*sizeof(double));
	for(int i=0; i<20; i++)
	{
		all_struct->opp_pos->x[i] = 0.0;
		all_struct->opp_pos->y[i] = 0.0;
	}

	all_struct->opp_pos->nb_opp = 0; 

	// lidar driver
	//all_struct->drv = connectLidar();

	//Time
	all_struct->start_time = time(NULL);
	all_struct->elapsed_time = 0.0;

	//strategy
	all_struct->strat = (Strategy*) malloc(sizeof(Strategy));
	all_struct->strat->state = CALIB_STATE;
	all_struct->strat->count_plant = 0;
	all_struct->strat->count_pot = 0;
	all_struct->strat->goal_x = 0.0;
	all_struct->strat->goal_y = 0.0;
	all_struct->strat->goal_theta = 0.0;
	all_struct->strat->goal_reached = false;
	all_struct->strat->count_pot = 0;
	all_struct->strat->count_plant = 0;

	all_struct->pot_list = (int*) malloc(7*sizeof(int));
	int[7] pot_list_blue = {0, 1, 2, 3, 4, 5, 6};// Faire un odre de passage des pots
	int[7] pot_list_yellow = {0, 1, 2, 3, 4, 5, 6};// Faire un odre de passage des pots

	if(all_struct->team_id == TEAM_BLUE)
	{
		for(int i=0; i<7; i++)
		{
			all_struct->pot_list[i] = pot_list_blue[i];
		}
		
	}
	else if(all_struct->team_id == TEAM_YELLOW)
	{
		for(int i=0; i<7; i++)
		{
			all_struct->pot_list[i] = pot_list_yellow[i];
		}
	}
	all_struct->strat->next_pot = all_struct->pot_list[0];

	return all_struct;
}

void free_BigStruct(BigStruct *all_struct)
{	
	free(all_struct->table);
	free(all_struct->rob_pos);

	//Opponent position free
	free(all_struct->opp_pos->x);
	free(all_struct->opp_pos->y);
	free(all_struct->opp_pos);

	
	//disconnectLidar(all_struct->drv);

	free(all_struct);
}