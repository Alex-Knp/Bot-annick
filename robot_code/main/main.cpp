

// main with the threads, we include all the .hh in same as all_struct.cpp
#include "main.hh"
#include "localisation.hh"
#include "motor_ask.hh"
#include "I2C.hh"
#include "SPI.hh"
#include "SPI_spidev.hh"
#include "UART.hh"


/*! \brief controller loop 
 *
 * \param[in] all_struct main structure
 */


void controller_loop(BigStruct *all_struct)
{
    BigStruct *all_struct = init_BigStruct();
    ILidarDriver* drv = connectLidar();



    main_lidar(all_struct);         // à lancer par le premier thread
    main_controller(all_struct);    // à lancer par le deuxième thread
    //main_camera(all_struct);        // à lancer par le troisième thread
}

/*! \brief last controller operations (called once)
 *
 * \param[in] all_struct controller main structure
 */
void controller_finish(BigStruct *all_struct)
{
    disconnectLidar(drv);
    free_BigStruct(all_struct);
}
