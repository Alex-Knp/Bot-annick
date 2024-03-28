

// main with the threads, we include all the .hh in same as all_struct.cpp
#include "main.hh"
#include "../localization/localisation.hh"
#include "../controller/motor_ask.hh"
#include "../communication/I2C.hh"
#include "../communication/SPI_spidev.hh"
#include "../communication/UART.hh"
#include "../Path_planning/Path_planning.hh"
#include "../Strategy/strategy.hh"


/*! \brief controller loop 
 *
 * \param[in] all_struct main structure
 */


int main()
{
    BigStruct *all_struct = init_BigStruct();
  
    scanLidar(all_struct);

    
    
    //std::thread scanThread(scanLidar, all_struct);
    //std::thread controlThread(main_controller, all_struct); 
  

    //controlThread.join();
    //scanThread.join();

    //main_camera(all_struct);        // à lancer par le troisième thread

    
    return 0;
}

/*! \brief last controller operations (called once)
 *
 * \param[in] all_struct controller main structure
 */
void controller_finish(BigStruct *all_struct)
{
    free_BigStruct(all_struct);
}
