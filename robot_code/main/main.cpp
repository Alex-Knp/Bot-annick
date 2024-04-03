

// main with the threads, we include all the .hh in same as all_struct.cpp
#include "main.hh"
#include "../localization/localisation.hh"
#include "../controller/motor_ask.hh"
#include "../communication/I2C.hh"
#include "../communication/SPI_spidev.hh"
#include "../communication/UART.hh"
#include "../Path_planning/Path_planning.hh"
#include "../Strategy/strategy.hh"
#include "../Sensors/Micro_switch.hh"
#include "../Sensors/odometry.hh"
#include "../controller/main_controller.hh"


/*! \brief controller loop 
 *
 * \param[in] all_struct main structure
 */


int main()
{
    BigStruct *all_struct = init_BigStruct();

    //Odometry init
    odometer_data *odo_data = new odometer_data;
    if(odo_init(all_struct,odo_data)!=0){
        printf("error odo");
    }
      
    //scanLidar(all_struct);

    
    
    std::thread scanThread(scanLidar, all_struct);
    std::thread controlThread(main_controller, all_struct); 
    std::thread time_thread(timeThread);
  

    controlThread.join();
    scanThread.join();

    //main_camera(all_struct);        // à lancer par le troisième thread

    
    return 0;
}

/*! \brief last controller operations (called once)
 *
 * \param[in] all_struct controller main structure
 */
void controller_finish(BigStruct *all_struct)
{   
    close(all_struct->fd1);
    close(all_struct->fd0);

    free_BigStruct(all_struct);
}


void timeThread(BigStruct *all_struct) {
    time_t game = 180;
    while (all_struct->strat->state != END_STATE) {
        // Obtenez le temps actuel
        time_t local_time = time(NULL);
        
        // Verrouiller la section critique pour accéder à la variable partagée
        all_struct->time_mutex.lock();
        all_struct->current_time = local_time;
        all_struct->elapsed_time = difftime(all_struct->current_time, all_struct->start_time);
        if (all_struct->elapsed_time > game)
        {
            all_struct->strat->state = END_STATE;
        }
        
        all_struct->time_mutex.unlock();

        
        // Attendre un certain temps (par exemple, 1 seconde) avant de vérifier à nouveau le temps
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}
