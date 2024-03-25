#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "../sdk/include/sl_lidar_driver.h"
#include "../sdk/include/sl_lidar.h"

#include "position_controller_cpp.h"

using namespace sl;
coordinates* coord = nullptr;
sl_lidar_response_measurement_node_hq_t* closest_point = nullptr;


int main(int argc, const char * argv[]){
  coord = new coordinates;
  coord->theta = 0;
  coord->x = 450;
  coord->y = 450;

  closest_point = new sl_lidar_response_measurement_node_hq_t; 
  closest_point->angle_z_q14 = 0;
  closest_point->dist_mm_q2 = 100000;

  ILidarDriver *lidar;
	lidar = connectLidar();

      /*if (pthread_mutex_init(&mutex, NULL) != 0) {
        perror("Erreur lors de l'initialisation du mutex");
        return 1;
    }*/

  std::thread scanThread(scanLidar, lidar);
  //std::thread controlThread(robot_controller); 
  

  //controlThread.join();
  scanThread.join();
 

	disconnectLidar(lidar);
  delete coord;
  delete closest_point;
  return 0;
}
