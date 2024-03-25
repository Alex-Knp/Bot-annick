
#include "localisation.hh"
#include <chrono>
#include <algorithm>

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif
std::mutex mtx;
/*
        {                                                      //////// thread lock mutex
        std::lock_guard<std::mutex> lock(mtx);                 //////// thread
        }*/


ILidarDriver* connectLidar(){

  ILidarDriver* lidar;
  IChannel* _channel;
	lidar = *createLidarDriver();
  _channel = (*createSerialPortChannel("/dev/ttyUSB0", 115200));
    if (SL_IS_OK((lidar)->connect(_channel))){
    	printf("Connected\n");

    	lidar->setMotorSpeed();
      //RplidarScanMode scanMode;
      //lidar->startScan(false, true, 0, &scanMode);
      return lidar;
    }
    printf("Connection failed\n");

    return NULL;

}

bool ctrl_c_pressed;
void ctrlc(int)
{
    ctrl_c_pressed = true;
}

void scanLidar(ILidarDriver* drv){

    lidar_data myLidarData;
    myLidarData.beaconsx[0] = 1950;
    myLidarData.beaconsx[1] = 1000;
    myLidarData.beaconsx[2] = 50;
    myLidarData.beaconsy[0] = 50;
    myLidarData.beaconsy[1] = 2950;
    myLidarData.beaconsy[2] = 50;
    myLidarData.theta_lidar_calibration = M_PI/2;

    std::vector<double> x_tab;
    std::vector<double> y_tab;
    std::vector<double> x_ref_tab;
    std::vector<double> y_ref_tab;

    //float mean = 1/2.0*(sqrt(pow((myLidarData.beaconsx[0]-myLidarData.beaconsx[1]),2.0)+pow(myLidarData.beaconsy[0]-myLidarData.beaconsy[1],2.0))+sqrt(pow((myLidarData.beaconsx[1]-myLidarData.beaconsx[2]),2.0)+pow((myLidarData.beaconsy[1]-myLidarData.beaconsy[2]),2.0)));
    

    sl_lidar_response_device_info_t devinfo;
    sl_result     op_result;

    op_result = drv->getDeviceInfo(devinfo);

        // change scan mode
    /*
    std::vector<LidarScanMode> scanModes;
    drv->getAllSupportedScanModes(scanModes); 
    drv->startScanExpress(0,scanModes[3].id); */

    signal(SIGINT, ctrlc);
    drv->startScan(0,1);
    

    

    int is = 0;
    // fetech result and print it out...
    //sl_lidar_response_measurement_node_hq_t closest_point;

    while (is<1000) {
        //auto start_time = std::chrono::high_resolution_clock::now();
        sl_lidar_response_measurement_node_hq_t nodes[8192];
        std::vector<Cluster> clusters;
        std::vector<Beacon> beacons;
        std::vector<Beacon> final_beacons;
        size_t   count = _countof(nodes);

        //closest_point.angle_z_q14 = 0;
        //closest_point.dist_mm_q2 = 100000;

        op_result = drv->grabScanDataHq(nodes, count);
        printf("count = %lu\n", count);


        if (SL_IS_OK(op_result)) {
            drv->ascendScanData(nodes, count);
            
            for (int pos = 0; pos < (int)count ; ++pos) {
/*
                if (closest_point->dist_mm_q2>=nodes[pos].dist_mm_q2 && nodes[pos].dist_mm_q2/4.0 != 0)
                    {
                        closest_point->dist_mm_q2 = nodes[pos].dist_mm_q2;
                        closest_point->angle_z_q14 = nodes[pos].angle_z_q14;
                        printf("closest point : %08.2f \n", closest_point->dist_mm_q2/4.0f);
                        printf("closest point angle : %03.2f \n", (closest_point->angle_z_q14* 90.f) / 16384.f);

                    }*/

            } 

            getClusters(clusters, nodes, count);
            /*
            for (size_t i = 0; i < clusters.size(); i++)
            {   printf("Cluster number : %ld \n", i);
                for (size_t j = 0; j < clusters[i].points.size(); j++)
                {
                    printf(" d: %08.2f ", clusters[i].points[j].dist_mm_q2/4.0f);
                    printf(" a: %03.2f ", (clusters[i].points[j].angle_z_q14* 90.f) / 16384.f);

                }
                
            }*/
            
            getBeacon(beacons, clusters);/*
            for (size_t i = 0; i < beacons.size(); i++)
            {

                printf("beacon %ld : %f, angle : %f \n", i, beacons[i].distance, beacons[i].angle);
            }*/
            getFinalBeacon(beacons, final_beacons);
   
            if (final_beacons.size() == 3) {
                lidar_update_position(*coord, myLidarData, final_beacons, x_tab, y_tab, x_ref_tab, y_ref_tab);
                printf("x = %f, y = %f, theta = %f\n", coord->x, coord->y, coord->theta*(180/M_PI));
            }
            else {
                printf("not enough beacons\n");
            }

            for (size_t i = 0; i < final_beacons.size(); i++)
            {
                printf("final_beacon %ld : %f, angle : %f \n", i, final_beacons[i].distance, final_beacons[i].angle);
            }
            printf("\n");
        }
                   //////////////// threads end of lock mutex

       if (ctrl_c_pressed){ 
          break;
        }
       is++;
    /*
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    std::cout << "Execution time: " << duration.count() << " microseconds" << std::endl;*/
    }


}
#define M_PI 3.14159265358979323846
double distance_btw_points(sl_lidar_response_measurement_node_hq_t p1, sl_lidar_response_measurement_node_hq_t p2) {
    //distance calculée avec formule du cos qui utilise les deux distances et la différence d'angle entre les deux points
    double dist = sqrt(pow(p1.dist_mm_q2/4.0,2) + pow(p2.dist_mm_q2/4.0,2) - 2*p1.dist_mm_q2/4.0*p2.dist_mm_q2/4.0*cos(  (p2.angle_z_q14* 90.f)/16384.f*(M_PI / 180.0) - (p1.angle_z_q14* 90.f)/16384.f*(M_PI / 180.0)  ));

    return dist;
}


void getClusters(std::vector<Cluster>& clusters, sl_lidar_response_measurement_node_hq_t* nodes, size_t count) {
    // On parcours tous les points et on les ajoute à un cluster si ils sont à une distance inférieure au rayon du cluster
    // realiser un clustering en sachant que les points sont classés par ordre croissant d'angle dans le tableau nodes

    float max_dist = 50.0; // distance maximal entre deux points pour qu'ils soient dans le même cluster
    for (size_t i = 0; i < count; i++) {
        
        if (i == 0) {
            clusters.push_back(Cluster{});
            clusters[0].points.push_back(nodes[i]);
        }
        else {
            if (nodes[i].dist_mm_q2/4.0f != 0 && nodes[i].dist_mm_q2/4.0f < 3300) {
                if (distance_btw_points(clusters.back().points.back(), nodes[i]) > max_dist) {    
                    clusters.push_back(Cluster{});
                }
                clusters.back().points.push_back(nodes[i]);
            }
        }
    }
}

void getBeacon(std::vector<Beacon>& beacons, std::vector<Cluster> tab){
    int nb_clusters = tab.size();
    beacons.reserve(3);

    // pour chaque cluster test si c'est un beacon et si oui on calcule la distance et l'angle en prenant pour distance la distance du point le plus proche (+rayon beacon) et pour angle la moyenne des angles des points du cluster 
    for (int i = 0; i < nb_clusters; i++) {
        if (isAbeacon(tab[i], 50))                            // 50 = diam du beacon
        {       
            Beacon beacon;
            beacon.distance = tab[i].points[0].dist_mm_q2/4.0;
            beacon.angle = 0;
            for (size_t j = 0; j < tab[i].points.size(); j++) {
                if (tab[i].points[j].dist_mm_q2/4.0 < beacon.distance) {
                    beacon.distance = tab[i].points[j].dist_mm_q2/4.0;
                }
                beacon.angle += (tab[i].points[j].angle_z_q14* 90.f) / 16384.f;
            }
            beacon.distance += 25;                          // 25 = rayon du beacon
            beacon.angle = beacon.angle/tab[i].points.size();
            beacons.push_back(beacon);        
      }
    }
}
bool isAbeacon(Cluster clust, double diam) {

    double size = clust.points.size();
    double diam_error = 53;
    int min_points = 1;
    int max_points = 35;

    if (size < min_points || size > max_points) {
        return false;
    }
    if (clust.points.size() == 1){

        if (clust.points[0].dist_mm_q2/4.0f > 3500){
            return false;
        }
    }   
    else {
    
        if (std::abs(distance_btw_points(clust.points[0], clust.points[size-1]) - diam) > diam_error || clust.points[0].dist_mm_q2/4.0f  > 3500) {     // soit ça soit technique qui calcule l'angle attendu à une certaine distance de la balise
            return false;
        }
    }
    return true;
}

void getFinalBeacon(std::vector<Beacon>& beacons, std::vector<Beacon>& final_beacons) {
    // on parcours les beacons et on les compare 3 par 3
    // une fois les trois bon beacons trouvés on les ajoute au vecteur final de beacons
    double sum = 0;
    int size = beacons.size();
    double desired_distance = 8020  ;           // distance cumulée entre les trois beacons
    double error = 100;                         // marge d'erreur pour la distance souhaitée
    for (int i = 0; i < size-2; ++i) {
        for (int j = i + 1; j < size-1; ++j) {
            for (int k = j + 1; k < size; ++k) {
                sum = 0;

                // Vérifier si la distance entre ces trois beacons correspond à la distance souhaitée
                double distance1 = DistBtwBeacon(beacons[i], beacons[j]);
                double distance2 = DistBtwBeacon(beacons[i], beacons[k]);
                double distance3 = DistBtwBeacon(beacons[j], beacons[k]);

                if (((1850 <= distance1 && distance1 <= 1950) || (3000 <= distance1 && distance1 <= 3110)) &&
                    ((1850 <= distance2 && distance2 <= 1950) || (3000 <= distance2 && distance2 <= 3110)) &&
                    ((1850 <= distance3 && distance3 <= 1950) || (3000 <= distance3 && distance3 <= 3110))) {

                    sum = distance1 + distance2 + distance3;

                    if (fabs(desired_distance-sum) < error) {
                        final_beacons.push_back(beacons[i]);
                        final_beacons.push_back(beacons[j]);
                        final_beacons.push_back(beacons[k]);
                        //return;
                    }
                }
            }
        }
    }    
}

double DistBtwBeacon(Beacon beacon1, Beacon beacon2) {
    return sqrt(pow(beacon1.distance, 2) + pow(beacon2.distance, 2) - 2 * beacon1.distance * beacon2.distance * cos(((beacon1.angle - beacon2.angle))*(M_PI / 180.0)));
}

void lidar_update_position(RobotPosition &coord,lidar_data &data, std::vector<Beacon> &final_beacons,std::vector<double> &x_tab, std::vector<double> &y_tab, std::vector<double> &x_ref_tab, std::vector<double> &y_ref_tab){

    newsort(final_beacons);

    float theta = 0;
    float x_int = 0;
    float y_int = 0;

    float A = 2*data.beaconsx[1] - 2*data.beaconsx[0];
    float B = 2*data.beaconsy[1] - 2*data.beaconsy[0];
    float C = powf(final_beacons[0].distance,2) - powf(final_beacons[1].distance,2) - powf(data.beaconsx[0],2) + powf(data.beaconsx[1],2) - powf(data.beaconsy[0],2) + powf(data.beaconsy[1],2);
    float D = 2*data.beaconsx[2] - 2*data.beaconsx[1];
    float E = 2*data.beaconsy[2] - 2*data.beaconsy[1];
    float F = powf(final_beacons[1].distance,2) - powf(final_beacons[2].distance,2) - powf(data.beaconsx[1],2) + powf(data.beaconsx[2],2) - powf(data.beaconsy[1],2) + powf(data.beaconsy[2],2);
    x_int = (C*E - F*B) / (E*A - B*D);
    y_int = (C*D - A*F) / (B*D - A*E);



    for (int i = 0; i <3;i++) {
	    float x_err = (data.beaconsx[i])-(x_int);
	    float y_err = (data.beaconsy[i])-(y_int);
	    float theta_aligned = atan(y_err/x_err);
	    if(x_err < 0){
		    theta_aligned += M_PI;
	    }

        float theta_int = theta_aligned - final_beacons[i].angle*(M_PI / 180.0) - 2*M_PI - data.theta_lidar_calibration;
        while(theta_int < 0){
		    theta_int += 2*M_PI;
	    }
	    while(theta_int > 2*M_PI){
		    theta_int -= 2*M_PI;
	    }
        if (fabs(theta_int - 2*M_PI) < 0.035 || fabs(theta_int) < 0.035)
        {
            theta = theta_int;
            printf("break : theta = %f\n", theta_int);

            break;
        }
        
        theta += 1/3.0*theta_int;
        
        printf("theta = %f\n", theta_int);
    }
    //fill_points(x_int, y_int, 0, 0, x_tab, y_tab, x_ref_tab, y_ref_tab);
    //pthread_mutex_lock(&mutex);                 //////// thread lock mutex
        coord.x = x_int/1000.0;
        coord.y = y_int/1000.0;
        coord.theta = theta;
        coord.update_by = 1;
    //pthread_mutex_unlock(&mutex);                 //////// thread lock mutex

/*
    {
        std::lock_guard<std::mutex> lock(mutex);                 //////// thread
        coord.x = x_int/1000.0;
        coord.y = y_int/1000.0;
        coord.theta = theta;
        coord.update_by = 1;
    }*/
}

bool comparerParAngle(const Beacon& a, const Beacon& b, float theta) {
    float comp1 = a.angle - theta + 360.0;
    float comp2 = b.angle - theta + 360.0;
    while(comp1>360.0){
        comp1 -= 360.0;
    }
    while(comp2>360.0){
        comp2 -= 360.0;
    }
    return comp1 < comp2;
}

void disconnectLidar(ILidarDriver* lidar){
    lidar->stop();
    lidar->setMotorSpeed(0);
    delete lidar;
}

void newsort(std::vector<Beacon>& final_beacons) {
    float mean = 2670;
    double distance1 = DistBtwBeacon(final_beacons[0], final_beacons[1]);
    double distance2 = DistBtwBeacon(final_beacons[1], final_beacons[2]);
    if (distance1 >= mean)               // si distance1 = A et distance2 = B
    {
        if (distance2 <= mean)
        {
            std::swap(final_beacons[0], final_beacons[2]);
            std::swap(final_beacons[2], final_beacons[1]);
        }   
    } else {
        std::swap(final_beacons[0], final_beacons[1]);
        std::swap(final_beacons[1], final_beacons[2]);
    }   
};



void fill_points(double x, double y, double x_ref, double y_ref, std::vector<double> &x_tab, std::vector<double> &y_tab, std::vector<double> &x_ref_tab, std::vector<double> &y_ref_tab) {
    x_tab.push_back(x);
    y_tab.push_back(y); // Correction ici
    x_ref_tab.push_back(x_ref);
    y_ref_tab.push_back(y_ref); // Correction ici

    std::ofstream fd("donnees.csv");
    for (size_t i = 0; i < x_tab.size(); ++i) {
        fd << x_tab[i] << "," << y_tab[i] << "," << x_ref_tab[i] << "," << y_ref_tab[i] << "\n"; // Correction ici
    }
    fd.close();
}


void main_lidar(BigStruct *all_struct, ILidarDriver* drv)
{
    scanLidar(drv);
}