#include "odometry.hh"

void update_position_encoders( BigStruct *all_struct, odometer_data *odo_data) 
{
    float odo_wheel_radius = 0.02245963698;
    float b = 0.2939241998; //wheel spacing in meters

	int fd1 = spi_init_1();
	uint8_t left[5] = {0x10, 0x00, 0x00, 0x00, 0x00};    
	uint8_t right[5] = {0x20, 0x00, 0x00, 0x00, 0x00};
	spi_transfer(fd1, left, 5);
	spi_transfer(fd1, right, 5);

	int32_t left_value = left[1] << 24 | left[2] << 16 | left[3] << 8 | left[4];
	int32_t right_value = right[1] << 24 | right[2] << 16 | right[3] << 8 | right[4];

	close(fd1); 

    float L_dist = 2*PI*(left_value-odo_data->left_prev)/8192.0*odo_wheel_radius;
    float R_dist = 2*PI*(right_value-odo_data->right_prev)/8192.0*odo_wheel_radius;

    float delta_s = (R_dist + L_dist)/2.0;
    float delta_theta = (R_dist - L_dist)/b;

    all_struct->rob_pos->x += delta_s*cos(all_struct->rob_pos->theta + delta_theta/2);
    all_struct->rob_pos->y += delta_s*sin(all_struct->rob_pos->theta + delta_theta/2);
    all_struct->rob_pos->theta += delta_theta;

    odo_data->left_prev = left_value;
    odo_data->right_prev = right_value;
};

int odo_init(BigStruct* all_struct,odometer_data *odo_data){
    int fd1 = all_struct->fd1;
    all_struct->odo_data = odo_data;
    uint8_t left_init[5] = {0x10, 0x0A, 0x00, 0x0A, 0x00};    
    uint8_t right_init[5] = {0x20, 0x0A, 0x00, 0x0A, 0x00};
    spi_transfer(fd1, left_init, 5);
    spi_transfer(fd1, right_init, 5);
    
    odo_data->left_prev = left_init[1] << 24 | left_init[2] << 16 | left_init[3] << 8 | left_init[4];
    odo_data->right_prev = right_init[1] << 24 | right_init[2] << 16 | right_init[3] << 8 | right_init[4];

    return 0;
};

// int main(){
//     RobotPosition *prev_known_coord = new RobotPosition;
//     odometer_data *odo_data = new odometer_data;
//     odo_init(odo_data);
//     prev_known_coord->x = 0;
//     prev_known_coord->y = 0;
//     prev_known_coord->theta = 0;
//     prev_known_coord->last_t = 0;

//     while(1){
//         update_position_encoders(prev_known_coord, odo_data);
//         printf("x: %f, y: %f, theta: %f\n\n", prev_known_coord->x, prev_known_coord->y, prev_known_coord->theta*180/PI);
//     }
//     delete prev_known_coord;
//     delete odo_data;
//     return 0;
// }
