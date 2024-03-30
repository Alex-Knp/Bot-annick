/* #include "position_controller_cpp.h"

void update_position_encoders(struct coordinates *prev_known_coord, struct odometer_data *odo_data) 
{
    struct coordinates new_coord;
    float odo_wheel_radius = 0.0228;
    uint8_t TEST_MESSAGE[] = {0xff, 0xff, 0xff, 0xff};

    odo_data->left_prev = odo_data->left_tics;
    odo_data->right_prev = odo_data->right_tics;

    spi_ask(0x20, TEST_MESSAGE, odo_data->odo_right,4);
    spi_ask(0x21, TEST_MESSAGE, odo_data->odo_left ,4);

    odo_data->left_tics = -(odo_data->odo_left[1] << 24 | odo_data->odo_left[2] << 16 | odo_data->odo_left[3] << 8 | odo_data->odo_left[4]);
    odo_data->right_tics = odo_data->odo_right[1] << 24 | odo_data->odo_right[2] << 16 | odo_data->odo_right[3] << 8 | odo_data->odo_right[4];

    float L_dist = 2*PI*(odo_data->left_tics-odo_data->left_prev)/8192*odo_wheel_radius;
    float R_dist = 2*PI*(odo_data->right_tics-odo_data->right_prev)/8192*odo_wheel_radius;

    float b = 0.236; //wheel spacing in meters
    float delta_s = (R_dist + L_dist)/2;
    float delta_theta = (R_dist - L_dist)/b;

    new_coord.x = prev_known_coord->x + delta_s*cos(prev_known_coord->theta + delta_theta/2);
    new_coord.y = prev_known_coord->y + delta_s*sin(prev_known_coord->theta + delta_theta/2);
    new_coord.theta = prev_known_coord->theta + delta_theta;

    *prev_known_coord = new_coord;
};

void update_encoders(struct encoder_data *encoder_data){

    uint8_t TEST_MESSAGE[] = {0xff, 0xff, 0xff, 0xff};

    spi_ask(0x10, TEST_MESSAGE, encoder_data->enc_right, 4);
    spi_ask(0x11, TEST_MESSAGE, encoder_data->enc_left, 4);

    uint16_t left_speed = encoder_data->enc_left[3] << 8 | encoder_data->enc_left[4];
    uint16_t right_speed = encoder_data->enc_right[3] << 8 | encoder_data->enc_right[4];
    uint16_t dir_left =  1-(encoder_data->enc_left[1] << 8 | encoder_data->enc_left[2]); // 1 = forward, 0 = reverse
    uint16_t dir_right =  encoder_data->enc_right[1] << 8 | encoder_data->enc_right[2]; //1 = forward, 0 = reverse


    //convert the data from the encoder to rad/s
    encoder_data->L_speed = (-1+2*dir_left)*11*PI*left_speed/62;          // 62 tick correspond à la vitesse max qui est de 330rpm = 5,5tr/s = 11pi rad/s
    encoder_data->R_speed = (-1+2*dir_right)*11*PI*right_speed/62;
}

float PI_motor_controller(float omega_in, float omega_ref, float* integral_term, time_t previous_time){

    float controller_time_constant = 0.3; //defines the preciseness of the controller, found by trial and error

    float mass = 8; //minibot mass [kg]
    float wheel_radius = 0.03; //wheel radius [m]

    //motor caracteristics
    float max_voltage = 24;
    float K = 1;    //power electronics gain

    float k_phi = 0.358;
    float R_a = 2.143;  //armature resistance    
    float K_v = 1.26e-3;
    float J_r = mass*wheel_radius*wheel_radius/2; //rotor inertia , inertie roue et moteur negligeable devant l'inertie du robot, on converti l'inertiel inéaire du robot en inertie rotationnelle
    float tau_m = J_r / K_v;

    //controller gains
    float Ki = (R_a * K_v)/(K*k_phi*controller_time_constant);
    float Kp = tau_m*Ki;

    float error = omega_ref - omega_in;
    
    time_t current_time;
    time(&current_time);
    float time_interval = difftime(current_time,previous_time);

    *integral_term += error * time_interval;

    //anti-windup
    if (*integral_term > max_voltage / Ki) {
        *integral_term = max_voltage / Ki;
    } else if (*integral_term < -max_voltage / Ki) {
        *integral_term = -max_voltage / Ki;
    }

    float output_voltage = K*(Kp*error + Ki*(*integral_term) + k_phi*omega_ref/K);
    
    //voltage limiter
    if (output_voltage > max_voltage) {
        output_voltage = max_voltage;
    } else if (output_voltage < -max_voltage) {
        output_voltage = -max_voltage;
    }
    
    previous_time = current_time;

    return output_voltage;
}

int position_controller(struct wheels_speed *wheels_speed,struct coordinates *curr_pos, struct coordinates *goal_pos){
    int arrived = 0;

    float K_dist = 10.0;
    float K_angle = 5.0;

    float x_err = goal_pos->x - curr_pos->x;
    float y_err = goal_pos->y - curr_pos->y;
    float dist_err = sqrt(x_err*x_err + y_err*y_err);
    float theta_goal = atan(y_err/x_err);

    if (x_err < 0){
        theta_goal += PI;
    }
    //printf("dist_err = %f\n",dist_err);

    float theta_err = theta_goal - curr_pos->theta;

    //If close enough from the goal position, the error is set to zero so the robot stops and
    // the goal angle is set to the wanted final angle.
    if(dist_err < 0.04){
        dist_err = 0;
        arrived = 1;
        if(goal_pos->theta != -1){
            theta_goal = goal_pos->theta;
            arrived = 0;
            if(abs(theta_goal - curr_pos->theta)<0.03){
                theta_err = 0;
                arrived = 1;
            }
        }
    }
    else{
        arrived = 0;
        dist_err = 1.0;
    }
    if(theta_err>PI){
        theta_err -= 2*PI;
    } else if(theta_err<-PI) {
        theta_err += 2*PI;
    }

    //printf("theta_err = %f\n",theta_err);

    //In this block, the distance gain is set to zero if the goal point is behind the robot so the
    // robot turns aroud on a point, and the gain is adjusted relatively to the angle error so the robot
    // prioritize aligning with the target before going full speed.
    if(abs(theta_err) > PI/2){
        K_dist = 0;
    } else {
        K_dist = K_dist*cos(theta_err);
    }


    wheels_speed->left_wheel_speed = K_dist*dist_err - K_angle*theta_err;
    wheels_speed->right_wheel_speed = K_dist*dist_err + K_angle*theta_err;//invert this speed because the angular speed has to be negative (clockwise) when the robot goes forward

    return arrived;
}


int robot_controller(){
    void *null_ptr;
    printf("started\n");
    //delay(5000);
    spi_init();
    init_motors();
    
    //odometers initialization
    float odo_wheel_radius = 0.0228; 

    struct odometer_data *odo_data = new struct odometer_data;

    uint8_t TEST_MESSAGE[] = { 0xff, 0xff, 0xff, 0xff};

    spi_ask(0x20, TEST_MESSAGE, odo_data->odo_right,4);
    spi_ask(0x21, TEST_MESSAGE, odo_data->odo_left ,4);

    odo_data->left_tics = -(odo_data->odo_left[1] << 24 | odo_data->odo_left[2] << 16 | odo_data->odo_left[3] << 8 | odo_data->odo_left[4]);
    odo_data->right_tics = odo_data->odo_right[1] << 24 | odo_data->odo_right[2] << 16 | odo_data->odo_right[3] << 8 | odo_data->odo_right[4];

    //encoders initialization
    struct encoder_data *enc_data = new struct encoder_data;

    //position initialization
    struct coordinates *current_position = new struct coordinates;
    current_position->x = 0;
    current_position->y = 0;
    current_position->theta = PI/2;

    //PI controller intialization
    struct wheels_speed *wheels_speed = new struct wheels_speed;
    float *integral_term_left = new float;
    float *integral_term_right = new float;
    time_t PI_previous_time_left;
    time_t PI_previous_time_right;
    time_t init_time;
    time_t current_time;

    *integral_term_left = 0;
    *integral_term_right = 0;
    time(&PI_previous_time_left);
    time(&PI_previous_time_right);
    time(&init_time);
    time(&current_time);

    //postiton controller initialization
    struct coordinates *goal_pos = new struct coordinates;
    float goal_pos_list_x[5] = {1.5,1.5,0.5,1.5,0.5};
    float goal_pos_list_y[5] = {1.1,2.1,2.1,2.1,0.5};
    float goal_pos_list_theta[5] = {-1,-1,-1,-1,-1};
    int waypoint = 0;
    int arrived = 0;

    int print_counter = 0;

    while(!coord->update_by){printf("wait\n");}
    if(coord->update_by == 1){coord->update_by = 0;}

    while(!button_ask()){
        /*
        {
        std::lock_guard<std::mutex> lock(mutex);
        if(coord->update_by){
            current_position->x = coord->x;
            current_position->y = coord->y;
            current_position->theta = coord->theta;
            current_position->update_by = coord->update_by;
            update_encoders(enc_data);
            coord->update_by = 0;
        }
        } // a commenter

        //pthread_mutex_lock(&mutex);
        if(coord->update_by){
            current_position->x = coord->x;
            current_position->y = coord->y;
            current_position->theta = coord->theta;
            current_position->update_by = coord->update_by;
            update_encoders(enc_data);
            coord->update_by = 0;
            printf("Lidar updated the position : x = %f, y = %f, theta = %f°\n",current_position->x,current_position->y,(current_position->theta)*180/PI);
        }
        //pthread_mutex_unlock(&mutex); 

        goal_pos->x = goal_pos_list_x[waypoint];
        goal_pos->y = goal_pos_list_y[waypoint];
        goal_pos->theta = goal_pos_list_theta[waypoint];

        update_position_encoders(current_position, odo_data);
        update_encoders(enc_data);
        arrived = position_controller(wheels_speed,current_position,goal_pos);

        //printf("left control: %f, right control: %f\n",wheels_speed->left_wheel_speed,wheels_speed->right_wheel_speed);

        int left_ask = (int) 100/24*PI_motor_controller(enc_data->L_speed,wheels_speed->left_wheel_speed,integral_term_left,PI_previous_time_left);
        int right_ask = (int) 100/24*PI_motor_controller(enc_data->R_speed,wheels_speed->right_wheel_speed,integral_term_right,PI_previous_time_right);

        if ( (arrived == 1) && (waypoint == 4)){
            motor_ask(0,0);
            break;
            waypoint =-1;
        }
        if (arrived == 1){
            waypoint++;
        }        

        motor_ask(left_ask,right_ask);

        if(print_counter == 100){
            printf("x: %f, y: %f, theta: %f, ,waypoint: %d,arrived : %d\n\n", current_position->x, current_position->y,360/(2*PI)*current_position->theta,waypoint,arrived);
            print_counter = 0;
        }
        print_counter++;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    Motor_clear();
    delete odo_data;
    delete enc_data;
    delete wheels_speed;
    delete goal_pos;
    delete integral_term_left;
    delete integral_term_right;
    delete current_position;

    return 0;
}
 */

/* int main(){
    robot_controller();
    return 0;
} */