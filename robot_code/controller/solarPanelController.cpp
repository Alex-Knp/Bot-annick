#include "solarPanelController.hh"

panel_controller_struct* panel_stuct_init() {
    panel_controller_struct *panel_struct = new panel_controller_struct;
    panel_struct->previous_error = 0;
    panel_struct->previous_time = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());

    return panel_struct;
}

int left_panel_controller(panel_controller_struct *panel_struct ,float speed, float goal_distance){
    // This function is used to control the distance between the robot and the wall using the IR sensor
    // It will input the speed and distance from the wall at which the robot has to travel
    // It will output the speed of each wheel to maintain the distance from the wall

    float current_distance = IR_Wall_tracking(1);
    float error = goal_distance - current_distance;

    printf("current_distance : %f\t", current_distance);
    printf("goal_distance : %f\t", goal_distance);

    std::time_t now_time_t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    float time_difference = difftime(now_time_t, panel_struct->previous_time);
    float error_diff = error - panel_struct->previous_error;
    float derivative = error_diff / time_difference;

    printf("error : %f\t", error);
    printf("derivative : %f\t", derivative);

    float Kp = 0.05;
    float Kd = 1;
    float speed_correction = Kp * error - Kd * derivative;

    printf("speed_correction : %f\n\n", speed_correction);


    float max_derivative = 0.5;
    if (derivative > max_derivative && speed_correction < 0.0){
        speed_correction = 0;
    }
    else if (derivative < -max_derivative && speed_correction > 0.0){
        speed_correction = 0;
    }

    printf("speed_correction : %f\n\n", speed_correction);

    float left_speed = speed*(1.0 + speed_correction);
    float right_speed = speed*(1.0 - speed_correction);

    motor_ask(left_speed, right_speed);

    panel_struct->previous_error = error;

    return 0;
}

//same as for the left panel controller but for the right panel
int right_panel_controller(panel_controller_struct *panel_struct ,float speed, float goal_distance){
    float current_distance = IR_Wall_tracking(2);
    float error = goal_distance - current_distance;

    std::time_t now_time_t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    float time_difference = difftime(now_time_t, panel_struct->previous_time);
    float error_diff = error - panel_struct->previous_error;
    float derivative = error_diff / time_difference;

    float Kp = 0.5;
    float Kd = 0.5;
    float speed_correction = Kp * error - Kd * derivative;

    float left_speed = speed + speed_correction;
    float right_speed = speed - speed_correction;

    motor_ask(left_speed, right_speed);

    return 0;
}

int main(){
    printf("solarPanelController test start");
    panel_controller_struct *left_panel_struct = panel_stuct_init();


    while(1){
        left_panel_controller(left_panel_struct,6.0, 8.0);
    }
}