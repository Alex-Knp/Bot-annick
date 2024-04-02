#include "dynamixel_main.hh"


// gripper_goal_angle in degrees
// travel_speed in % of max speed
//goal angle in degrees
int left_dyn_set_angle(DynStruct* dyn, float gripper_goal_angle, float travel_speed){
    //set id
    dyn->Servo.is_id(0x05);

    float gripper_angle_error = gripper_goal_angle - dyn->gripper_current_angle;
    int direction = gripper_angle_error > 0 ? 1.0 : 0.0;

    printf("gripper_goal_angle: %f\n", gripper_goal_angle);
    printf("gripper_current_angle: %f\n", dyn->gripper_current_angle);
    printf("gripper_angle_error: %f\n", gripper_angle_error);

    //finding the sector of the goal angle
    int goal_sector;
    if(0 <= gripper_goal_angle && gripper_goal_angle <= 100){           goal_sector = 0; }
    else if(120 <= gripper_goal_angle && gripper_goal_angle <= 220){    goal_sector = 1; }
    else if(240 <= gripper_goal_angle && gripper_goal_angle <= 340){    goal_sector = 2; }
    else {
        std::cout << "Goal angle is not in the range of the gripper" << std::endl;
        return -1; }

    //finding the dynamixel final position (between 0 and 1023)
    int final_dynamixel_angle = (int) 1023*(1-(fmod(gripper_goal_angle, 120.0) * 3.0)/300);
    printf("final_dynamixel_angle: %d\n", final_dynamixel_angle);

    // setting the dynamixel in wheel mode and trying to stop near the goal position
    dyn->Servo.setTorqueEnable(0);
    dyn->Servo.setControlMode(1);
    dyn->Servo.setTorqueEnable(1);
    dyn->Servo.setSpeed((int) (travel_speed*1023.0/100.0 + 1024.0*direction));

    //#######################################################

    float correction_factor = 0.940;
    float asymetry_correction = 23.0;
    float gripper_travel_speed_in_degPerSec = travel_speed/100 * 684/3*correction_factor+ (1-direction)*asymetry_correction; // Convert to deg/sec
    usleep(abs(gripper_angle_error / gripper_travel_speed_in_degPerSec*1e6));

    dyn->Servo.setSpeed(0);

    // setting the dynamixel in joint mode and setting the goal position to adjust for the innacuracy of the wheel mode
    dyn->Servo.setTorqueEnable(0);
    dyn->Servo.setControlMode(2);
    dyn->Servo.setGoalPosition(final_dynamixel_angle);
    dyn->Servo.setTorqueEnable(1);

    dyn->gripper_current_angle = gripper_goal_angle;

    return 0; 
}

int right_dyn_set_angle(DynStruct* dyn, float gripper_goal_angle, float travel_speed){
    dyn->Servo.is_id(0x04);

    float gripper_angle_error = gripper_goal_angle - dyn->gripper_current_angle;
    int direction = gripper_angle_error > 0 ? 0.0 : 1.0;

    printf("gripper_goal_angle: %f\n", gripper_goal_angle);
    printf("gripper_current_angle: %f\n", dyn->gripper_current_angle);
    printf("gripper_angle_error: %f\n", gripper_angle_error);

    //finding the sector of the goal angle
    int goal_sector;
    if(0 <= gripper_goal_angle && gripper_goal_angle <= 100){           goal_sector = 0; }
    else if(120 <= gripper_goal_angle && gripper_goal_angle <= 220){    goal_sector = 1; }
    else if(240 <= gripper_goal_angle && gripper_goal_angle <= 340){    goal_sector = 2; }
    else {
        std::cout << "Goal angle is not in the range of the gripper" << std::endl;
        return -1; }

    //finding the dynamixel final position (between 0 and 1023)
    int final_dynamixel_angle = (int) 1023*(fmod(gripper_goal_angle, 120.0) * 3.0)/300;
    printf("final_dynamixel_angle: %d\n", final_dynamixel_angle);

    // setting the dynamixel in wheel mode and trying to stop near the goal position
    dyn->Servo.setTorqueEnable(0);
    dyn->Servo.setControlMode(1);
    dyn->Servo.setTorqueEnable(1);
    dyn->Servo.setSpeed((int) (travel_speed*1023.0/100.0 + 1024.0*direction));

    //#######################################################

    float correction_factor = 0.940;
    float asymetry_correction = 23.0;
    float gripper_travel_speed_in_degPerSec = travel_speed/100 * 684/3*correction_factor+ (1.0-direction)*asymetry_correction; // Convert to deg/sec
    usleep(abs(gripper_angle_error / gripper_travel_speed_in_degPerSec*1e6));

    dyn->Servo.setSpeed(0);

    // setting the dynamixel in joint mode and setting the goal position to adjust for the innacuracy of the wheel mode
    dyn->Servo.setTorqueEnable(0);
    dyn->Servo.setControlMode(2);
    dyn->Servo.setGoalPosition(final_dynamixel_angle);
    dyn->Servo.setTorqueEnable(1);

    dyn->gripper_current_angle = gripper_goal_angle;

    return 0;
}


int left_dyn_init(DynStruct* left_dyn){
    left_dyn->Servo.is_id(0x05);
    // torque disable
    left_dyn->Servo.setTorqueEnable(0);
    //set joint mode
    left_dyn->Servo.setControlMode(2);
    //set position
    left_dyn->Servo.setGoalPosition(256);
    //set moving speed
    left_dyn->Servo.setSpeed(300);
    //torque enable
    left_dyn->Servo.setTorqueEnable(1);

    sleep(1);

    left_dyn->gripper_current_angle = 75;
    return 0;
}

int right_dyn_init(DynStruct* right_dyn){
    right_dyn->Servo.is_id(0x04);
    // torque disable
    right_dyn->Servo.setTorqueEnable(0);
    //set joint mode
    right_dyn->Servo.setControlMode(2);
    //set position
    right_dyn->Servo.setGoalPosition(767);
    //set moving speed
    right_dyn->Servo.setSpeed(300);
    //torque enable
    right_dyn->Servo.setTorqueEnable(1);

    sleep(1);

    right_dyn->gripper_current_angle = 75;
    return 0;
}



int main(){
        
    XL_320 Servo;

    Servo.verbose = true;

    DynStruct *left_dyn = new DynStruct;
    left_dyn->Servo = Servo;
    DynStruct *right_dyn = new DynStruct;
    right_dyn->Servo = Servo;

    left_dyn_init(left_dyn);
    right_dyn_init(right_dyn);
    printf("Init done\n");

    while(1){
        dyn_go_to(left_dyn, NULL, LEFT, STANDBY, 100);
        sleep(1);
        dyn_go_to(left_dyn, NULL, LEFT, PICKING, 100);
        sleep(1);
        dyn_go_to(left_dyn, NULL, LEFT, PLANT1, 100);
        sleep(1);
        dyn_go_to(left_dyn, NULL, LEFT, PLANT2, 100);
        sleep(1);
        dyn_go_to(left_dyn, NULL, LEFT, PLANT3, 100);
        sleep(1);
        dyn_go_to(left_dyn, NULL, LEFT, STANDBY, 100);

        //same for the right dynamixel
        dyn_go_to(NULL, right_dyn, RIGHT, STANDBY, 100);
        sleep(1);
        dyn_go_to(NULL, right_dyn, RIGHT, PICKING, 100);
        sleep(1);
        dyn_go_to(NULL, right_dyn, RIGHT, PLANT1, 100);
        sleep(1);
        dyn_go_to(NULL, right_dyn, RIGHT, PLANT2, 100);
        sleep(1);
        dyn_go_to(NULL, right_dyn, RIGHT, PLANT3, 100);
        sleep(1);
        dyn_go_to(NULL, right_dyn, RIGHT, STANDBY, 100);
        
    }
    dyn_go_to(left_dyn, NULL, LEFT, PLANT1, 100);
    sleep(100);

    return 0;
}

int dyn_go_to(DynStruct *left_dyn,DynStruct *right_dyn, side side, Angle angle, int speed){
    //left side
    if(side == LEFT){
        switch (angle)
        {
        case PICKING: //pick up
            left_dyn_set_angle(left_dyn, 39, speed);
            break;
        case PLANT1: //plant 1
            left_dyn_set_angle(left_dyn, 90.5, speed);
            break;
        case PLANT2: //plant 2
            left_dyn_set_angle(left_dyn, 186, speed);
            break;
        case PLANT3: //plant 3
            left_dyn_set_angle(left_dyn, 245, speed);
            break;        
        default: //standby
            left_dyn_set_angle(left_dyn, 78, speed);
            break;
        }
    }
    //right side
    else{
        switch (angle)
        {
        case PICKING: //pick up
            right_dyn_set_angle(right_dyn, 39, speed);
            break;
        case PLANT1: //plant 1
            right_dyn_set_angle(right_dyn, 90.5, speed);
            break;
        case PLANT2: //plant 2
            right_dyn_set_angle(right_dyn, 186, speed);
            break;
        case PLANT3: //plant 3
            right_dyn_set_angle(right_dyn, 245, speed);
            break;        
        default: //standby
            right_dyn_set_angle(right_dyn, 78, speed);
            break;
        }
    }

    return 0;
}
