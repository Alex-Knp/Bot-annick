#include "dynamixel_main.hh"


// gripper_goal_angle in degrees
// travel_speed in % of max speed
//goal angle in degrees
int left_dyn_set_angle(DynStruct* dyn, float gripper_goal_angle, float travel_speed){

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



int home(DynStruct* dyn){
    // float homing_speed = 10; // % of the max speed
    // float homing_detection_torque = 5; //% of the max torque
    // float standby_angle = 220; // dynamixel degrees

    // // Set the speed of the dynamixel
    // setWheelMode(dyn->id);
    // setSpeed(homing_speed, dyn->id);
    // while(getTorque(dyn->id) < homing_detection_torque){}
    // setSpeed(0, dyn->id);

    // setJointMode(dyn->id);
    // setAngle(standby_angle, dyn->id);
    // dyn->current_sector = 1;
    // dyn->gripper_current_angle = standby_angle/3.0;

    // dyn->gripper_current_angle = dyn->homing_sector1_angle;

    return 0;
}

int dyn_init(DynStruct* left_dyn){//,DynStruct* right_dyn){


    // torque disable
    left_dyn->Servo.setTorqueEnable(0);
    //set joint mode
    left_dyn->Servo.setControlMode(2);
    //set position
    left_dyn->Servo.setGoalPosition(256);//256
    //set moving speed
    left_dyn->Servo.setSpeed(300);
    //torque enable
    left_dyn->Servo.setTorqueEnable(1);

    // printf("before while\n");
    // while(left_dyn->Servo.isMoving() == 1){
    //     sleep(0.01);
    // }
    // printf("after while\n");
    sleep(1);

    left_dyn->gripper_current_angle = 75;//75
    return 0;
}

int test(){
    
    XL_320 Servo;
    Servo.verbose = true;
    Servo.is_id(0x05);

    // // Test ping
    Servo.ping();

    Servo.setTorqueEnable(0);
    Servo.setControlMode(2);
    Servo.setSpeed(1023);
    Servo.setGoalPosition(0);
    Servo.setTorqueEnable(1);

    return 0;
}

int main(){
    // DynStruct *left_dyn = new DynStruct;
    // //DynStruct *right_dyn = new DynStruct;
    // dyn_init(left_dyn);//,right_dyn);
    // printf("Init done\n");
    // left_dyn_set_angle(left_dyn, 150, 50);
    //test();
        
    XL_320 left_Servo;
    left_Servo.verbose = true;
    left_Servo.is_id(0x05);

    DynStruct *left_dyn = new DynStruct;
    left_dyn->Servo = left_Servo;

    dyn_init(left_dyn);
    printf("Init done ############################################################################\n");

    // sleep(1);
    // left_dyn_set_angle(left_dyn, 120, 100);
    // sleep(1);
    // left_dyn_set_angle(left_dyn, 22, 70);

    while(1){
        dyn_go_to(left_dyn, NULL, 1, 4, 100);
        sleep(1);
        dyn_go_to(left_dyn, NULL, 1, 0, 100);
        sleep(1);
        dyn_go_to(left_dyn, NULL, 1, 1, 100);
        sleep(1);
        dyn_go_to(left_dyn, NULL, 1, 2, 100);
        sleep(1);
        dyn_go_to(left_dyn, NULL, 1, 3, 100);
        sleep(1);
    }

    dyn_go_to(left_dyn, NULL, 1, 1, 100);


    return 0;

}

int dyn_go_to(DynStruct *left_dyn,DynStruct *right_dyn, int side, int number, int speed){
    //left side
    if(side == 1){
        switch (number)
        {
        case 0: //pick up
            left_dyn_set_angle(left_dyn, 32, speed);
            break;
        case 1: //plant 1
            left_dyn_set_angle(left_dyn, 90.5, speed);
            break;
        case 2: //plant 2
            left_dyn_set_angle(left_dyn, 186, speed);
            break;
        case 3: //plant 3
            left_dyn_set_angle(left_dyn, 245, speed);
            break;        
        default: //standby
            left_dyn_set_angle(left_dyn, 78, speed);
            break;
        }
    }

    return 0;
}