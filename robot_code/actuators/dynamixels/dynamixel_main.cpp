#include "dynamixel_main.hh"




// gripper_goal_angle in degrees
// travel_speed in % of max speed
int left_dyn_set_angle(DynStruct* dyn, float gripper_goal_angle, float travel_speed){

    //goal angle in degrees

    float gripper_angle_error = gripper_goal_angle - dyn->gripper_current_angle;
    int direction = gripper_angle_error > 0 ? 1 : -1;
    int goal_sector;

    if(0 <= gripper_goal_angle && gripper_goal_angle <= 100){
        goal_sector = 0;        
    }
    else if(120 <= gripper_goal_angle && gripper_goal_angle <= 220){
        goal_sector = 1;
    }
    else if(240 <= gripper_goal_angle && gripper_goal_angle <= 340){
        goal_sector = 2;
    }
    else{
        std::cout << "Goal angle is not in the range of the gripper" << std::endl;
        return -1;
    }

    float final_dynamixel_angle = fmod(gripper_goal_angle, 120.0) * 3.0;

    //IL FAUT ENCORE CALCULER LA VRAIE CONVERSION
    float gripper_travel_speed_in_degPerSec = travel_speed/100 * 228; // Convert to rad/s

    // trorque disable
    dyn->Servo.setTorqueEnable(0);
    //set wheel mode
    dyn->Servo.setControlMode(1);
    //set speed
    dyn->Servo.setSpeed(travel_speed*1023/100 + 1024*direction);
    //torque enable
    dyn->Servo.setTorqueEnable(1);
    sleep(gripper_angle_error / gripper_travel_speed_in_degPerSec);
    dyn->Servo.setSpeed(0);

    // trorque disable
    dyn->Servo.setTorqueEnable(0);
    //set joint mode
    dyn->Servo.setControlMode(0);
    //set speed
    dyn->Servo.setGoalPosition(final_dynamixel_angle/300*1023);
    //torque enable
    dyn->Servo.setTorqueEnable(1);

    while(dyn->Servo.isMoving() == 1){}

    dyn->gripper_current_angle = 120*goal_sector + final_dynamixel_angle/3.0;

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
    XL_320 left_Servo;
    //XL_320 right_Servo;
    left_Servo.verbose = true;
    //right_Servo.verbose = true;
    left_dyn->Servo = left_Servo;
    //right_dyn->Servo = right_Servo;

    // trorque disable
    left_dyn->Servo.setTorqueEnable(0);
    //set joint mode
    left_dyn->Servo.setControlMode(0);
    //set speed
    left_dyn->Servo.setGoalPosition(0);
    //torque enable
    left_dyn->Servo.setTorqueEnable(1);

    while(left_dyn->Servo.isMoving() == 1){}


    home(left_dyn);
    //home(right_dyn);

    left_dyn->gripper_current_angle = 0;
    left_dyn->current_sector = 0;
    //right_dyn->gripper_current_angle = 0;
    //right_dyn->current_sector = 0;
    return 0;
}

int main(){
    DynStruct *left_dyn = new DynStruct;
    //DynStruct *right_dyn = new DynStruct;
    dyn_init(left_dyn);//,right_dyn);
    left_dyn_set_angle(left_dyn, 50, 10);
    return 0;

}