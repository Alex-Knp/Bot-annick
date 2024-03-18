#include <iostream>
#include <cmath>



struct DynStruct {
    float gripper_current_angle;
    float current_sector;
    uint8_t id;
};

int dyn_set_angle(DynStruct* dyn, float gripper_goal_angle, float gripper_travel_speed){

    //goal angle in degrees

    float gripper_angle_error = gripper_goal_angle - dyn->gripper_current_angle;

    if(0 <= gripper_goal_angle && gripper_goal_angle <= 100){
        dyn->current_sector = 1;        
    }
    else if(120 <= gripper_goal_angle && gripper_goal_angle <= 220){
        dyn->current_sector = 2;
    }
    else if(240 <= gripper_goal_angle && gripper_goal_angle <= 340){
        dyn->current_sector = 3;
    }
    else{
        std::cout << "Goal angle is not in the range of the gripper" << std::endl;
        return -1;
    }

    float final_dynamixel_angle = fmod(gripper_goal_angle, 120.0) * 3.0;

    //IL FAUT ENCORE CALCULER LA VRAIE CONVERSION
    float gripper_travel_speed_in_radPerSec = gripper_travel_speed * 0.0111111; // Convert to rad/s

    // Set the speed of the dynamixel
    setWheelMode(dyn->id);
    setSpeed(gripper_travel_speed*3.0, dyn->id);
    sleep(gripper_angle_error / gripper_travel_speed_in_radPerSec);
    setSPeed(0, dyn->id);

    setJointMode(dyn->id);
    setAngle(final_dynamixel_angle, dyn->id);

    dyn->gripper_current_angle = final_dynamixel_angle;

    return 0; 
}

int home(DynStruct* dyn){
    float homing_speed = 10; // % of the max speed
    float homing_detection_torque = 5; //% of the max torque
    float standby_angle = 220; // dynamixel degrees

    // Set the speed of the dynamixel
    setWheelMode(dyn->id);
    setSpeed(homing_speed, dyn->id);
    while(getTorque(dyn->id) < homing_detection_torque){}
    setSpeed(0, dyn->id);

    setJointMode(dyn->id);
    setAngle(standby_angle, dyn->id);
    dyn->current_sector = 1;
    dyn->gripper_current_angle = standby_angle/3.0;

    dyn->gripper_current_angle = dyn->homing_sector1_angle;

    return 0;
}