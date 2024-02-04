#include<iostream>

struct coord{
    //theta is equal to 0 when aligned with the x axis and positive when rotated counterclockwise
    float x;
    float y;
    float theta;
}

struct robot_state{
    bool left_V_micoswitch;
    bool right_V_micoswitch;
    bool left_gripper_homing_microswitch;
    bool right_gripper_homing_microswitch;

    float left_IR_distance;
    float right_IR_distance;
    float bottom_pot_IR_distance;
    float top_pot_IR_distance;

    float left_gripper_roatation_angle;
    float right_gripper_roatation_angle;
    float left_gripper_height;
    float right_gripper_height;
    bool left_gripper_extension;
    bool right_gripper_extension;
    bool left_gripper_closed;
    bool right_gripper_closed;

    struct coord desired_position;

    //TO ADD : led control, LCD screen control

}

void main_control_thread(){

    robot_state* current_robot_state = new robot_state();

    //the main FSM sets the desired states for all the actuators and the desired position for the robot
    main_FSM(current_robot_state);

    current postition get_lidar_postition();

    path_planning(current_robot_state);





}

