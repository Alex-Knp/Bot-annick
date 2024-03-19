#include "main_FSM.h"

int main_FSM(robot_state* current_robot_state) {
    States current_state = Idle;

    // Main loop
    while (true) {
        switch (current_state) {
            case Idle:
                handleIdleState(current_state);
                break;
            case picking_pots1:
                handlePickingPots1State(current_state);
                break;

            case picking_pots2:
                handlePickingPots2State(current_state);
                break;
            
            case picking_plants:
                handlePickingPlantsState(current_state);
                break;

            case finished:
                handleFinishedState(current_state);
                break;
        }
    }

    return 0;
}

void handleIdleState(State& current_state) {
    // Check conditions to change state
    if (/* condition to start moving */) {
        changeState(current_state, Moving);
    }
}

void handlePickingPots1State(State& current_state) {
    // Check conditions to change state
    if (/* condition to start picking */) {
        changeState(current_state, picking_pots2);
    }
}

void handlePickingPots2State(State& current_state) {
    // Check conditions to change state
    if (/* condition to start picking */) {
        changeState(current_state, picking_plants);
    }
}

void handlePickingPlantsState(State& current_state) {
    // Check conditions to change state
    if (/* condition to start picking */) {
        changeState(current_state, finished);
    }
}

void handleFinishedState(State& current_state) {
    // Check conditions to change state
    if (/* condition to start picking */) {
        changeState(current_state, Idle);
    }
}


void changeState(RobotState& current_state, RobotState new_state) {
    // Here you can handle transitions
    current_state = new_state;
}
