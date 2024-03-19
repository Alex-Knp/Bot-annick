#include <iostream>


// Define the possible states as a plain enum
enum States {
    Idle,
    picking_pots1,
    picking_pots2,
    picking_plants,
    finished
};

// Function prototypes
void handleIdleState(State& current_state);
void handlePickingPots1State(State& current_state);
void handlePickingPots2State(State& current_state);
void handlePickingPlantsState(State& current_state);
void handleFinishedState(State& current_state);
void changeState(State& current_state, State new_state);

