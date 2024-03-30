#include "picking_FSM.hh"

Angle drop_slot(Plant_Manager *PM, side side){
    Angle DEFAULT = PICKING;

    if(side == LEFT){
        if(PM->type_expected == POT){
            //verify there are no plants in the storage
            if(PM->storage.left.storage_3 == PLANT || PM->storage.left.storage_3 == POT_AND_PLANT || PM->storage.left.storage_3 == DOUBLE_POT_AND_PLANT){
                printf("Error: left emplacement 3 contains a plant");
                return DEFAULT;
            }
            if(PM->storage.left.storage_2 == PLANT || PM->storage.left.storage_2 == POT_AND_PLANT || PM->storage.left.storage_2 == DOUBLE_POT_AND_PLANT){
                printf("Error: left emplacement 2 contains a plant");
                return DEFAULT;
            }
            if(PM->storage.left.storage_1 == PLANT || PM->storage.left.storage_1 == POT_AND_PLANT || PM->storage.left.storage_1 == DOUBLE_POT_AND_PLANT){
                printf("Error: left emplacement 1 contains a plant");
                return DEFAULT;
            }            

            if(PM->storage.left.storage_3 == EMPTY){ return PLANT3; }
            if(PM->storage.left.storage_2 == EMPTY){ return PLANT2; }
            if(PM->storage.left.storage_1 == EMPTY){ return PLANT1; }
            //double potting
            if(PM->storage.left.storage_3 == POT){ return PLANT3; }
            if(PM->storage.left.storage_2 == POT){ return PLANT2; }
            if(PM->storage.left.storage_1 == POT){ return PLANT1; }
            printf("Error: left pot storage is full");
            return DEFAULT;
        } else if (PM->type_expected == PLANT){
            if(PM->storage.left.storage_3 == POT || PM->storage.left.storage_3 == DOUBLE_POT || PM->storage.left.storage_3 == EMPTY){ return PLANT3; }
            if(PM->storage.left.storage_2 == POT || PM->storage.left.storage_2 == DOUBLE_POT || PM->storage.left.storage_2 == EMPTY){ return PLANT2; }
            if(PM->storage.left.storage_1 == POT || PM->storage.left.storage_1 == DOUBLE_POT || PM->storage.left.storage_1 == EMPTY){ return PLANT1; }
            printf("Error: left plant storage is full");
            return DEFAULT;
        } else { 
            printf("Error: wrong expected type");
            return DEFAULT;
        }

    } else {
        if(PM->type_expected == POT){
            //verify there are no plants in the storage
            if(PM->storage.right.storage_3 == PLANT || PM->storage.right.storage_3 == POT_AND_PLANT || PM->storage.right.storage_3 == DOUBLE_POT_AND_PLANT){
                printf("Error: right emplacement 3 contains a plant");
                return DEFAULT;
            }
            if(PM->storage.right.storage_2 == PLANT || PM->storage.right.storage_2 == POT_AND_PLANT || PM->storage.right.storage_2 == DOUBLE_POT_AND_PLANT){
                printf("Error: right emplacement 2 contains a plant");
                return DEFAULT;
            }
            if(PM->storage.right.storage_1 == PLANT || PM->storage.right.storage_1 == POT_AND_PLANT || PM->storage.right.storage_1 == DOUBLE_POT_AND_PLANT){
                printf("Error: right emplacement 1 contains a plant");
                return DEFAULT;
            }            

            if(PM->storage.right.storage_3 == EMPTY){ return PLANT3; }
            if(PM->storage.right.storage_2 == EMPTY){ return PLANT2; }
            if(PM->storage.right.storage_1 == EMPTY){ return PLANT1; }
            //double potting
            if(PM->storage.right.storage_3 == POT){ return PLANT3; }
            if(PM->storage.right.storage_2 == POT){ return PLANT2; }
            if(PM->storage.right.storage_1 == POT){ return PLANT1; }
            printf("Error: right pot storage is full");
            return DEFAULT;
        } else if (PM->type_expected == PLANT){
            if(PM->storage.right.storage_3 == POT || PM->storage.right.storage_3 == DOUBLE_POT || PM->storage.right.storage_3 == EMPTY){ return PLANT3; }
            if(PM->storage.right.storage_2 == POT || PM->storage.right.storage_2 == DOUBLE_POT || PM->storage.right.storage_2 == EMPTY){ return PLANT2; }
            if(PM->storage.right.storage_1 == POT || PM->storage.right.storage_1 == DOUBLE_POT || PM->storage.right.storage_1 == EMPTY){ return PLANT1; }
            printf("Error: right plant storage is full");
            return DEFAULT;
        } else {
            printf("Error: wrong expected type");
            return DEFAULT;
        }
    }
}

float travel_height(Plant_Manager *PM, side side, Angle angle){
    float empty = 10;
    float pot = 6;
    float double_pot = 5;
    float plant = 4.5;
    float pot_and_plant = 2;
    float double_pot_and_plant = 0.5;

    float DEFAULT = double_pot_and_plant;

    if(side == LEFT){
        switch (angle)
        {
        case PLANT1:
            if(PM->storage.left.storage_1 == EMPTY){ return empty; }
            if(PM->storage.left.storage_1 == POT){ return pot; }
            if(PM->storage.left.storage_1 == DOUBLE_POT){ return double_pot; }
            if(PM->storage.left.storage_1 == PLANT){ return plant; }
            if(PM->storage.left.storage_1 == POT_AND_PLANT){ return pot_and_plant; }
            if(PM->storage.left.storage_1 == DOUBLE_POT_AND_PLANT){ return double_pot_and_plant; }
            break;
        case PLANT2:
            if(PM->storage.left.storage_2 == POT){ return pot; }
            if(PM->storage.left.storage_2 == DOUBLE_POT){ return double_pot; }
            if(PM->storage.left.storage_2 == PLANT){ return plant; }
            if(PM->storage.left.storage_2 == POT_AND_PLANT){ return pot_and_plant; }
            if(PM->storage.left.storage_2 == DOUBLE_POT_AND_PLANT){ return double_pot_and_plant; }
            break;
        case PLANT3:
            if(PM->storage.left.storage_3 == POT){ return pot; }
            if(PM->storage.left.storage_3 == DOUBLE_POT){ return double_pot; }
            if(PM->storage.left.storage_3 == PLANT){ return plant; }
            if(PM->storage.left.storage_3 == POT_AND_PLANT){ return pot_and_plant; }
            if(PM->storage.left.storage_3 == DOUBLE_POT_AND_PLANT){ return double_pot_and_plant; }
            break;
        default:
            return DEFAULT;
            break;
        }
    } else {
        switch (angle)
        {
        case PLANT1:
            if(PM->storage.right.storage_1 == EMPTY){ return empty; }
            if(PM->storage.right.storage_1 == POT){ return pot; }
            if(PM->storage.right.storage_1 == DOUBLE_POT){ return double_pot; }
            if(PM->storage.right.storage_1 == PLANT){ return plant; }
            if(PM->storage.right.storage_1 == POT_AND_PLANT){ return pot_and_plant; }
            if(PM->storage.right.storage_1 == DOUBLE_POT_AND_PLANT){ return double_pot_and_plant; }
            break;
        case PLANT2:
            if(PM->storage.right.storage_2 == POT){ return pot; }
            if(PM->storage.right.storage_2 == DOUBLE_POT){ return double_pot; }
            if(PM->storage.right.storage_2 == PLANT){ return plant; }
            if(PM->storage.right.storage_2 == POT_AND_PLANT){ return pot_and_plant; }
            if(PM->storage.right.storage_2 == DOUBLE_POT_AND_PLANT){ return double_pot_and_plant; }
            break;
        case PLANT3:
            if(PM->storage.right.storage_3 == POT){ return pot; }
            if(PM->storage.right.storage_3 == DOUBLE_POT){ return double_pot; }
            if(PM->storage.right.storage_3 == PLANT){ return plant; }
            if(PM->storage.right.storage_3 == POT_AND_PLANT){ return pot_and_plant; }
            if(PM->storage.right.storage_3 == DOUBLE_POT_AND_PLANT){ return double_pot_and_plant; }
            break;
        default:
            return DEFAULT;
            break;
        }
    }
}

int update_fill(Plant_Manager *PM, side side, Angle angle){
    if(side == LEFT){
        Fill previous_fill;
        if(angle == PLANT1){    previous_fill = PM->storage.left.storage_1; }
        else if(angle == PLANT2){    previous_fill = PM->storage.left.storage_2; }
        else if(angle == PLANT3){    previous_fill = PM->storage.left.storage_3; }
        else {
            printf("Error: wrong angle");
            return -1;
        }
        Fill new_fill;
        if(PM->type_expected == POT){
            if(previous_fill == EMPTY){ new_fill = POT; }
            else if(previous_fill == POT){ new_fill = DOUBLE_POT; }
            else {
                printf("Error: wrong previous fill");
                return -1;
            }
        } else if (PM->type_expected == PLANT) {
            if(previous_fill == EMPTY){ new_fill = PLANT; }
            else if(previous_fill == POT){ new_fill = POT_AND_PLANT; }
            else if(previous_fill == DOUBLE_POT){ new_fill = DOUBLE_POT_AND_PLANT; }
            else {
                printf("Error: wrong previous fill");
                return -1;
            }
        } else {
            printf("Error: wrong expected type");
            return -1;
        }
        if(angle == PLANT1){    PM->storage.left.storage_1 = new_fill; }
        else if(angle == PLANT2){    PM->storage.left.storage_2 = new_fill; }
        else if(angle == PLANT3){    PM->storage.left.storage_3 = new_fill; }
        else {
            printf("Error: wrong angle");
            return -1;
        }
        return 0;
    } else {
        Fill previous_fill;
        if(angle == PLANT1){    previous_fill = PM->storage.right.storage_1; }
        else if(angle == PLANT2){    previous_fill = PM->storage.right.storage_2; }
        else if(angle == PLANT3){    previous_fill = PM->storage.right.storage_3; }
        else {
            printf("Error: wrong angle");
            return -1;
        }
        Fill new_fill;
        if(PM->type_expected == POT){
            if(previous_fill == EMPTY){ new_fill = POT; }
            else if(previous_fill == POT){ new_fill = DOUBLE_POT; }
            else {
                printf("Error: wrong previous fill");
                return -1;
            }
        } else if (PM->type_expected == PLANT) {
            if(previous_fill == EMPTY){ new_fill = PLANT; }
            else if(previous_fill == POT){ new_fill = POT_AND_PLANT; }
            else if(previous_fill == DOUBLE_POT){ new_fill = DOUBLE_POT_AND_PLANT; }
            else {
                printf("Error: wrong previous fill");
                return -1;
            }
        } else {
            printf("Error: wrong expected type");
            return -1;
        }
        if(angle == PLANT1){    PM->storage.right.storage_1 = new_fill; }
        else if(angle == PLANT2){    PM->storage.right.storage_2 = new_fill; }
        else if(angle == PLANT3){    PM->storage.right.storage_3 = new_fill; }
        else {
            printf("Error: wrong angle");
            return -1;
        }
        return 0;
    }
}
