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



        } else { 
            printf("Error: wrong expected type");
            return DEFAULT;
        }

    }
}