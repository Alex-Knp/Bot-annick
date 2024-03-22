#include "IR.hh"

// Variable globale contenant la liste de points
std::vector<Point> globalPoints = {{2336, 2.0}, {2014, 2.5}, {1818, 3.0}, {1612, 3.5}, {1458, 4.0}, {1312, 4.5}, {1211, 5.0}, {1114, 5.5}, {1041, 6.0}, {968, 6.5}, {894, 7.0}, {842, 7.5}, {792, 8.0}, {743, 8.5}, {696, 9.0}, {671, 9.5}, {621, 10.0}, {596, 10.5}, {546, 11.0}, {522, 11.5}, {495, 12.0}, {471, 12.5}, {469, 13.0}, {445, 13.5}, {420, 14.0}, {395, 14.5}, {390, 15.0}};

int simple_IR_ask(int fd, int IR_id){
    // Function use for testing the IR sensor
    // fd : file descriptor of the SPI communication

    uint8_t IR_message[5] = {0x00, 0x00, 0x00, 0x00, 0x00};

    switch(IR_id){
        case 1:
            IR_message[0] = 0x3f;
            break;
        case 2:
            IR_message[0] = 0x4f;
            break;
        case 3:
            IR_message[0] = 0x5f;
            break;
        case 4:
            IR_message[0] = 0x6f;
            break;
        default:
            printf("Error : wrong IR id, please enter a number between 1 and 4\n");
            return -1;
    }

    spi_transfer(fd, IR_message, 5);
    int IR_value = IR_message[1] << 24 | IR_message[2] << 16 | IR_message[3] << 8 | IR_message[4];
    printf("IR value : %d\n", IR_value);
    return IR_value;
}

double IR_Wall_tracking(int IR_id){
    // Function to get the distance from the wall
    // IR_id : id of the micro switch to read the IR value from
    // id 1 = panel left, id 2 = panel right


    uint8_t IR_message[5] = {0x00, 0x00, 0x00, 0x00, 0x00};

    switch(IR_id){
        case 1:
            IR_message[0] = 0x3f;
            break;
        case 2:
            IR_message[0] = 0x4f;
            break;
        default:
            printf("Error : wrong IR id, please enter a number between 1 and 4\n");
            return -1;
    }

    int fd = spi_init_1();
    spi_transfer(fd, IR_message, 5);
    close(fd);

    int IR_value = IR_message[1] << 24 | IR_message[2] << 16 | IR_message[3] << 8 | IR_message[4];

    return interpolateLinear(IR_value);
}

// Function to perform linear interpolation
double interpolateLinear(int xi) {
    
    // Find the interval for interpolation
	int upper_bound = 0;
    while (globalPoints[upper_bound].x > xi){
        upper_bound++; 
	}

	Point P1 = globalPoints[upper_bound-1];
	Point P2 = globalPoints[upper_bound];
    
    // Perform linear interpolation
	double slope = (P2.y - P1.y) / (P2.x - P1.x);
    return P1.y + slope * (xi - P1.x);
}

int If_pot_IR(int fd){

    int trigger = 1470; // Value to trigger the IR sensor, if sensed value higher than this value, return 1
    
    uint8_t IR_message[5] = {0x00, 0x00, 0x00, 0x00, 0x00};
    IR_message[0] = 0x5f;

    spi_transfer(fd, IR_message, 5);
    int IR_value = IR_message[1] << 24 | IR_message[2] << 16 | IR_message[3] << 8 | IR_message[4];

    if(IR_value > trigger){
        return 1;
    }
    else{
        return 0;
    }
}

int If_plant_IR(int fd){

    int trigger = 1300; // Value to trigger the IR sensor, if sensed value higher than this value, return 1
    
    uint8_t IR_message[5] = {0x00, 0x00, 0x00, 0x00, 0x00};
    IR_message[0] = 0x5f;

    spi_transfer(fd, IR_message, 5);
    int IR_value = IR_message[1] << 24 | IR_message[2] << 16 | IR_message[3] << 8 | IR_message[4];

    if(IR_value > trigger){
        return 1;
    }
    else{
        return 0;
    }
}


// int main(){

//     int fd = spi_init_1();

//     while(true){
//         int result = If_pot_IR(fd); 
//         printf("Result : %d\n", result);
//     }
    
//     close(fd);
//     return 0;
// }