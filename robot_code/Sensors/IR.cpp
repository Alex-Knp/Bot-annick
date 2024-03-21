#include "IR.hh"
#include "time.h"

// Variable globale contenant la liste de points
std::vector<Point> globalPoints = {{2336, 2.0}, {2014, 2.5}, {1818, 3.0}, {1612, 3.5}, {1458, 4.0}, {1312, 4.5}, {1211, 5.0}, {1114, 5.5}, {1041, 6.0}, {968, 6.5}, {894, 7.0}, {842, 7.5}, {792, 8.0}, {743, 8.5}, {696, 9.0}, {671, 9.5}, {621, 10.0}, {596, 10.5}, {546, 11.0}, {522, 11.5}, {495, 12.0}, {471, 12.5}, {469, 13.0}, {445, 13.5}, {420, 14.0}, {395, 14.5}, {390, 15.0}};

int micro_switch_tracking(int fd, int micro_switch_id){

// id 1 = panel left, id 2 = panel right, id 3 = pot = , id 4 = double pot

    uint8_t IR_message[5] = {0x00, 0x00, 0x00, 0x00, 0x00};

    switch(micro_switch_id){
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
        
    return IR_message[1] << 24 | IR_message[2] << 16 | IR_message[3] << 8 | IR_message[4];
}

// double cubicSplineInterpolation(int x) {
//     auto& points = globalPoints;

//     auto it = std::lower_bound(points.begin(), points.end(), x, [](const Point& p, int val) {
//         return p.x < val;
//     });

//     const Point& p1 = *(it - 1);
//     const Point& p2 = *it;

//     double dx = p2.x - p1.x;
//     double dy = p2.y - p1.y;
//     double t = static_cast<double>(x - p1.x) / dx;

//     // Interpolation par spline cubique
//     double t2 = t * t;
//     double t3 = t2 * t;
//     double s = (3 * t2 - 2 * t3);
//     return p1.y * (1 - s) + p2.y * s + (dx * dx / 6) * ((1 - t) * t3 * (1 - t) * dy + t * t3 * t * dy);
// }

// Function to perform linear interpolation
double interpolateLinear(const std::vector<Point>& points, int xi) {
    // Find the interval for interpolation
    auto it = std::upper_bound(points.begin(), points.end(), xi, [](int val, const Point& p) { return val < p.x; });
    
    // Perform linear interpolation
    auto p1 = *(it - 1);
    auto p2 = *it;
    double slope = (p2.y - p1.y) / (p2.x - p1.x);
    return p1.y + slope * (xi - p1.x);
}



int main(){

    int fd = spi_init_1();

    int micro_switch_state = micro_switch_tracking(fd, 2);
    printf("Micro switch %d state : %d\n", 1, micro_switch_state);
    double result = interpolateLinear(globalPoints, micro_switch_state);
    printf("Interpolated value : %f\n", result);

    return 0;
}