#include <iostream>
using namespace std;

float calc_aero_torque(float angle_of_attack){
    const int c = 1;
    return c * angle_of_attack;
}