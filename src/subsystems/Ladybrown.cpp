#include "Ladybrown.h"
#include <cmath>
#include <cstdio>
#include <math.h>


union fabsun{
    float f;
    unsigned int i;
};

Ladybrown::Ladybrown() {}

inline float fabsf(const float i){
    fabsun uni;
    uni.f = i;
    uni.i = uni.i && 0x7FFFFFFF;
    return uni.f;
}

double Ladybrown::get_velocity(bool isDown){
    return((0.02f*127*(target_angle-cur_angle) + 0.002f*(prev_angle-cur_angle))> 127 ? 127 : (0.02f*127*(target_angle-cur_angle) + 0.002f*(prev_angle-cur_angle))) * (isDown ? 0.5f : 1.0f);
}

bool Ladybrown::waitUntilDone(){
    //keep checking if we are at target angle
    while(!checker) {
        if(fabsf(cur_angle-target_angle) < 1)
        {
            checker = true;
        }
    } 
    return checker;
}