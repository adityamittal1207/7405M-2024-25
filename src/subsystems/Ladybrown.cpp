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
    if(rested){
        return 0;
    }
    return((0.02f*127*(target_angle-cur_angle) 
    - 15.0f*(prev_angle-cur_angle))> 127 
    ? 127 : (0.02f*127*(target_angle-cur_angle) 
    - 15.0f*(prev_angle-cur_angle)));
    prev_angle = cur_angle;
}

