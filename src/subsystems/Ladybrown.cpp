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
    printf("cur %f target %f \n", cur_angle, target_angle);
    return (((p*127*(target_angle-cur_angle) 
    + 0.00*(prev_angle-cur_angle))> 127 
    ? 127 : (p*127*(target_angle-cur_angle) 
    + 0.00*(prev_angle-cur_angle))) * (isDown ? 0.5f : 1.0f));
}

