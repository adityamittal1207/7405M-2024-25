#include "Ladybrown.h"
#include <cstdio>


Ladybrown::Ladybrown() {}


double Ladybrown::get_velocity(bool isDown){
    printf("cur %f target %f \n", cur_angle, target_angle);
    return((0.02f*127*(target_angle-cur_angle) + 0.002f*(prev_angle-cur_angle))> 127 ? 127 : (0.02f*127*(target_angle-cur_angle) + 0.002f*(prev_angle-cur_angle))) * (isDown ? 0.5f : 1.0f);
}