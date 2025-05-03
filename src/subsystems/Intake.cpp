#include "Intake.h"

Intake::Intake() {}

void Intake::update(double torque){
    printf("%f \n", torque);
    if(torque > 0.5f){
        detect_jam_counter = 30;
    }
}


double Intake::get_velocity(){

    if(detect_jam_counter > 0){
        detect_jam_counter--;
        printf("a");
        return -127;
    }
    // if(releasedMogo){
    //     --releasedMogo;
    //     return -127.0;
    // }

    return target_velocity;
}



