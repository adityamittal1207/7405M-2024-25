#include "Intake.h"

Intake::Intake() {}

void Intake::update(double torque){
    if(torque > maxTorque){
        detect_jam_counter++;
    }
    else {
        detect_jam_counter = 0;
    }
}


double Intake::get_velocity(){

    jammed = detect_jam_counter > 15;

    if (outtake_duration_counter > 20){
        jammed = false;
        outtake_duration_counter = 0;
    }

    if (jammed){
        outtake_duration_counter++;
        return -127.0;
    }

    return target_velocity;
}



