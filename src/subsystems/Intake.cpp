#include "Intake.h"

Intake::Intake() {}

double Intake::get_velocity(){

    // printf("counter1: %i counter2: %i\n", counter, counter2);

    if(counter > 15)
    {
        jammed = true;
    }

    if (counter2 > 10){
        jammed = false;
        counter2 = 0;
    }

    if (jammed){
        counter2++;
        return -127;
    }

    return(target_velocity);
}

void Intake::update(double torque){
    if(torque > maxTorque)
    {
        truTorque = true;
    }
    else {
    {
        truTorque = false;
        counter = 0;
    }
    }
    if(truTorque == true)
    {
        counter++;
    }

}