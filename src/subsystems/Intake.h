#pragma once
#include "main.h"

class Intake{
    private:
        double target_velocity;
        int detect_jam_counter;
        int outtake_duration_counter;
        bool jammed;

        
        

    public:
    int releasedMogo = 0;
    double maxTorque = .35f;
    Intake();

    void set_velocity(double _target_velocity) {target_velocity=_target_velocity;}
    double get_velocity();

    void update(double torque);
};



