#pragma once
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include <cstdio>

#include <cmath>

class Intake{
    private:
        double target_velocity;
        double torque;
        int counter;
        bool truTorque;
        int counter2;
        bool jammed;
        

    public:
    double maxTorque = .35f;
    Intake();

    void set_velocity(double _target_velocity) {target_velocity=_target_velocity;}
    double get_velocity();

    void update(double _torque);
 };