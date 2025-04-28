#pragma once

#include "./api.h"

class Ladybrown{
    private:
        double target_angle;
        double cur_angle;
        double prev_angle;
        bool checker = false;
        bool rested = false;

    public:
    Ladybrown();

    void set_angle(double _target_angle) {
        rested = false;
        target_angle = _target_angle;
    }
    double get_velocity(bool isDown = false);
    bool waitUntilDone();

    void update(double _cur_angle) {
        
        cur_angle = _cur_angle/100.0f;
        prev_angle = cur_angle;
        }
        
    void rest(){
        rested = true;
    }
 };




