#pragma once

#include "./api.h"




class Ladybrown{
    private:
        double target_angle;
        double cur_angle;
        double prev_angle;

    public:

    Ladybrown();

    void set_angle(double _target_angle) {
        target_angle = _target_angle;
    }
    double get_velocity(bool isDown = false);

    void update(double _cur_angle) {
        prev_angle = cur_angle;
        cur_angle = _cur_angle/100.0f;}


    // void reset();
 };