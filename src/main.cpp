#include "main.h"
#include "lemlib/api.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include <cstdio>
#include <functional>
#include <valarray>

// #define six_ball 0
// #define far_safe_awp 1
// #define close_rush 2
// #define close_safe_awp 3

// ASSET(path_txt);

pros::Motor left_front_motor(14, pros::E_MOTOR_GEAR_BLUE, true);
pros::Motor left_center_motor(5, pros::E_MOTOR_GEAR_BLUE	, true);
pros::Motor left_back_motor(11, pros::E_MOTOR_GEAR_BLUE	, true);    
pros::Motor right_front_motor(2, pros::E_MOTOR_GEAR_BLUE	, false);
pros::Motor right_center_motor(1, pros::E_MOTOR_GEAR_BLUE	, false);
pros::Motor right_back_motor(20, pros::E_MOTOR_GEAR_BLUE	, false);

pros::Motor intake(19); //
pros::Motor intake2(15);

pros::Motor wallmotor(13);

// pros::Motor redirect(7); //

pros::Imu inertial_sensor(9);

pros::ADIDigitalOut clamp('H'); //backwings H, F
pros::ADIDigitalOut hangs('G');


pros::MotorGroup left_side_motors({left_front_motor, left_center_motor, left_back_motor});
pros::MotorGroup right_side_motors({right_front_motor, right_center_motor, right_back_motor});

pros::Controller master(pros::E_CONTROLLER_MASTER);

bool intaking = false;
bool outtaking = false;
int mechstage = 5;

lemlib::Drivetrain drivetrain {
        &left_side_motors, // left drivetrain motors
        &right_side_motors, // right drivetrain motors
        11.25, // track width
        3.25, // wheel diameter
        450, // wheel rpm
        0
};

// 3.5 in horizontal rot displacement

pros::Rotation wallrot(5, false); 

pros::Rotation horizontal_rot(10, false); // port 1, not reversed

lemlib::TrackingWheel horizontal_track(&horizontal_rot, lemlib::Omniwheel::NEW_2 , -3.6); // 0.6 -0.9

pros::Rotation vertical_rot(16, true); // port 1, not reversed

lemlib::TrackingWheel vertical_track(&vertical_rot, lemlib::Omniwheel::NEW_2,3.5); // 0.6 -0.9

// odometry struct
lemlib::OdomSensors sensors {
        &vertical_track, // vertical tracking wheel 1
        nullptr, // vertical tracking wheel 2
        // nullptr,
        &horizontal_track, // horizontal tracking wheel 1
        nullptr, // we don't have a second tracking wheel, so we set it to nullptr
        &inertial_sensor // inertial sensor
};

// forward/backward PID
lemlib::ControllerSettings lateralController(7, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              7, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// turning PID
lemlib::ControllerSettings angularController(6, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              39, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              10 // maximum acceleration (slew)
);
// lemlib::ControllerSettings angularController {
//         1.5, // kP
//         0,
//         40, // kD
//         0,
//         1, // smallErrorRange
//         100, // smallErrorTimeout
//         3, // largeErrorRange
//         500, // largeErrorTimeout
//         40 // slew rate
// };

lemlib::Chassis chassis(drivetrain, lateralController, angularController, sensors);

double shooter_coeff = 0.775;
bool hang_released = false;
int auton_running = 0;
int as = 0;
bool clamped = false;
bool macro = false;


void move(double power, double turn, bool swing=false) {
    int left = power + turn;
    int right = power - turn;

    if (swing && left < 0) {left = 0;}
    if (swing && right < 0) {right = 0;}

    left_front_motor = left;
    left_center_motor = left;
    left_back_motor = left;
    right_front_motor = right;
    right_center_motor = right;
    right_back_motor = right;
}

void move_drive(double power, double turn) {

    pros::lcd::print(2, "turnpower: %f", turn);

    int left = power + turn;
    int right = power - turn;

    // if (left < 0) left = left - 9;
    // else if (left > 0) left = left + 9;

    // if (right < 0) right = right - 9;
    // else if (right > 0) right = right + 9;


    left_front_motor = left;
    // left_center_motor = left;
    // left_back_motor = left;
    right_front_motor = right;
    // right_center_motor = right;
    // right_back_motor = right;
}


void screen() {
    // loop forever
    while (true) {
        lemlib::Pose pose = chassis.getPose(); // get the current position of the robot
        pros::lcd::print(0, "x: %f | y: %f", pose.x, pose.y, pose.theta); // print the x position
        pros::lcd::print(1, "H: %f", pose.theta); // print the x position
        // printf("x: %f | y: %f | H: %f | rot: %d \n", pose.x, pose.y, pose.theta, vertical_rot.get_position());
        pros::delay(10);
    }
}

static int dampen(int input) {
    /*
        double s = 40;
        double a = .60;
        double v = (127*a-127)/(-s*s+254*s-16129);
        double c = a - 2*v*s;
        double output;
        if (abs(input) < abs(s)) {
            output = a * input;
        }
        else {
            double x = abs(input);
            double y = -(s - x) * (c + v * (s + x)) + a * s;
            output = y * input / abs(input);
        }
    */
    double expo = 0.4;
    double output = 0;
    double in = input / 127.0;
    output = ((in * in * in) * 118 * expo) + in * 118 * (1 - expo);
    return (int)std::round(output);
}

void rotate_to(double targetHeading, double turnAcc, double maxSpeed, bool swing) {
    double theta = inertial_sensor.get_rotation();
    double curPosHeading = std::fmod(theta, 180.0) - 180.0 * std::round(theta / (360.0));
    double headingErr = targetHeading - curPosHeading;
    double errorsum = 0;
    double turnSpeed = 0;
    if (std::fabs(headingErr) > 180.0) { headingErr = headingErr > 0.0 ? headingErr - 360.0 : headingErr + 360.0; }

    int i = 0;
    double turnCompleteBuff = 0;
    while (turnCompleteBuff < 30) {
        i++;
        if (std::fabs(headingErr) > turnAcc) {
            turnCompleteBuff = 0;
        } else {
            turnCompleteBuff += 1;
        }
        double theta = inertial_sensor.get_rotation();
        double curPosHeading = std::fmod(theta, 180.0) - 180.0 * std::round(theta / (360.0));
        headingErr = targetHeading - curPosHeading;

        if(headingErr < 10) errorsum += headingErr;

        if (std::fabs(headingErr) > 180.0) { headingErr = headingErr > 0.0 ? headingErr - 360.0 : headingErr + 360.0; }

        turnSpeed = headingErr * 1.2 + errorsum * 0.01;

        // if (i % 3 == 0) {
        //     std::cout << "curPos: " << curPos.toString() << ", targetHeading: " << targetHeading << ", "
        //               << "turnCompleteBuff: " << turnCompleteBuff << ", "
        //               << "headingErr: " << headingErr << ", turnSpeed: " << turnSpeed
        //               << std::endl;
        //     printf("turning turn function\n");
        // }

        if(std::abs(turnSpeed) > maxSpeed) {turnSpeed = turnSpeed < 0 ? -maxSpeed : maxSpeed; }

        move(0, turnSpeed, swing);
    }

    printf("turn done");
    move(0, 0);
}


void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    horizontal_rot.reset_position();
    vertical_rot.reset_position();
    wallrot.set_position(0);
    chassis.calibrate(); // calibrate the chassis
    chassis.setPose(0, 0, 0); // X: 0, Y: 0, Heading: 0
    hangs.set_value(false);
    pros::Task screenTask(screen); // create a task to print the position to the screen
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

void skills() {
}

void left_auton() {
}

void right_auton() {
}

void mogorushblueold() {

chassis.moveToPoint(0,-39.5, 1700, {.forwards = false, .maxSpeed = 100}, false);

    clamp.set_value(true);

    pros::delay(450);

    intake.move(127);
    intake2.move(127);

    pros::delay(500);

    chassis.turnToPoint(9, -26, 1500, {.forwards = true});

    intake.move(0);

    chassis.moveToPoint(9, -26, 1700, {.forwards = true, .maxSpeed = 100}, false);

    intake2.move(0);

    // pros::delay(200000000000000);

    chassis.turnToPoint(-35.6, -23.5, 1500, {.forwards = true, .maxSpeed = 100});

    pros::delay(500);

    clamp.set_value(false);

    // pros::delay(700000000000);

    chassis.turnToPoint(-14.5, -7.6, 1500, {.forwards = true, .maxSpeed = 90});

    chassis.moveToPoint(-14.5, -7.6, 1700, {.forwards = true, .maxSpeed = 80}, false);

    chassis.turnToPoint(-30.9, -10.4, 800, {.forwards = true, .maxSpeed = 90});

    chassis.moveToPoint(-30.9, -10.4, 800, {.forwards = true, .maxSpeed = 80}, false);

    chassis.turnToPoint(-44.2, -0.7, 800, {.forwards = false, .maxSpeed = 90});

    chassis.moveToPoint(-44.2, -0.7, 800, {.forwards = false, .maxSpeed = 80}, false);

    chassis.moveToPose(-44.2, -0.7, -212, 800, {.forwards = false, .maxSpeed = 80}, false);
    intake.move(127);
    intake.move(127);
    

    move(-127, -5);

    // pros::delay(500);

    pros::delay(500);

    

    // tm 3.7, -26.4
    // left_auton();
    // right_auton();

}

void bluemogorushnew() {
    intake2.move(127);
    chassis.moveToPoint(0, 8, 700, {.forwards = true, .maxSpeed = 120}, false);

    // pros::delay(0220202002202022);

    chassis.turnToPoint(1.2, 11.2, 700, {.forwards = true, .maxSpeed = 60});

    intake2.move(0);

    chassis.moveToPose(1.2, 11.2, 60, 1000, {.forwards = true, .maxSpeed = 127}, false);

    move(50, 0);


    // intake.move(127);

    // move(60, 0);
    // intake.move(127);
    // intake2.move(127);
    wallmotor.move(127);
    pros::delay(850);
    wallmotor.move(-127);
    move(-20,0);
    pros::delay(700);
    wallmotor.move(0);
    move(0,0);

    // pros::delay(25252242242242);

    chassis.turnToPoint(-18.6, -6.07, 1500, {.forwards = false, .maxSpeed = 60});

    intake2.move(-127);

    chassis.moveToPoint(-18.6, -6.07, 1000, {.forwards = false, .maxSpeed = 127}, false);

    // pros::delay(299299292929);

    intake2.move(0); 

    chassis.turnToPoint(-27.2, -16.65, 600, {.forwards = false, .maxSpeed = 60});

    chassis.moveToPoint(-27.2, -16.65, 1000, {.forwards = false, .maxSpeed = 50}, false);
    clamp.set_value(true);

    pros::delay(300);

    // pros::delay(299299992292929);

    intake.move(127);
    intake2.move(127);

    // pros::delay(29299922929222222);
// heading 171
    chassis.turnToPoint(-25.4, -37.2, 2000, {.forwards = true, .maxSpeed = 50});
    chassis.moveToPoint(-25.4, -37.2, 1750, {.forwards = true, .maxSpeed = 127}, false);

    chassis.turnToPoint(-41.9, -12.8, 1000, {.forwards = false, .maxSpeed = 60}, false);

    // pros::delay(750);

    intake2.move(0);
    intake.move(0);

    wallmotor.move(127);

    move(-127, 0);

    pros::delay(300);

    left_back_motor.brake();
    left_center_motor.brake();
    left_front_motor.brake();
    right_front_motor.brake();
    right_center_motor.brake();
    right_back_motor.brake();
    wallmotor.move(0);
}

void redmogorush() {
    intake2.move(127);
    chassis.moveToPoint(0, 8, 700, {.forwards = true, .maxSpeed = 120}, false);

    // pros::delay(0220202002202022);

    chassis.turnToPoint(-2.3, 11.1, 700, {.forwards = true, .maxSpeed = 60});

    intake2.move(0);

    // pros::delay(499449494994444);

    chassis.moveToPose(-2.3, 11.1, -58, 1000, {.forwards = true, .maxSpeed = 127}, false);

    move(50, 0);


    // intake.move(127);

    pros::delay(50)
;
    // move(60, 0);
    // intake.move(127);
    // intake2.move(127);
    wallmotor.move(127);
    pros::delay(700);
    wallmotor.move(-127);
    move(-20,0);
    pros::delay(700);
    wallmotor.move(0);
    move(0,0);

    // pros::delay(25252242242242);

    chassis.turnToPoint(18.6, -6.07, 1500, {.forwards = false, .maxSpeed = 60});

    intake2.move(-127);

    chassis.moveToPoint(18.6, -6.07, 1000, {.forwards = false, .maxSpeed = 127}, false);

    // pros::delay(299299292929);

    intake2.move(0); 

    chassis.turnToPoint(27.2, -16.65, 600, {.forwards = false, .maxSpeed = 60});

    chassis.moveToPoint(27.2, -16.65, 1000, {.forwards = false, .maxSpeed = 50}, false);
    clamp.set_value(true);

    pros::delay(300);

    // pros::delay(299299992292929);

    intake.move(127);
    intake2.move(127);

    // pros::delay(29299922929222222);
// heading 171
    chassis.turnToPoint(25.4, -37.2, 2000, {.forwards = true, .maxSpeed = 50});
    chassis.moveToPoint(25.4, -37.2, 1750, {.forwards = true, .maxSpeed = 127}, false);

    chassis.turnToPoint(41.9, -12.8, 1250, {.forwards = true, .maxSpeed = 60}, false);

    // pros::delay(750);

    intake2.move(0);
    intake.move(0);

    wallmotor.move(127);

    move(127, 0);

    pros::delay(350);

    left_back_motor.brake();
    left_center_motor.brake();
    left_front_motor.brake();
    right_front_motor.brake();
    right_center_motor.brake();
    right_back_motor.brake();
    wallmotor.move(50);

}

void bluehighscore(){
    intake2.move(127);
    chassis.moveToPoint(0, 8, 700, {.forwards = true, .maxSpeed = 120}, false);

    chassis.turnToPoint(-2.3, 11.1, 650, {.forwards = true, .maxSpeed = 80});

    intake2.move(0);

    chassis.moveToPose(-2.3, 11.1,-58.5, 670, {.forwards = true, .maxSpeed = 127}, false);

    move(30, 0);


    // intake.move(127);

    // move(60, 0);
    // intake.move(127);
    // intake2.move(127);
    wallmotor.move(127);
    pros::delay(750);
    wallmotor.move(-127);
    move(-20,0);
    pros::delay(700);
    wallmotor.move(0);
    move(0,0);

    // pros::delay(25252242242242);

    // chassis.turnToPoint(-18.6, -6.07, 800, {.forwards = false, .maxSpeed = 60});

    // pros::delay(2499242942942942942424);

    intake2.move(-127);

    // chassis.moveToPoint(-18.6, -6.07, 1000, {.forwards = false, .maxSpeed = 127}, false);

    // pros::delay(299299292929);

    chassis.turnToPoint(28.2, -16.2, 800, {.forwards = false, .maxSpeed = 60});

    // intake2.move(0); 

    chassis.moveToPoint(28.2, -16.2, 1000, {.forwards = false, .maxSpeed = 65}, false);

    intake2.move(0); 

    pros::delay(200);

    clamp.set_value(true);

    pros::delay(400);

    // pros::delay(299299992292929);

    intake.move(127);
    intake2.move(127);

    // pros::delay(29299922929222222);
// heading 171
    chassis.turnToPoint(25.4, -37.2, 1000, {.forwards = true, .maxSpeed = 50});
    chassis.moveToPoint(25.4, -37.2, 1000, {.forwards = true, .maxSpeed = 127}, false);

    // pros::delay(292992999992299992922);

    chassis.turnToPoint(35.6, -40.3, 850, {.forwards = true, .maxSpeed = 50});
    chassis.moveToPoint(35.6, -40.3, 1000, {.forwards = true, .maxSpeed = 80}, false);

    // pros::delay(992999929929222);

    chassis.turnToPoint(38.9, -44.6, 800, {.forwards = true, .maxSpeed = 50});

    chassis.moveToPoint(38.9, -44.6, 800, {.forwards = true, .maxSpeed = 40}, false);

    pros::delay(500);

    // chassis.moveToPose(48.8, -47.9, -263, 800, {.forwards = true, .maxSpeed = 40}, false);

    // pros::delay(92922929292);

    chassis.turnToPoint(39.6, -8, 950, {.forwards = true, .maxSpeed = 75}, false);

    intake.move(0);

    move(127, 0);

    wallmotor.move(127);

    pros::delay(370);

    left_back_motor.brake();
    left_center_motor.brake();
    left_front_motor.brake();
    right_front_motor.brake();
    right_center_motor.brake();
    right_back_motor.brake();
    wallmotor.move(20);

    // back turn move 43.5, -9.3

    // chassis.turnToPoint(41.9, -12.8, 1000, {.forwards = false, .maxSpeed = 60}, false);

    // // pros::delay(750);

    // intake2.move(0);
    // intake.move(0);

    // wallmotor.move(127);

    // move(-127, 0);

    // pros::delay(300);

    // left_back_motor.brake();
    // left_center_motor.brake();
    // left_front_motor.brake();
    // right_front_motor.brake();
    // right_center_motor.brake();
    // right_back_motor.brake();
    // wallmotor.move(0);

} 

void newbluemogoside() {
    intake2.move(127);
    chassis.moveToPoint(0, 8, 700, {.forwards = true, .maxSpeed = 120}, false);

    chassis.turnToPoint(2.3, 11.1, 650, {.forwards = true, .maxSpeed = 80});

    intake2.move(0);

    chassis.moveToPose(2.3, 11.1, 59, 670, {.forwards = true, .maxSpeed = 127}, false);

    move(30, 0);

    pros::delay(90);

    // intake.move(127);

    // move(60, 0);
    // intake.move(127);
    // intake2.move(127);
    wallmotor.move(127);
    pros::delay(875);
    wallmotor.move(-127);
    move(-20,0);
    pros::delay(700);
    wallmotor.move(0);
    move(0,0);

    // pros::delay(25252242242242);

    // chassis.turnToPoint(-18.6, -6.07, 800, {.forwards = false, .maxSpeed = 60});

    // pros::delay(2499242942942942942424);

    intake2.move(-127);

    // chassis.moveToPoint(-18.6, -6.07, 1000, {.forwards = false, .maxSpeed = 127}, false);

    // pros::delay(299299292929);

    chassis.turnToPoint(-28.2, -16.2, 800, {.forwards = false, .maxSpeed = 60});

    // intake2.move(0); 

    chassis.moveToPoint(-28.2, -16.2, 1000, {.forwards = false, .maxSpeed = 65}, false);

    intake2.move(0); 

    pros::delay(200);

    clamp.set_value(true);

    pros::delay(400);

    // pros::delay(299299992292929);

    intake.move(127);
    intake2.move(127);

    // pros::delay(29299922929222222);
// heading 171
    chassis.turnToPoint(-25.4, -37.2, 1000, {.forwards = true, .maxSpeed = 50});
    chassis.moveToPoint(-25.4, -37.2, 1000, {.forwards = true, .maxSpeed = 127}, false);

    // pros::delay(292992999992299992922);

    // chassis.turnToPoint(-35.6, -40.3, 850, {.forwards = true, .maxSpeed = 50});
    // chassis.moveToPoint(-35.6, -40.3, 1000, {.forwards = true, .maxSpeed = 80}, false);

    // // pros::delay(992999929929222);

    // chassis.turnToPoint(-38.9, -44.6, 800, {.forwards = true, .maxSpeed = 50});

    // chassis.moveToPoint(-38.9, -44.6, 800, {.forwards = true, .maxSpeed = 40}, false);

    // chassis.moveToPose(48.8, -47.9, -263, 800, {.forwards = true, .maxSpeed = 40}, false);


    chassis.turnToPoint(-40.3, -17.2, 1000, {.forwards = true, .maxSpeed = 60}, false);

    intake.move(0);

    move(127, 0);

    wallmotor.move(127);

    pros::delay(370);

    left_back_motor.brake();
    left_center_motor.brake();
    left_front_motor.brake();
    right_front_motor.brake();
    right_center_motor.brake();
    right_back_motor.brake();
    wallmotor.move(20);

    // back turn move 43.5, -9.3
}

void newredmogoside() {

    pros::delay(29929292929222);
    intake2.move(127);

    // pros::delay(29929292929222);
    chassis.moveToPoint(0, 10, 700, {.forwards = true, .maxSpeed = 120}, false);

    chassis.turnToPoint(-1.2, 15.7, 650, {.forwards = true, .maxSpeed = 80});

    intake2.move(0);

    chassis.moveToPose(-1.2, 15.7, -60, 670, {.forwards = true, .maxSpeed = 127}, false);

    move(30, 0);


    // intake.move(127);

    // move(60, 0);
    // intake.move(127);
    // intake2.move(127);
    wallmotor.move(127);
    pros::delay(750);
    wallmotor.move(-127);
    move(-20,0);
    pros::delay(700);
    wallmotor.move(0);
    move(0,0);

    // pros::delay(25252242242242);

    // chassis.turnToPoint(-18.6, -6.07, 800, {.forwards = false, .maxSpeed = 60});

    // pros::delay(2499242942942942942424);

    intake2.move(-127);

    // chassis.moveToPoint(-18.6, -6.07, 1000, {.forwards = false, .maxSpeed = 127}, false);

    // pros::delay(299299292929);

    chassis.turnToPoint(28.2, -16.2, 800, {.forwards = false, .maxSpeed = 60});

    // intake2.move(0); 

    chassis.moveToPoint(28.2, -16.2, 1000, {.forwards = false, .maxSpeed = 65}, false);

    intake2.move(0); 

    pros::delay(200);

    clamp.set_value(true);

    pros::delay(400);

    // pros::delay(299299992292929);

    intake.move(127);
    intake2.move(127);

    // pros::delay(29299922929222222);
// heading 171
    chassis.turnToPoint(25.4, -37.2, 1000, {.forwards = true, .maxSpeed = 50});
    chassis.moveToPoint(25.4, -37.2, 1000, {.forwards = true, .maxSpeed = 127}, false);

    // pros::delay(292992999992299992922);

    // chassis.turnToPoint(-35.6, -40.3, 850, {.forwards = true, .maxSpeed = 50});
    // chassis.moveToPoint(-35.6, -40.3, 1000, {.forwards = true, .maxSpeed = 80}, false);

    // // pros::delay(992999929929222);

    // chassis.turnToPoint(-38.9, -44.6, 800, {.forwards = true, .maxSpeed = 50});

    // chassis.moveToPoint(-38.9, -44.6, 800, {.forwards = true, .maxSpeed = 40}, false);

    // chassis.moveToPose(48.8, -47.9, -263, 800, {.forwards = true, .maxSpeed = 40}, false);


    chassis.turnToPoint(40.3, -17.2, 1000, {.forwards = true, .maxSpeed = 60}, false);

    intake.move(0);

    move(127, 0);

    wallmotor.move(127);

    pros::delay(370);

    left_back_motor.brake();
    left_center_motor.brake();
    left_front_motor.brake();
    right_front_motor.brake();
    right_center_motor.brake();
    right_back_motor.brake();
    wallmotor.move(20);
}
void redhighscore() {
    intake2.move(127);
    chassis.moveToPoint(0, 8, 700, {.forwards = true, .maxSpeed = 120}, false);

    chassis.turnToPoint(2.3, 11.1, 650, {.forwards = true, .maxSpeed = 80});

    intake2.move(0);

    chassis.moveToPose(2.3, 11.1, 57.5, 670, {.forwards = true, .maxSpeed = 127}, false);

    move(30, 0);


    // intake.move(127);

    pros::delay(100);

    // move(60, 0);
    // intake.move(127);
    // intake2.move(127);
    wallmotor.move(127);
    pros::delay(825);
    wallmotor.move(-127);
    move(-20,0);
    pros::delay(700);
    wallmotor.move(0);
    move(0,0);

    // pros::delay(25252242242242);

    // chassis.turnToPoint(-18.6, -6.07, 800, {.forwards = false, .maxSpeed = 60});

    // pros::delay(2499242942942942942424);

    intake2.move(-127);

    // chassis.moveToPoint(-18.6, -6.07, 1000, {.forwards = false, .maxSpeed = 127}, false);

    // pros::delay(299299292929);

    chassis.turnToPoint(-26.5, -17.9, 800, {.forwards = false, .maxSpeed = 60});

    // intake2.move(0); 

    chassis.moveToPoint(-26.5, -17.9, 1000, {.forwards = false, .maxSpeed = 65}, false);

    intake2.move(0); 

    pros::delay(200);

    clamp.set_value(true);

    pros::delay(400);

    // pros::delay(299299992292929);

    intake.move(127);
    intake2.move(127);

    // pros::delay(29299922929222222);
// heading 171
    chassis.turnToPoint(-25.4, -37.2, 1000, {.forwards = true, .maxSpeed = 50});
    chassis.moveToPoint(-25.4, -37.2, 1000, {.forwards = true, .maxSpeed = 127}, false);

    // pros::delay(292992999992299992922);

    chassis.turnToPoint(-35.6, -40.3, 850, {.forwards = true, .maxSpeed = 50});
    chassis.moveToPoint(-35.6, -40.3, 1000, {.forwards = true, .maxSpeed = 80}, false);

    // pros::delay(992999929929222);

    chassis.turnToPoint(-38.9, -44.6, 800, {.forwards = true, .maxSpeed = 50});

    chassis.moveToPoint(-38.9, -44.6, 800, {.forwards = true, .maxSpeed = 40}, false);

    // chassis.moveToPose(48.8, -47.9, -263, 800, {.forwards = true, .maxSpeed = 40}, false);

    // pros::delay(92922929292);

    chassis.turnToPoint(-39.6, -8, 950, {.forwards = true, .maxSpeed = 75}, false);

    intake.move(0);

    move(127, 0);

    wallmotor.move(127);

    pros::delay(370);

    left_back_motor.brake();
    left_center_motor.brake();
    left_front_motor.brake();
    right_front_motor.brake();
    right_center_motor.brake();
    right_back_motor.brake();
    wallmotor.move(15);

    // back turn move 43.5, -9.3

    // chassis.turnToPoint(41.9, -12.8, 1000, {.forwards = false, .maxSpeed = 60}, false);

    // // pros::delay(750);

    // intake2.move(0);
    // intake.move(0);

    // wallmotor.move(127);

    // move(-127, 0);

    // pros::delay(300);

    // left_back_motor.brake();
    // left_center_motor.brake();
    // left_front_motor.brake();
    // right_front_motor.brake();
    // right_center_motor.brake();
    // right_back_motor.brake();
    // wallmotor.move(0);

}

void autonomous() {
    left_front_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    left_back_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    left_center_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    right_front_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    right_back_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    right_center_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

    hangs.set_value(false);

    // drivetrain.
    chassis.setPose(0, 0, 0);

    // bluehighscore();
    // newbluemogoside();
    redhighscore();
    // newredmogoside();

    // chassis.moveToPose(0, 0, 180, 1500, {.forwards = false, .maxSpeed = 127});

    // pros::delay(1000);
    
    // chassis.turnToHeading(180, 100000);
    
}


void opcontrol() {
    left_front_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    left_back_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    left_center_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    right_front_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    right_back_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    right_center_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    wallmotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

	while (true) {

        int power = master.get_analog(ANALOG_LEFT_Y);
        int turn = master.get_analog(ANALOG_RIGHT_X);

        bool outtakebutton = master.get_digital_new_press(DIGITAL_X);
        bool intakebutton = master.get_digital_new_press(DIGITAL_Y);

        bool yesredirect = master.get_digital(DIGITAL_L1);
        bool notredirect = master.get_digital(DIGITAL_L2);

        bool clampbutton = master.get_digital_new_press(DIGITAL_R2);
        bool unclamp = master.get_digital_new_press(DIGITAL_R1);

        bool wallmech = false;
        bool aaaaaaa = false;

        bool toptwo = master.get_digital(DIGITAL_UP);

        bool downwall = master.get_digital(DIGITAL_DOWN);

        bool hang = master.get_digital_new_press(DIGITAL_A);

        if (hang){
            // hangs.set_value(true);
            // pros::delay(100);
            hangs.set_value(true);
        }

        if (toptwo || downwall){
            macro = false;
        }

        if (toptwo){
            wallmotor.move(127);
        }
        else if (downwall){
            wallmotor.move(-127);
        }
        else if (!macro){
            wallmotor.move(0);
        }

        // if (toptwo){
        //     left_front_motor.move(127);
        //     right_front_motor.move(127);
        // }

        // bool midtwo = master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT);

        // if (midtwo){
        //     left_center_motor.move(127);
        //     right_center_motor.move(127);
        // }

        // bool bottwo = master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);

        // if (bottwo){
        //     left_back_motor.move(127);
        //     right_back_motor.move(127);
        // }

        printf("%d \n", wallrot.get_position());

        if (wallmech){
            mechstage += 1;
            macro = true;
        }

        if (mechstage == 1){
            if (wallrot.get_position() < 1000){
                wallmotor.move(127);
            }
            else{
                wallmotor.move(0);
                mechstage += 1;
            }
        }

        if (mechstage == 3){
            if (wallrot.get_position() < 9000){
                wallmotor.move(127);
            }
            else{
                wallmotor.move(0);
                mechstage += 1;
            }
        }

        if (mechstage == 5){
            if (wallrot.get_position() > 100){
                wallmotor.move(-127);
            }
            else{
                wallmotor.move(0);
                wallrot.reset_position();
                mechstage = 0;
            }
        }

        if (clamped){
            master.set_text(3, 0, ".");
        }
        else{
            master.set_text(3, 0, "");
        }
        if (clampbutton){
            clamp.set_value(true);
            clamped = true;
            
        }
        if (unclamp){
            clamp.set_value(false);
            clamped = false;
        }

        // int intakepower = master.get_analog(ANALOG_RIGHT_X);
        // intake.move(intakepower);
        // move_drive(power, turn);
        // std::printf("power %d", power);
        // std::printf("turn %d", turn);
        
        chassis.arcade(power, turn);
        //move_drive(power, turn);

        if (intakebutton){
            outtaking = false;
            intaking = !intaking; 
        }
        else if (outtakebutton){
            intaking = false;
            outtaking = !outtaking;
        }

        if (outtaking){
            intake.move(127);
            intake2.move(127);
        }
        if (intaking){
            intake.move(-127);
            intake2.move(-127);
        }
        if(!intaking && !outtaking){
            intake.move(0);
            intake2.move(0);
        }

        // if (yesredirect){
        //     redirect = -127;
        // }
        // else if (notredirect){
        //     intake = 127;
        // }
        // else{
        //     intake = 0;
        // }
        
	}
}
