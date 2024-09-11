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

pros::Motor left_front_motor(12, pros::E_MOTOR_GEAR_BLUE, true);
pros::Motor left_center_motor(11, pros::E_MOTOR_GEAR_BLUE	, true);
pros::Motor left_back_motor(14, pros::E_MOTOR_GEAR_BLUE	, true);    
pros::Motor right_front_motor(1, pros::E_MOTOR_GEAR_BLUE	, false);
pros::Motor right_center_motor(4, pros::E_MOTOR_GEAR_BLUE	, false);
pros::Motor right_back_motor(20, pros::E_MOTOR_GEAR_BLUE	, false);

pros::Motor intake(19); //
pros::Motor intake2(15);

// pros::Motor redirect(7); //

pros::Imu inertial_sensor(6);

pros::ADIDigitalOut clamp('H'); //backwings H, F


pros::MotorGroup left_side_motors({left_front_motor, left_center_motor, left_back_motor});
pros::MotorGroup right_side_motors({right_front_motor, right_center_motor, right_back_motor});

pros::Controller master(pros::E_CONTROLLER_MASTER);

bool intaking = false;
bool outtaking = false;

lemlib::Drivetrain drivetrain {
        &left_side_motors, // left drivetrain motors
        &right_side_motors, // right drivetrain motors
        11.25, // track width
        3.25, // wheel diameter
        450, // wheel rpm
        0
};

// 3.5 in horizontal rot displacement

pros::Rotation horizontal_rot(10, false); // port 1, not reversed

lemlib::TrackingWheel horizontal_track(&horizontal_rot, lemlib::Omniwheel::NEW_2 , 4); // 0.6 -0.9

pros::Rotation vertical_rot(16, true); // port 1, not reversed

lemlib::TrackingWheel vertical_track(&vertical_rot, lemlib::Omniwheel::NEW_2,3.375); // 0.6 -0.9

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
lemlib::ControllerSettings lateralController(5.5, // proportional gain (kP)
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
lemlib::ControllerSettings angularController(7, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              15, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
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
    left_center_motor = left;
    left_back_motor = left;
    right_front_motor = right;
    right_center_motor = right;
    right_back_motor = right;
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
    chassis.calibrate(); // calibrate the chassis
    chassis.setPose(0, 0, 0); // X: 0, Y: 0, Heading: 0
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

void autonomous() {
    left_front_motor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    left_back_motor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    left_center_motor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    right_front_motor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    right_back_motor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    right_center_motor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

    // drivetrain.
    chassis.setPose(0, 0, 0);
    pros::delay(1000);
    
    // chassis.turnToHeading(180, 100000);


    chassis.moveToPoint(0,24, 150000, {.forwards = true}, true);
    // left_auton();
    // right_auton();
}


void opcontrol() {
    left_front_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    left_back_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    left_center_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    right_front_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    right_back_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    right_center_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

	while (true) {

        int power = master.get_analog(ANALOG_LEFT_Y);
        int turn = master.get_analog(ANALOG_RIGHT_X);

        bool outtakebutton = master.get_digital_new_press(DIGITAL_X);
        bool intakebutton = master.get_digital_new_press(DIGITAL_Y);

        bool yesredirect = master.get_digital(DIGITAL_L1);
        bool notredirect = master.get_digital(DIGITAL_L2);

        bool clampbutton = master.get_digital_new_press(DIGITAL_R2);
        bool unclamp = master.get_digital_new_press(DIGITAL_R1);

        if (clampbutton){
            clamp.set_value(true);
        }
        if (unclamp){
            clamp.set_value(false);
        }

        // int intakepower = master.get_analog(ANALOG_RIGHT_X);
        // intake.move(intakepower);
        // move_drive(power, turn);
        // std::printf("power %d", power);
        // std::printf("turn %d", turn);
        
        chassis.arcade(power, turn - power/20);
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
