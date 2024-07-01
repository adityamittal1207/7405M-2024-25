#include "main.h"
#include "lemlib/api.hpp"
#include "pros/adi.hpp"
#include "pros/rtos.hpp"
#include <functional>
#include <valarray>

// #define six_ball 0
// #define far_safe_awp 1
// #define close_rush 2
// #define close_safe_awp 3

// ASSET(path_txt);

pros::Motor left_front_motor(0, pros::E_MOTOR_GEARSET_06, true);
pros::Motor left_center_motor(0, pros::E_MOTOR_GEARSET_06, true);
pros::Motor left_back_motor(0, pros::E_MOTOR_GEARSET_06, true);
pros::Motor right_front_motor(0, pros::E_MOTOR_GEARSET_06, false);
pros::Motor right_center_motor(0, pros::E_MOTOR_GEARSET_06, false);
pros::Motor right_back_motor(0, pros::E_MOTOR_GEARSET_06, false);

pros::Motor intake(0); //

pros::Imu inertial_sensor(0);


pros::MotorGroup left_side_motors({left_front_motor, left_center_motor, left_back_motor});
pros::MotorGroup right_side_motors({right_front_motor, right_center_motor, right_back_motor});

pros::Controller master(pros::E_CONTROLLER_MASTER);

lemlib::Drivetrain drivetrain {
        &left_side_motors, // left drivetrain motors
        &right_side_motors, // right drivetrain motors
        11.188, // track width
        3.25, // wheel diameter
        450, // wheel rpm
        8
        
};

pros::Rotation horizontal_rot(15, false); // port 1, not reversed

lemlib::TrackingWheel horizontal_track(&horizontal_rot, 2.75, 4.252888446, 1); // 0.6 -0.9


// odometry struct
lemlib::OdomSensors sensors {
        nullptr, // vertical tracking wheel 1
        nullptr, // vertical tracking wheel 2
        // nullptr,
        &horizontal_track, // horizontal tracking wheel 1
        nullptr, // we don't have a second tracking wheel, so we set it to nullptr
        &inertial_sensor // inertial sensor
};

// forward/backward PID
lemlib::ControllerSettings lateralController {
        4, // kP
        0,
        10, // kD
        0,
        0.75, // smallErrorRange
        100, // smallErrorTimeout
        2, // largeErrorRange
        500, // largeErrorTimeout
        5 // slew rate
};

// turning PID
lemlib::ControllerSettings angularController {
        1.4, // kP
        0,
        12, // kD
        0,
        1, // smallErrorRange
        100, // smallErrorTimeout
        3, // largeErrorRange
        500, // largeErrorTimeout
        40 // slew rate
};

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

    if (left < 0) left = left - 9;
    else if (left > 0) left = left + 9;

    if (right < 0) right = right - 9;
    else if (right > 0) right = right + 9;


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
    left_front_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    left_back_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    left_center_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    right_front_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    right_back_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    right_center_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    // left_auton();
    // right_auton();
}


void opcontrol() {
    left_front_motor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    left_back_motor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    left_center_motor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    right_front_motor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    right_back_motor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    right_center_motor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

	while (true) {

        int power = master.get_analog(ANALOG_LEFT_Y);
        int turn = master.get_analog(ANALOG_RIGHT_X);
        move_drive(power, dampen(turn));
	}
}
