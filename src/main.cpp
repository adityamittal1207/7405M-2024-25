#include "main.h"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include <cstdio>
// #define six_ball 0
// #define far_safe_awp 1
// #define close_rush 2
// #define close_safe_awp 3

// angles: stage 1 - 3000, 

// ASSET(path_txt);

#define RED_RING 0
#define RED_MOGO 1
#define BLUE_RING 2
#define BLUE_MOGO 3
#define SKILLS 5
char autonRunning = BLUE_RING;

const char* autonNames[] = {"RED RING", "RED_MOGO", "BLUE_RING", "BLUE_MOGO"};

pros::Motor left_front_motor(1, pros::E_MOTOR_GEAR_BLUE, true);
pros::Motor left_center_motor(2, pros::E_MOTOR_GEAR_BLUE	, false);
pros::Motor left_back_motor(3, pros::E_MOTOR_GEAR_BLUE	, true);    
pros::Motor right_front_motor(4, pros::E_MOTOR_GEAR_BLUE	, false);
pros::Motor right_center_motor(5, pros::E_MOTOR_GEAR_BLUE	, true);
pros::Motor right_back_motor(6, pros::E_MOTOR_GEAR_BLUE	, false);

pros::Motor intake(7, pros::E_MOTOR_GEAR_BLUE, false); //reversed
pros::Motor intake2(0);

pros::Motor wallmotor(8);

// pros::Motor redirect(7); //

pros::Imu inertial_sensor(20);

pros::ADIDigitalOut clamp('A'); //backwings H, F
pros::ADIDigitalOut hangs('G');
pros::ADIDigitalOut doinker('B');
pros::ADIDigitalOut raiseasdasd('C');


pros::MotorGroup left_side_motors({left_front_motor, left_center_motor, left_back_motor});
pros::MotorGroup right_side_motors({right_front_motor, right_center_motor, right_back_motor});

pros::Controller master(pros::E_CONTROLLER_MASTER);

bool intaking = false;
bool outtaking = false;
int wallstage = 0;
bool raised = false;

lemlib::Drivetrain drivetrain {
        &left_side_motors, // left drivetrain motors
        &right_side_motors, // right drivetrain motors
        12, // track width
        lemlib::Omniwheel::NEW_325_HALF, // wheel diameter
        450, // wheel rpm
        0
};

// 3.5 in horizontal rot displacement

pros::Rotation wallrot(19, false); 

pros::Rotation horizontal_rot(10); // port 1, not reversed
pros::Rotation vertical_rot(9); // port 1, not reversed

lemlib::TrackingWheel horizontal_track(&horizontal_rot, lemlib::Omniwheel::NEW_275 , -3.6f); // 0.6 -0.9
lemlib::TrackingWheel vertical_track(&vertical_rot, lemlib::Omniwheel::NEW_2 , 0.0f); // 0.6 -0.9

// pros::Rotation vertical_rot(16, true); // port 1, not reversed

// lemlib::TrackingWheel vertical_track(&vertical_rot, lemlib::Omniwheel::NEW_2,3.5); // 0.6 -0.9

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
lemlib::ControllerSettings lateralController(15.5, // proportional gain (kP)
                                              0.005, // integral gain (kI)
                                              5.3, // derivative gain (kD)
                                              0, // anti windup
                                              0.5, // small error range, in inches
                                              1000, // small error range timeout, in milliseconds
                                              1, // large error range, in inches
                                              3000, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// turning PID
// lemlib::ControllerSettings angularController(8, // proportional gain (kP)
//                                               0.0, // integral gain (kI)
//                                               125, // derivative gain (kD)
//                                               0, // anti windup
//                                               3, // small error range, in inches
//                                             1000, // small error range timeout, in milliseconds
//                                               6, // large error range, in inches
//                                             3000, // large error range timeout, in milliseconds
//                                               0 // maximum acceleration (slew)
// );
lemlib::ControllerSettings angularController(3.0, // proportional gain (kP)
                                              0.1, // integral gain (kI)
                                              17, // derivative gain (kD)
                                              4, // anti windup
                                              1, // small error range, in inches
                                              1000, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              2000, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);


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

    left_front_motor.move(left);
    left_center_motor.move(left);
    left_back_motor.move(left);
    right_front_motor.move(left);
    right_center_motor.move(left);
    right_back_motor.move(left);
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
    pros::lcd::print(2, "Auton Running: %s", autonNames[autonRunning]); // print the heading
    while (true) {
        
        lemlib::Pose pose = chassis.getPose(); // get the current position of the robot
        char c = autonRunning;
        pros::lcd::print(0, "x: %f | y: %f", pose.x, pose.y, pose.theta); // print the x y position
        pros::lcd::print(1, "H: %f", pose.theta); // print the heading
        pros::lcd::print(3, "TL: %f ML: %f", left_front_motor.get_position(), left_center_motor.get_position());
        pros::lcd::print(4, "BL %f, TR %f", left_back_motor.get_position(), right_front_motor.get_position());
        pros::lcd::print(5, "MR: %f BR: %f", right_center_motor.get_position(), right_back_motor.get_position());
        
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
    wallmotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
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

void arin_je_crncuga(bool forwards, float strength){
    right_front_motor.move(127 * strength * (forwards ? 1 : -1));
    right_center_motor.move(127 * strength * (forwards ? 1 : -1));
    right_back_motor.move(127 * strength * (forwards ? 1 : -1));

    left_front_motor.move(127 * strength * (forwards ? 1 : -1));
    left_center_motor.move(127 * strength * (forwards ? 1 : -1));
    left_back_motor.move(127 * strength * (forwards ? 1 : -1));
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

void random123(){
    chassis.moveToPoint(0, 20, 700, {.forwards = true, .maxSpeed = 120}, false);
    chassis.turnToPoint(90, 11.1, 1500, {.forwards = true, .maxSpeed = 40});
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

    pros::delay(0);
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

void redmogo(){
    clamp.set_value(true);
    chassis.moveToPoint(0, -9.1, 700, {.forwards=false, .maxSpeed = 127});
    //pros::delay(1000000000);
    chassis.turnToHeading(-87, 700, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 127});
    chassis.moveToPoint(2.2, -9.1, 1300, {.forwards=false, .maxSpeed = 127});
    pros::delay(600);
    intake.move(127);
    pros::delay(900);
    intake.move(0);
    chassis.moveToPoint(-1, -7.9, 1300, {.forwards=false, .maxSpeed = 127});
    chassis.turnToHeading(-232, 1300, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 127});
    chassis.moveToPoint(-24.7, 3.4, 1300, {.forwards=false, .maxSpeed = 120});
    pros::delay(1000);
    clamp.set_value(false);
    
    pros::delay(500);
    chassis.turnToHeading(-15, 1300, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 70});
    chassis.moveToPoint(-17, 21, 1300, {.forwards=true, .maxSpeed = 70});
    // chassis.turnToPoint(1.7, -9.6, 1500, {.forwards=false, .maxSpeed = 90});
    // chassis.moveToPose(2.7, -7.8, -86.6, 1500, {.forwards=false, .maxSpeed=70});
    // chassis.moveToPoint(2.7, -7.8, 1500, {.forwards=false, .maxSpeed = 70});
    intake.move(127);
    
    //chassis.moveToPoint(-18.4, 25.3, 1500, {.forwards=true, .maxSpeed = 70});
    chassis.turnToHeading(-149.5, 1300, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 70});
    chassis.moveToPoint(-37, -1.3, 1300, {.forwards=true, .maxSpeed = 70});
    wallmotor.move(70);
    pros::delay(1300);
    wallmotor.move(0);
    //chassis.turnToHeading(80, 1500, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 127});
    //chassis.moveToPoint(-26.1, 20.4, 1500, {.forwards=false, .maxSpeed = 70});
    //chassis.setPose(0, 0, 0);

    //chassis.moveToPoint(-28.5, 21, 1500, {.forwards=false, .maxSpeed = 70});

    // 0, -9.2.        1.7, -9.6
}

void redmogo2(){
    clamp.set_value(true);
    // pros::delay(32943949492394923432943234234);
    chassis.turnToHeading(-280, 1300, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 80});
    // pros::delay(191294923942394); 
    chassis.moveToPoint(-6.5, -5, 1300, {.forwards=false, .maxSpeed = 80});
    chassis.turnToHeading(-220, 1300, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 60}); 
    chassis.moveToPoint(-21.3, 11.1, 1300, {.forwards=false, .maxSpeed = 100});
    pros::delay(600);
    clamp.set_value(false);

    // pros::delay(34435435354354354353453);
    
    pros::delay(500);

    intake.move(127);

    chassis.turnToHeading(8.5, 1300, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 70});
    chassis.moveToPoint(-14.8, 30, 1300, {.forwards=true, .maxSpeed = 70});

    // pros::delay(399423493242343924324);
    // chassis.turnToPoint(1.7, -9.6, 1500, {.forwards=false, .maxSpeed = 90});
    // chassis.moveToPose(2.7, -7.8, -86.6, 1500, {.forwards=false, .maxSpeed=70});
    // chassis.moveToPoint(2.7, -7.8, 1500, {.forwards=false, .maxSpeed = 70});
    // intake.move(127);
    
    //chassis.moveToPoint(-18.4, 25.3, 1500, {.forwards=true, .maxSpeed = 70});
    chassis.turnToHeading(-149.5, 1300, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 70});
    chassis.moveToPoint(-37, -1.3, 1300, {.forwards=true, .maxSpeed = 40});
    wallmotor.move(55);
    pros::delay(1000);
    wallmotor.move(0);
    // // chassis.turnToHeading(-143.4, 1500, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 127});
    // // chassis.moveToPoint(-32, 15, 1500, {.forwards=true, .maxSpeed = 70});
    // wallmotor.move(60);
    // pros::delay(650);
    // wallmotor.move(0);
    //chassis.setPose(0, 0, 0);

    //chassis.moveToPoint(-28.5, 21, 1500, {.forwards=false, .maxSpeed = 70});

    // 0, -9.2.        1.7, -9.6
}


void bluemogo(){
    clamp.set_value(true);
    chassis.moveToPoint(0, -8, 700, {.forwards=false, .maxSpeed = 127});
    chassis.turnToHeading(89, 700, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 60});
    chassis.moveToPoint(-4.5, -8, 1000, {.forwards=false, .maxSpeed = 70}, false);
    intake.move(127);
    pros::delay(500);
    intake.move(0);
    chassis.moveToPoint(3, -8, 1100, {.forwards=true, .maxSpeed = 70});
    chassis.turnToHeading(226, 700, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 60});
    chassis.moveToPoint(21.4, 10, 1100, {.forwards=false, .maxSpeed = 80});
    pros::delay(900);
    clamp.set_value(false);
    chassis.turnToHeading(355, 700, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 60});
    intake.move(127);
    chassis.moveToPoint(20, 30, 1300, {.forwards=true, .maxSpeed = 80}, false);
    pros::delay(600);
    intake.move(0);
    clamp.set_value(true);
    chassis.moveToPoint(20.5, 39.8, 700, {.forwards=true, .maxSpeed = 80});
    chassis.turnToHeading(290.5, 900, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 60});
    chassis.moveToPoint(31, 38.3, 1000, {.forwards=false, .maxSpeed = 50});
    pros::delay(1000);
    clamp.set_value(false);
    lemlib::Pose p = lemlib::Pose(0, 0);
    chassis.setPose(p);
    chassis.moveToPoint(0, 10, 1600, {.forwards=true, .maxSpeed = 50});
    chassis.turnToHeading(-99, 900, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 60});
    chassis.moveToPoint(-39, 2, 1600, {.forwards=true, .maxSpeed = 50});
    //chassis.turnToHeading(178, 900, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 60});
    //chassis.moveToPoint(28.7, 18.1, 1300, {.forwards=false, .maxSpeed = 80});


    //chassis.turnToHeading(491, 700, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 60});
    //chassis.moveToPoint(37.6, 14, 1300, {.forwards=true, .maxSpeed = 80});

    //pros::delay(600);
    //intake.move(127);
    //pros::delay(900);
    //intake.move(0);
}

void bluemogo2(){
    clamp.set_value(true);
    // pros::delay(32943949492394923432943234234);
    chassis.turnToHeading(-123, 1300, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 80});
    // pros::delay(191294923942394); 
    chassis.moveToPoint(16, 13, 1300, {.forwards=false, .maxSpeed = 80});
    // chassis.turnToHeading(220, 1300, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 60}); 
    // chassis.moveToPoint(21.3, 11.1, 1300, {.forwards=false, .maxSpeed = 100});
    pros::delay(600);
    clamp.set_value(false);

    // pros::delay(34435435354354354353453);
    
    pros::delay(700);

    intake.move(127);

    chassis.turnToHeading(-16, 1300, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 70});
    chassis.moveToPoint(9, 32, 1300, {.forwards=true, .maxSpeed = 70});

    // pros::delay(399423493242343924324);
    // chassis.turnToPoint(1.7, -9.6, 1500, {.forwards=false, .maxSpeed = 90});
    // chassis.moveToPose(2.7, -7.8, -86.6, 1500, {.forwards=false, .maxSpeed=70});
    // chassis.moveToPoint(2.7, -7.8, 1500, {.forwards=false, .maxSpeed = 70});
    // intake.move(127);
    
    //chassis.moveToPoint(-18.4, 25.3, 1500, {.forwards=true, .maxSpeed = 70});
    chassis.turnToHeading(149.5, 1300, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 70});
    chassis.moveToPoint(37, -1.3, 1300, {.forwards=true, .maxSpeed = 40});
    wallmotor.move(55);
    pros::delay(1000);
    wallmotor.move(0);
    // chassis.turnToHeading(-143.4, 1500, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 127});
    // chassis.moveToPoint(-32, 15, 1500, {.forwards=true, .maxSpeed = 70});
    // wallmotor.move(60);
    // pros::delay(650);
    // wallmotor.move(0);
    // chassis.setPose(0, 0, 0);

    // chassis.moveToPoint(-28.5, 21, 1500, {.forwards=false, .maxSpeed = 70});

    // 0, -9.2.        1.7, -9.6
}

void redStupid(){
    chassis.moveToPoint(0, -8.3, 1600, {.forwards=false, .maxSpeed = 50});
    chassis.turnToHeading(90, 700, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 60});
    chassis.moveToPoint(-3.2, -8.3, 1600, {.forwards=false, .maxSpeed = 50});
    chassis.moveToPoint(5.3, -8.5, 1600, {.forwards=true, .maxSpeed = 50});
    chassis.turnToHeading(224, 700, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 60});
    chassis.moveToPoint(21.6, 9.3, 1600, {.forwards=false, .maxSpeed = 50});
    chassis.turnToHeading(349, 700, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 60});
    chassis.moveToPoint(18.5, 20.4, 1600, {.forwards=true, .maxSpeed = 50});
    chassis.turnToHeading(444, 700, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 60});
    chassis.moveToPoint(32.6, 31.4, 1600, {.forwards=true, .maxSpeed = 50});
}

void redmogo3(){
    clamp.set_value(true);
    chassis.moveToPoint(0, -8.1, 1599, {.forwards=false});
    chassis.turnToHeading(-90, 700, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 60});
    chassis.moveToPoint(1.8, -8.1, 1599, {.forwards=false});
    pros::delay(300);
    intake.move(127);
    // pros::delay(3924923492349349234342);
    chassis.moveToPoint(-9, -6.1, 1599, {.forwards=true, .maxSpeed=80});
    chassis.turnToHeading(130, 1500, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 60});
    chassis.moveToPoint(-25, 6, 1599, {.forwards=false});
    pros::delay(500);
    clamp.set_value(false);
    chassis.turnToHeading(0, 1500, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE});
    chassis.moveToPoint(-23, 30, 1599, {.forwards=true});
    
    // move 0, -8.2
    // turn 90
    // move -3.4, -8.2
    // move 2.2, -8.2, 90
    // turn 228
    //move 19.5, 8.2

}

void redMogoG(){
    pros::delay(2000);
    clamp.set_value(false);
    
    chassis.moveToPoint(0, 7, 700, {.forwards=true, .maxSpeed=80}, true);
    chassis.turnToHeading(-3, 300, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 60});
    wallmotor.move(127);
    pros::delay(550);
    wallmotor.move(0);
    wallmotor.move(-127);
    chassis.turnToHeading(-33.5, 700, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 70});
    pros::delay(700);

    wallmotor.move(0);
    //chassis.moveToPoint(5, 0.1, 850, {.forwards=false, .maxSpeed=80}, true);
    //chassis.turnToHeading(0.3, 800, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 60});
    chassis.moveToPoint(4.3, -16, 950, {.forwards=false, .maxSpeed=80}, true);
    pros::delay(1000);
    clamp.set_value(true);
    pros::delay(200);
    chassis.turnToHeading(-481, 900, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 60});
    chassis.moveToPoint(-16, -27.8, 1000, {.forwards=true, .maxSpeed=100}, true);
    intake.move(127);
    
    chassis.turnToHeading(-31, 1000, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 60});
    
    chassis.moveToPoint(-33, -7, 900, {.forwards=true, .maxSpeed=100}, true);
    pros::delay(300);
    intake.move(-127);
    pros::delay(1000);
    intake.move(127);
    chassis.moveToPoint(-28.9, -12, 800, {.forwards=false, .maxSpeed=100}, true);
    chassis.turnToHeading(-84, 800, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 60});
    chassis.moveToPoint(-48, -9, 1000, {.forwards=true, .maxSpeed=127}, false);
    arin_je_crncuga(true, 1.0);
    pros::delay(600);
    arin_je_crncuga(true, 0.0);
    chassis.moveToPoint(-29.7, -12.8, 900, {.forwards=false, .maxSpeed=80}, true);
    chassis.turnToHeading(99, 900, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 60});
    chassis.moveToPoint(-4, -17, 900, {.forwards=true, .maxSpeed=80}, true);
    intake.move(127);
    wallmotor.move(127);
    pros::delay(600);
    wallmotor.move(0);
    

}   

void bluMogoG(){

    // move_drive(-40, 0);
    // pros::delay(700);
    

    clamp.set_value(false);
    
    chassis.moveToPoint(0, 7.3, 600, {.forwards=true, .maxSpeed=80}, false);
    chassis.setPose(0, 7.3, 0);
    // pros::delay(2527575572572572525);
    chassis.turnToHeading(3, 600, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 60});
    wallmotor.move(127);
    pros::delay(700);
    wallmotor.move(-127);
    pros::delay(100);
    chassis.moveToPoint(-5, 0.1, 800, {.forwards=false, .maxSpeed=80}, true);
    pros::delay(800);

    wallmotor.move(0);
    chassis.turnToHeading(-0.3, 800, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 60});
    chassis.moveToPoint(-2.0, -16.5, 900, {.forwards=false, .maxSpeed=70}, true);
    pros::delay(750);
    clamp.set_value(true);
    chassis.turnToHeading(113, 1000, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 60});
    intake.move(127);
    chassis.moveToPoint(21.4, -28.1, 900, {.forwards=true, .maxSpeed=80}, true);
    
    
    chassis.turnToHeading(20, 900, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 60});
    chassis.moveToPoint(29.6, -3.4, 900, {.forwards=true, .maxSpeed=80}, true);
    pros::delay(300);
    intake.move(0-127);
    chassis.moveToPoint(31, -13.6, 900, {.forwards=false, .maxSpeed=80}, true);
    ////hassis.moveToPoint(15, -15, 900, {.forwards=false, .maxSpeed=80}, true);
    pros::delay(500);
    intake.move(127);
    //chassis.moveToPoint(26.1, -7.6, 1000, {.forwards=false, .maxSpeed=80}, true);
    chassis.turnToHeading(40, 900, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 60});
    chassis.moveToPoint(46.8, 2, 1200, {.forwards=true, .maxSpeed=80}, true);
    arin_je_crncuga(true, 0.5);
    pros::delay(500);
    arin_je_crncuga(true, 0);
    chassis.moveToPoint(23.5, -11.8, 700, {.forwards=false, .maxSpeed=80}, true);
    chassis.turnToHeading(-72.8, 900, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 60});
    chassis.moveToPoint(-4.6, 0.6, 1000, {.forwards=true, .maxSpeed=80}, true);
    arin_je_crncuga(true, 0.4);
    wallmotor.move(127);
    pros::delay(600);
    wallmotor.move(0);

    

}   

void bluRingG(){
    clamp.set_value(false);
    chassis.moveToPoint(0, 7, 700, {.forwards=true, .maxSpeed=80}, true);
    chassis.turnToHeading(-3, 700, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 60});
    wallmotor.move(127);
    pros::delay(600);
    wallmotor.move(0);
    wallmotor.move(-127);
    // chassis.turnToHeading(-33.5, 1500, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 60});
    pros::delay(600);
    // wallmotor.move(0);

    

  

    // chassis.moveToPoint(5, 0.1, 1599, {.forwards=false, .maxSpeed=80}, true);
    // chassis.turnToHeading(0.3, 1500, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 60});
    chassis.moveToPoint(4.3, -16, 1100, {.forwards=false, .maxSpeed=70}, true);
    wallmotor.move(0);
    pros::delay(1000);
    clamp.set_value(true);
    intake.move(127);
    chassis.turnToHeading(-481, 900, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 60});
    chassis.moveToPoint(-16, -28.3, 1000, {.forwards=true, .maxSpeed=80}, true);
    pros::delay(600);
    chassis.turnToHeading(-278, 1000, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 60});
    
    chassis.moveToPoint(-1.3, -28, 1000, {.forwards=true, .maxSpeed=80}, true);

    chassis.turnToHeading(-188, 800, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 60});
    chassis.moveToPoint(1.4, -40, 1100, {.forwards=true, .maxSpeed=40}, true);
    //pros::delay(100000 * 3/5 * 'n' * 'i' * 'g' * 'g' * 'a');
    pros::delay(300);
    chassis.moveToPoint(0.75, -37.5, 1000, {.forwards=true, .maxSpeed=80}, true);
    chassis.turnToHeading(-176, 800, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 60});
    chassis.moveToPoint(-1, -46.6, 1000, {.forwards=true, .maxSpeed=80}, true);

    chassis.moveToPoint(-0.3, -28.3, 1000, {.forwards=false, .maxSpeed=80}, true);
    chassis.turnToHeading(-221, 750, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 60});
    chassis.moveToPoint(8, -39, 1000, {.forwards=true, .maxSpeed=80}, true);
    arin_je_crncuga(true,1 );





    /*
    pros::delay(300);
    intake.move(-127);
    pros::delay(1000);
    intake.move(127);
    chassis.moveToPoint(-28.9, -12, 1599, {.forwards=false, .maxSpeed=80}, true);
    chassis.turnToHeading(-70, 1500, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 60});
    chassis.moveToPoint(-60.4, -0.5, 1599, {.forwards=true, .maxSpeed=80}, true);
    chassis.moveToPoint(-29.7, -12.8, 1599, {.forwards=false, .maxSpeed=80}, true);
    chassis.turnToHeading(99, 1500, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 60});
    chassis.moveToPoint(-4, -17, 1599, {.forwards=true, .maxSpeed=80}, true);
    intake.move(127);
    */

}   

void redRingA(){
    clamp.set_value(false);
    
    chassis.moveToPoint(0, 7.7, 700, {.forwards=true, .maxSpeed=80}, false);
    wallmotor.move(127);
    pros::delay(580);
    wallmotor.move(0);
    wallmotor.move(-127);
    // chassis.turnToHeading(-33.5, 1500, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 60});
    
    // wallmotor.move(0);

    

  

    // chassis.moveToPoint(5, 0.1, 1599, {.forwards=false, .maxSpeed=80}, true);
    // chassis.turnToHeading(0.3, 1500, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 60});
    chassis.moveToPoint(-7.6,-16.5, 1100, {.forwards=false, .maxSpeed=70}, true);
    pros::delay(600);
    wallmotor.move(0);
    pros::delay(725);
    clamp.set_value(true);
    chassis.turnToHeading(122, 800, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 60});
    chassis.moveToPoint(8.4, -27.5, 700, {.forwards=true, .maxSpeed=80}, true);
    intake.move(127);
    
    chassis.turnToHeading(278, 800, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 60});

    // pros::delay(242424242424424);
    
    chassis.moveToPoint(0.9, -27.7, 800, {.forwards=true, .maxSpeed=80}, true);

    // pros::delay(294294924924924244242424);
    
    chassis.turnToHeading(200, 600, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 70});
    chassis.moveToPoint(-4.1, -39.2, 1300, {.forwards=true, .maxSpeed=35}, false);
    pros::delay(400);
    chassis.moveToPoint(-2.6, -35.3, 1300, {.forwards=false, .maxSpeed=50}, true);
    chassis.turnToHeading(174.5, 1000, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 60});
    chassis.moveToPoint(-0.3, -45.9, 1300, {.forwards=true, .maxSpeed=35}, true);
    pros::delay(400);

    // pros::delay(546456654654654654646456);
    chassis.moveToPoint(-0.4, -33.9, 1000, {.forwards=false, .maxSpeed=80}, true);
    chassis.turnToHeading(302, 750, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 60});
    chassis.moveToPoint(-20.5,-18, 1000, {.forwards=true, .maxSpeed=80}, true);
    arin_je_crncuga(true,0.1 );

}

void bluRingA(){
    clamp.set_value(false);
    
    chassis.moveToPoint(0, 7, 700, {.forwards=true, .maxSpeed=80}, true);
    chassis.turnToHeading(-6, 700, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 60});

    wallmotor.move(127);
    pros::delay(750);
    wallmotor.move(0);
    pros::delay(100);
    wallmotor.move(-127);
    // chassis.turnToHeading(-33.5, 1500, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 60});
    
    // wallmotor.move(0);

    

  

    // chassis.moveToPoint(5, 0.1, 1599, {.forwards=false, .maxSpeed=80}, true);
    // chassis.turnToHeading(0.3, 1500, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 60});
    chassis.moveToPoint(-7.6,-16.5, 1100, {.forwards=false, .maxSpeed=70}, true);
    pros::delay(600);
    wallmotor.move(0);
    pros::delay(725);
    clamp.set_value(true);
    chassis.turnToHeading(122, 800, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 60});
    chassis.moveToPoint(8.4, -27.5, 700, {.forwards=true, .maxSpeed=80}, true);
    intake.move(127);
    
    chassis.turnToHeading(278, 800, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 60});

    // pros::delay(242424242424424);
    
    chassis.moveToPoint(0.9, -27.7, 800, {.forwards=true, .maxSpeed=80}, true);

    // pros::delay(294294924924924244242424);
    
    chassis.turnToHeading(200, 600, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 70});
    chassis.moveToPoint(-4.1, -39.2, 1300, {.forwards=true, .maxSpeed=35}, false);
    pros::delay(400);
    chassis.moveToPoint(-2.6, -35.3, 1300, {.forwards=false, .maxSpeed=50}, true);
    chassis.turnToHeading(174.5, 1000, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 60});
    chassis.moveToPoint(-0.3, -45.9, 1300, {.forwards=true, .maxSpeed=35}, true);
    pros::delay(400);

    // pros::delay(546456654654654654646456);
    chassis.moveToPoint(-0.4, -33.9, 1000, {.forwards=false, .maxSpeed=80}, true);
    chassis.turnToHeading(302, 750, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 60});
    chassis.moveToPoint(-20.5,-18, 1000, {.forwards=true, .maxSpeed=80}, true);
    arin_je_crncuga(true,0.1 );

}

void redRingG(){
    clamp.set_value(false);
    
    chassis.moveToPoint(0, 7.8, 700, {.forwards=true, .maxSpeed=80}, true);
    chassis.turnToHeading(3, 700, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 60});
    wallmotor.move(127);
    pros::delay(650);
    wallmotor.move(0);
    wallmotor.move(-127);
    // chassis.turnToHeading(-33.5, 1500, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 60});
    // wallmotor.move(0);
    // chassis.moveToPoint(5, 0.1, 1599, {.forwards=false, .maxSpeed=80}, true);
    // chassis.turnToHeading(0.3, 1500, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 60});
    chassis.moveToPoint(-2, -17, 1100, {.forwards=false, .maxSpeed=70}, true);
    pros::delay(600);
    wallmotor.move(0);
    pros::delay(725);
    clamp.set_value(true);
    chassis.turnToHeading(481, 900, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 60});
    chassis.moveToPoint(16, -27.8, 1000, {.forwards=true, .maxSpeed=80}, true);
    intake.move(127);
    
    chassis.turnToHeading(278, 1000, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 60});
    
    chassis.moveToPoint(0.6, -25.3, 1000, {.forwards=true, .maxSpeed=80}, true);

    // pros::delay(294294924924924244242424);
    
    chassis.turnToHeading(182, 1500, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 60});
    chassis.moveToPoint(-2.9, -38.7, 1300, {.forwards=true, .maxSpeed=35}, false);
    pros::delay(300);
    chassis.moveToPoint(-2.4, -32.3, 1300, {.forwards=false, .maxSpeed=50}, true);
    chassis.turnToHeading(161, 1000, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 60});
    chassis.moveToPoint(3.3, -48.6, 1300, {.forwards=true, .maxSpeed=35}, true);
    pros::delay(300);
    chassis.moveToPoint(2.08, -34.4, 1000, {.forwards=false, .maxSpeed=80}, true);
    chassis.turnToHeading(297, 750, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 60});
    chassis.moveToPoint(-20.4, -23.4, 1000, {.forwards=true, .maxSpeed=80}, true);
    arin_je_crncuga(true,0.75);





    /*
    pros::delay(300);
    intake.move(-127);
    pros::delay(1000);
    intake.move(127);
    chassis.moveToPoint(-28.9, -12, 1599, {.forwards=false, .maxSpeed=80}, true);
    chassis.turnToHeading(-70, 1500, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 60});
    chassis.moveToPoint(-60.4, -0.5, 1599, {.forwards=true, .maxSpeed=80}, true);
    chassis.moveToPoint(-29.7, -12.8, 1599, {.forwards=false, .maxSpeed=80}, true);
    chassis.turnToHeading(99, 1500, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 60});
    chassis.moveToPoint(-4, -17, 1599, {.forwards=true, .maxSpeed=80}, true);
    intake.move(127);
    */


}   

void blueelim(){
    clamp.set_value(true);
    chassis.moveToPoint(0, -26, 1700, {.forwards=false, .maxSpeed = 90});
    pros::delay(800);
    clamp.set_value(false);
    chassis.turnToHeading(-105, 1500, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 60});
    pros::delay(800);
    intake.move(127);
    chassis.moveToPoint(-21.5, -27.4, 1700, {.forwards=true, .maxSpeed = 40});
    chassis.turnToHeading(-71, 1500, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 60});
    chassis.moveToPoint(-27, -25.7, 1700, {.forwards=true, .maxSpeed = 70});
    chassis.moveToPoint(-17, -29, 1700, {.forwards=false, .maxSpeed = 60});
    chassis.turnToHeading(-29, 1500, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 60});
    chassis.moveToPoint(-27, -14.7, 1700, {.forwards=true, .maxSpeed = 80});
    chassis.turnToHeading(8.5, 1500, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 60});

    pros::delay(0);

    chassis.moveToPoint(-21.5, 21.7, 1700, {.forwards=true, .maxSpeed = 127});
    // turn -105
    // move -23, -28
    // turn -69.5
    // move -31, 25.1
    // move -17, -29
    // turn -29
    // move -27, -14.7
    // turn 6.6
}

void turnAuton(){
    chassis.turnToHeading(180, 1500, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 60});
    chassis.turnToHeading(0, 1500, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 60});
}

void redRingAuton123() {
    clamp.set_value(true);
    chassis.turnToHeading(-2, 550, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 60});
    wallmotor.move(-57);
    pros::delay(400);
    wallmotor.move(-20);
    pros::delay(200);
    chassis.moveToPoint(0.7, 7.8, 600, {.forwards=true, .maxSpeed = 40}, false);
    pros::delay(100);
    wallmotor.move(100);
    pros::delay(300);
    wallmotor.move(0);
    chassis.moveToPoint(0.0, -8, 700, {.forwards=false, .maxSpeed = 100});
    chassis.turnToHeading(-43.6, 550, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 60});
    chassis.moveToPoint(16, -27, 800, {.forwards=false, .maxSpeed = 100});
    pros::delay(600);
    clamp.set_value(false);
    chassis.turnToHeading(-130, 700, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 60});
    intake.move(127);
    chassis.moveToPoint(4.1, -40.7, 700, {.forwards=true, .maxSpeed = 100}, false);
    pros::delay(200);
    intake.move(-127);
    //chassis.turnToHeading(-69, 500, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 60});
    //chassis.moveToPoint(-34.6, -25, 900, {.forwards=true, .maxSpeed = 100}, false);
    //pros::delay(200);
    //intake.move(127);
    // chassis.turnToHeading(50, 1500, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 60});
    // chassis.moveToPoint(6.6, 4.9, 1700, {.forwards=true, .maxSpeed = 80}, false);
    // wallmotor.move(70);
    // pros::delay(700);
    // wallmotor.move(0);
    // pros::delay(200);
    // chassis.moveToPoint(1.2, 0.3, 1700, {.forwards=true, .maxSpeed = 80});
    // pros::delay(100);
    // clamp.set_value(false);
    // pros::delay(200);
    // chassis.turnToHeading(11, 1500, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 60});
    // chassis.moveToPoint(-6.6, -32.3, 1700, {.forwards=false, .maxSpeed = 80});
}

void skillsAuton() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    clamp.set_value(true);
    intake.move(127);
    pros::delay(700);
    intake.move(0);
    pros::delay(200);
    chassis.moveToPoint(0, 12, 500, {.forwards=true, .maxSpeed = 80});
    chassis.turnToHeading(-89, 550, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 55});
    chassis.moveToPoint(21, 13, 700, {.forwards=false, .maxSpeed = 80});
    pros::delay(600);
    clamp.set_value(false);
    chassis.turnToHeading(7, 600, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 50});
    chassis.moveToPoint(24.7, 38.1, 800, {.forwards=true, .maxSpeed = 80});
    intake.move(127);
    //cross auton line & get ring to store in lb
    //go to auton line & turn towards wall stake while intaking ring
    //score wall stake
    chassis.turnToHeading(60, 600, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 55});
    chassis.moveToPoint(47, 67, 1100, {.forwards=true, .maxSpeed = 120});
    chassis.turnToHeading(6, 500, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 60});
    chassis.moveToPoint(50, 88, 850, {.forwards=true, .maxSpeed = 100});
    chassis.moveToPoint(46.7, 69.5, 800, {.forwards=false, .maxSpeed = 100});
    chassis.turnToHeading(90.63, 900, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 60});
    chassis.moveToPoint(61, 63, 700, {.forwards=true, .maxSpeed = 100});
    chassis.moveToPoint(46, 62, 700, {.forwards=false, .maxSpeed = 100}); 
    chassis.turnToHeading(180, 700, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 60}); 
    chassis.moveToPoint(49, 1, 4000, {.forwards=true, .maxSpeed = 60});  
    chassis.moveToPoint(43, 10, 1000, {.forwards=false, .maxSpeed = 100});  
    chassis.turnToHeading(320, 1500, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 60}); 
    chassis.moveToPoint(58.7, -6.0, 1000, {.forwards=false, .maxSpeed = 100});  
    pros::delay(300);
    intake.move(-50);
    clamp.set_value(true);

    pros::delay(300);
    intake.move(127);
    chassis.moveToPoint(-25, 18, 1000, {.forwards=true, .maxSpeed = 50});
    chassis.turnToHeading(90, 1000, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 55});
    chassis.moveToPoint(-32, 18, 2300, {.forwards=false, .maxSpeed = 100});
    pros::delay(1500);
    clamp.set_value(false);

    chassis.turnToHeading(367, 800, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 50});
    chassis.moveToPoint(-24, 35.3, 1000, {.forwards=true, .maxSpeed = 80});

    chassis.turnToHeading(337, 600, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 50});
    intake.move(127);
    chassis.moveToPoint(-41, 92, 2500, {.forwards=true, .maxSpeed = 80});
    
    //cross auton line & get ring to store in lb
    //go to auton line & turn towards wall stake while intaking ring
    //score wall stake
    chassis.turnToHeading(214, 600, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 55});
    chassis.moveToPoint(-54, 74.8, 1100, {.forwards=true, .maxSpeed = 120});
    chassis.turnToHeading(158, 500, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 60});
    chassis.moveToPoint(-46, 47, 1000, {.forwards=true, .maxSpeed = 100});
    chassis.turnToHeading(186.63, 600, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 60});
    chassis.moveToPoint(-50, 9, 3000, {.forwards=true, .maxSpeed = 40});
    //gkk
    chassis.turnToHeading(75, 600, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 60});
    chassis.moveToPoint(-63.8, 8.6, 3000, {.forwards=false, .maxSpeed = 40}, false);
    clamp.set_value(true);  
    chassis.moveToPoint(-50, 9, 700, {.forwards=false, .maxSpeed = 100});

    chassis.turnToHeading(180, 600, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 60});
    chassis.moveToPoint(-50, 100, 3000, {.forwards=false, .maxSpeed = 100});
    chassis.turnToHeading(90, 600, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 60});
    chassis.moveToPoint(60, 113.5, 3000, {.forwards=true, .maxSpeed = 100});
    chassis.turnToHeading(90, 600, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 60});
    chassis.moveToPoint(-70, 113.5, 3000, {.forwards=false, .maxSpeed = 100});
}

float sumError2 = 0.0f; 
float p2 = 0.1f;
float targetPos2 = 30;

void pidUpdate() {
    float error = targetPos2 - wallrot.get_angle();
    float returnVolt = p2 * error;
    wallmotor.move(127*returnVolt);
    p2 = 0.01;

}

void newBotRedMogo(){
    clamp.set_value(true);
    raiseasdasd.set_value(false);

    //initial move & slight turn towards wall motor - gets on alliance stake
    wallmotor.move(-127);
    chassis.moveToPoint(0, 10.5, 1000, {.forwards=true, .maxSpeed = 70});  
    pros::delay(200);
    wallmotor.move(0);
    chassis.turnToHeading(-7, 1000, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 90}); 
    wallmotor.move(127);
    pros::delay(335);
    wallmotor.move(-127);

    //turn towards ring in front of alliance stake, moves to get it, 
    chassis.turnToHeading(89, 1000, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 90}); 
    raiseasdasd.set_value(true);
    intake.move(127);
    chassis.moveToPoint(12, 10, 1000, {.forwards=true, .maxSpeed = 70});  
    wallmotor.move(0);
    chassis.turnToHeading(-16, 1000, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 80}); 
    pros::delay(300);
    intake.move(0);
    wallmotor.move(0);
    raiseasdasd.set_value(false);

    //move to mogo to clamp, turn towards ring in front of center rings to intake 
    chassis.moveToPoint(17, -27, 1050, {.forwards=false, .maxSpeed = 80});  
    pros::delay(950);
    clamp.set_value(false);
    chassis.turnToHeading(-129, 1000, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 80}); 
    chassis.moveToPoint(5.8, -40.5, 1000, {.forwards=true, .maxSpeed = 100});  
    intake.move(127);

    chassis.turnToHeading(-48, 1000, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 80}); 
    chassis.moveToPoint(-22.8, -19.2, 1000, {.forwards=true, .maxSpeed = 100});  

    intake.move(-127);

    chassis.moveToPoint(-16.1, -25.5, 1000, {.forwards=false, .maxSpeed = 100});  

    chassis.turnToHeading(-89, 1000, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 80}, false);

    intake.move(127);

    move(80, 0);

    pros::delay(800);

    move(0,0);

    chassis.moveToPoint(35, -25, 1000, {.forwards=false, .maxSpeed = 100});  
}

void newBotBlueMogo(){
    clamp.set_value(true);
    raiseasdasd.set_value(false);

    //initial move & slight turn towards wall motor - gets on alliance stake
    wallmotor.move(-127);
    chassis.moveToPoint(-0, 10, 1000, {.forwards=true, .maxSpeed = 70});  
    pros::delay(200);
    wallmotor.move(0);
    chassis.turnToHeading(7, 1000, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 90}); 
    wallmotor.move(127);
    pros::delay(335);
    wallmotor.move(-127);

    //turn towards ring in front of alliance stake, moves to get it, 
    chassis.turnToHeading(-89, 1000, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 90}); 
    raiseasdasd.set_value(true);
    intake.move(127);
    chassis.moveToPoint(-12, 10, 1000, {.forwards=true, .maxSpeed = 70});  
    wallmotor.move(0);
    chassis.turnToHeading(16, 1000, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 80}); 
    pros::delay(300);
    intake.move(0);
    wallmotor.move(0);
    raiseasdasd.set_value(false);

    //move to mogo to clamp, turn towards ring in front of center rings to intake 
    chassis.moveToPoint(-15.9, -25, 1000, {.forwards=false, .maxSpeed = 80});  
    pros::delay(900);
    clamp.set_value(false);
    chassis.turnToHeading(129, 1000, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 80}); 
    chassis.moveToPoint(-5.8, -40.5, 1000, {.forwards=true, .maxSpeed = 100});  
    intake.move(127);

    chassis.turnToHeading(48, 1000, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 80}); 
    chassis.moveToPoint(22.8, -19.2, 1000, {.forwards=true, .maxSpeed = 100});  

    intake.move(-127);

    chassis.moveToPoint(16.1, -25.5, 1000, {.forwards=false, .maxSpeed = 100});  

    chassis.turnToHeading(89, 1000, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 80}, false);

    intake.move(127);

    move(80, 0);

    pros::delay(800);

    move(0,0);

    chassis.moveToPoint(-35, -25, 1000, {.forwards=false, .maxSpeed = 100});  
}

void newBotBlueRing(){
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    clamp.set_value(true);
    raiseasdasd.set_value(false);

    //wallmotor.move(-127);
    chassis.moveToPoint(0, 9.2, 700, {.forwards=true, .maxSpeed = 127});  
    chassis.turnToHeading(4, 400, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 127}); 
    wallmotor.move(100);
    pros::delay(500);
    wallmotor.move(-127);
    pros::delay(100);
    wallmotor.move(0);
    chassis.moveToPoint(0, 0, 700, {.forwards=false, .maxSpeed = 127}, false);  
    wallmotor.move(-127);
    //pros::delay(1000);

    //turn towards ring in front of alliance stake, moves to get it, 
    chassis.turnToHeading(68, 700, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 127}); 
    raiseasdasd.set_value(true);
    intake.move(127);
    chassis.moveToPoint(11.7, 6.6, 700, {.forwards=true, .maxSpeed = 127});  
    wallmotor.move(-30);
    pros::delay(800);
    intake.move(0);
    chassis.turnToHeading(-16, 700, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 80}); 
    wallmotor.move(0);
    raiseasdasd.set_value(false);

    //move to mogo to clamp, turn towards ring in front of center rings to intake 
    chassis.moveToPoint(15.9, -25, 1000, {.forwards=false, .maxSpeed = 80});  
    pros::delay(800);
    clamp.set_value(false);
    chassis.turnToHeading(-124, 800, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 80}); 
    //pros::delay(10000);
    chassis.moveToPoint(4.5, -41, 900, {.forwards=true, .maxSpeed = 127});  
    intake.move(127);
    //pros::delay(10000);

    //gets the ring in front of center rings, then gets rings on center
    chassis.moveToPoint(18, -35.4, 800, {.forwards=false, .maxSpeed = 127});
    chassis.turnToHeading(-181, 800, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 80}); 
    chassis.moveToPoint(17.8, -47.5, 800, {.forwards=true, .maxSpeed = 45});
    chassis.moveToPoint(17.6, -42.4, 500, {.forwards=false, .maxSpeed = 90});
    chassis.turnToHeading(-97, 600, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 80}); 
    chassis.moveToPoint(2.4, -43.9, 900, {.forwards=true, .maxSpeed = 45});
    chassis.turnToHeading(-208, 800, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 80}); 
    chassis.moveToPoint(7.5, -55.5, 900, {.forwards=true, .maxSpeed = 127});

    chassis.turnToHeading(-217, 500, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 127}); 

    chassis.moveToPoint(-18, -32.1, 1000, {.forwards=false, .maxSpeed = 127});

    chassis.turnToHeading(-92, 800, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 100}); 

    chassis.moveToPoint(-35.6, -24.4, 800, {.forwards=true, .maxSpeed = 127});


    //corner
    //raiseasdasd.set_value(false);
    // chassis.turnToHeading(39, 800, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 80}, false);
    // chassis.moveToPoint(12.5, -24.5, 1000, {.forwards=true, .maxSpeed = 40}, false);
    // chassis.turnToHeading(75.7, 700, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 80}, false);
    // chassis.moveToPoint(-35, -25, 1000, {.forwards=false, .maxSpeed = 100});  
    // wallmotor.move(30);
}

void newBotBotRedRing() 
{
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    clamp.set_value(true);
    raiseasdasd.set_value(false);

    //wallmotor.move(-127);
    chassis.moveToPoint(0.1, 9.7, 700, {.forwards=true, .maxSpeed = 70});  
    chassis.turnToHeading(-6, 400, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 90}); 
    wallmotor.move(127);
    pros::delay(500);
    wallmotor.move(-127);
    pros::delay(100);
    wallmotor.move(0);
    chassis.moveToPoint(1.9, -6.2, 800, {.forwards=false, .maxSpeed = 70}, false);  
    wallmotor.move(-127);
    //pros::delay(1000);

    //turn towards ring in front of alliance stake, moves to get it, 
    chassis.turnToHeading(-51, 700, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 90}); 
    raiseasdasd.set_value(true);
    intake.move(127);
    chassis.moveToPoint(-12.3, 4.9, 700, {.forwards=true, .maxSpeed = 70});  
    wallmotor.move(-30);
    pros::delay(800);
    intake.move(0);
    chassis.turnToHeading(5.5, 700, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 80}); 
    wallmotor.move(0);
    raiseasdasd.set_value(false);

    //move to mogo to clamp, turn towards ring in front of center rings to intake 
    chassis.moveToPoint(-19.7, -23.6, 1000, {.forwards=false, .maxSpeed = 80});  
    pros::delay(900);
    clamp.set_value(false);
    chassis.turnToHeading(157, 800, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 80}); 
    //pros::delay(10000);
    chassis.moveToPoint(-11.5, -44, 900, {.forwards=true, .maxSpeed = 127});  
    intake.move(127);
    // //pros::delay(10000);

    //gets the ring in front of center rings, then gets rings on center
    // chassis.moveToPoint(-12.2, -30.7, 800, {.forwards=false, .maxSpeed = 127});
    // chassis.turnToHeading(188, 800, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 80}); 
    // chassis.moveToPoint(-14.7, -49, 800, {.forwards=true, .maxSpeed = 45});
    // chassis.moveToPoint(-13.8, -43.8, 500, {.forwards=false, .maxSpeed = 45});
    // chassis.turnToHeading(89, 600, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 80}); 
    // chassis.moveToPoint(-0.6, -45, 1000, {.forwards=true, .maxSpeed = 45});
    // chassis.turnToHeading(213, 1000, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 80}); 
    // chassis.moveToPoint(-6.5, -56.4, 1000, {.forwards=true, .maxSpeed = 45});
}

void newBotRedRing(){
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    clamp.set_value(true);
    raiseasdasd.set_value(false);

    wallmotor.move(-127);
    chassis.moveToPoint(0, 9.5, 700, {.forwards=true, .maxSpeed = 70});  
    pros::delay(300);
    wallmotor.move(0);
    chassis.turnToHeading(-4, 400, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 90}); 
    wallmotor.move(127);
    pros::delay(500);
    wallmotor.move(0);
    pros::delay(300); 
    chassis.moveToPoint(0, 0, 650, {.forwards=false, .maxSpeed = 70}, false);  
    wallmotor.move(-127);

    //turn towards ring in front of alliance stake, moves to get it, 
    chassis.turnToHeading(-50, 700, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 90}); 
    raiseasdasd.set_value(true);
    intake.move(127);
    chassis.moveToPoint(-12, 10, 700, {.forwards=true, .maxSpeed = 70});  
    wallmotor.move(-30);
    pros::delay(800);
    intake.move(0);
    chassis.turnToHeading(16, 700, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 80}); 
    wallmotor.move(0);
    raiseasdasd.set_value(false);

    //move to mogo to clamp, turn towards ring in front of center rings to intake 
    chassis.moveToPoint(-15.9, -25, 1000, {.forwards=false, .maxSpeed = 80});  
    pros::delay(900);
    clamp.set_value(false);
    chassis.turnToHeading(129, 800, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 80}); 
    chassis.moveToPoint(-5.8, -40.5, 900, {.forwards=true, .maxSpeed = 127});  
    intake.move(127);

    //gets the ring in front of center rings, then gets rings on center
    chassis.moveToPoint(-15.4, -37.6, 800, {.forwards=false, .maxSpeed = 127});
    chassis.turnToHeading(-182, 1000, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 80}); 
    intake.move(127);
    chassis.moveToPoint(-17.1, -45.5, 800, {.forwards=true, .maxSpeed = 45});
    // chassis.turnToHeading(81, 600, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 80}); 
    // chassis.moveToPoint(-4.2, -43.9, 700, {.forwards=true, .maxSpeed = 45});
    intake.move(0);
    // chassis.turnToHeading(-50, 1000, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 80}, false); 
    // // move(50, 0);
    // // wallmotor.move(30);
    // // pros::delay(1000);
    // // move(10, 0);
    // chassis.turnToHeading(198, 1000, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 80}, false); 
    // pros::delay(300);
    // intake.move(127);
    //chassis.moveToPoint(-8.2, -54.2, 700, {.forwards=true, .maxSpeed = 45}, false);

    //corner
    //raiseasdasd.set_value(false);
    // chassis.turnToHeading(39, 800, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 80}, false);
    // chassis.moveToPoint(12.5, -24.5, 1000, {.forwards=true, .maxSpeed = 40}, false);
    // chassis.turnToHeading(75.7, 700, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 80}, false);
    // chassis.moveToPoint(-35, -25, 1000, {.forwards=false, .maxSpeed = 100});  
    // wallmotor.move(30);
}

void redRingNoFourth()
{
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    clamp.set_value(true);
    raiseasdasd.set_value(false);

    wallmotor.move(-127);
    chassis.moveToPoint(0, 9.5, 700, {.forwards=true, .maxSpeed = 70});  
    pros::delay(300);
    wallmotor.move(0);
    chassis.turnToHeading(-4, 400, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 90}); 
    wallmotor.move(127);
    pros::delay(500);
    wallmotor.move(-127);
    pros::delay(500); 

    //turn towards ring in front of alliance stake, moves to get it, 
    chassis.turnToHeading(-89, 700, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 90}); 
    raiseasdasd.set_value(true);
    intake.move(127);
    chassis.moveToPoint(-12, 10, 700, {.forwards=true, .maxSpeed = 70});  
    wallmotor.move(0);
    pros::delay(700);
    intake.move(0);
    chassis.turnToHeading(16, 700, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 80}); 
    wallmotor.move(0);
    raiseasdasd.set_value(false);

    //move to mogo to clamp, turn towards ring in front of center rings to intake 
    chassis.moveToPoint(-15.9, -25, 1000, {.forwards=false, .maxSpeed = 80});  
    pros::delay(900);
    clamp.set_value(false);
    chassis.turnToHeading(129, 800, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 80}); 
    chassis.moveToPoint(-5.8, -40.5, 900, {.forwards=true, .maxSpeed = 127});  
    intake.move(127);

    //gets the ring in front of center rings, then gets rings on center
    chassis.moveToPoint(-15.4, -37.6, 800, {.forwards=false, .maxSpeed = 127});
    chassis.turnToHeading(-182, 1000, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 80}); 
    intake.move(127);
    chassis.moveToPoint(-17.1, -45.5, 800, {.forwards=true, .maxSpeed = 45});

    // chassis.moveToPoint(-4.2, -43.9, 700, {.forwards=true, .maxSpeed = 45});
    // intake.move(0);
    // chassis.turnToHeading(198, 1000, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 80}, false); 
    // pros::delay(300);
    // intake.move(127);
    // chassis.moveToPoint(-8.2, -54.2, 700, {.forwards=true, .maxSpeed = 45}, false);

}

void autonomous() {

    autonRunning = BLUE_RING;

    doinker.set_value(false);
    //clamp.set_value
    left_front_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    left_back_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    left_center_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    right_front_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    right_back_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    right_center_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

    hangs.set_value(false);

    //skillsAuton();
    //newBotBlueRing();
        newBotBlueRing();
    
        //newBotRedRing();
    
        // newBotBlueMogo();

        // newBotRedMogo();

        //skillsAuton();
    // newBotBlueRing();
    // wallmotor.move(-50);
    // wallrot.set_position(0);
    // newBotBlueMogo();

    //turnAuton();
    // bluRingG();
    // redRingG();
    // redRingA();
    // redRingG();
    // redMogoG();
    //bluMogoG();
    // skillsAuton();
    // redRingAuton123();
    // bluRingA();


    // drivetrain.
    // chassis.setPose(0, 0, 0);

    // bluehighscore();
    // newbluemogoside();
    // redhighscore();
    // newredmogoside();

    // bluemogo();
    // redmogo();
    // bluemogo2();
    // redmogo2();
    // redmogo3();
    //blueelim();
// 
    // chassis.turnToHeading(90, 3000, {.maxSpeed=127});

    // bluMogoG();


    // pros::delay(1000);
    
    // chassis.turnToHeading(180, 100000);
    // pros::task_t my_task = task_create(pidUpdate, (void*)"PROS", TASK_PRIORITY_DEFAULT,
    //                            TASK_STACK_DEPTH_DEFAULT, "My Task");
    // pros::Task my_cpp_task (my_task);
}

#define targetWait 34
#define targetTop 154
#define targetDown 210
#define targetHold 70

void opcontrol() {

    wallrot.set_position(0);
    bool doinkdoinkdoinkdoinkdoink = false;
    bool doinkdoink2 = false;
    doinker.set_value(false);
    left_front_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    left_back_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    left_center_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    right_front_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    right_back_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    right_center_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    wallmotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    // wallstage = 4;
    int timer = 0;
    int pos = 0;
    float pwall, iwall, dwall;
    pwall = 0.02f;
    iwall = 0.0f;
    dwall = 0.002f;
    float threshold = 5.0f;
    bool clamped2 = false;
    float preverr = 0.0f;
    float sumerrorwall = 0.0f;
    float targetpos = targetWait;
    float err;
    int stage = 0;
    bool lastCycle = false;
    wallmotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    bool manual = false;
	while (true) {

        int power = master.get_analog(ANALOG_LEFT_Y);
        int turn = master.get_analog(ANALOG_RIGHT_X);
        bool manualswitch = master.get_digital_new_press(DIGITAL_LEFT);
        if(manualswitch){
            manual = !manual;
        }

        bool outtakebutton = master.get_digital_new_press(DIGITAL_X);
        bool intakebutton = master.get_digital_new_press(DIGITAL_Y);
        bool wallcycler = master.get_digital_new_press(DIGITAL_L1);
        bool wallreturn = master.get_digital_new_press(DIGITAL_L2);
        bool wallMid = master.get_digital_new_press(DIGITAL_B);
        bool wallDown = master.get_digital_new_press(DIGITAL_DOWN);
        pos = wallrot.get_position()/100.0f;
        err = targetpos - pos;
        float derr = err-preverr;
        sumerrorwall += err;
        float ret = pwall * err + iwall * sumerrorwall + dwall * derr;
        if(manual){goto skp;}
        wallmotor.move((ret * 127) > 127 ? 127 : (ret*127));
        // bool yesredirect = master.get_digital(DIGITAL_L1);
        // bool notredirect = master.get_digital(DIGITAL_L2);

        
        if(wallcycler && !lastCycle){
            if(stage == 4){
                stage = 2;
            goto skprest;
            }
            stage = ((stage <= 1) ? stage+1 : 1);
            skprest:
            if(stage == 2){
                intaking = false;
                outtaking = false;
                intake.move(-127);
                pros::delay(50);
                intake.move(0);
            }
        }
        
        if(wallDown) {
            stage = (stage != 3) ? 3 : 0;
            sumerrorwall = 0.0f;
        }
        if(wallMid) {
            stage = (stage != 4) ? 4 : 0;
            sumerrorwall = 0.0f;
            intaking = false;
            outtaking = false;
            intake.move(-127);
            pros::delay(50);
            intake.move(0);
        }
        if(wallcycler || wallreturn){
            sumerrorwall = 0.0f;
        }
        if(wallreturn){
            sumerrorwall = 0.0f;
            stage = 0;
        }
        // if(fabs(wallrot.get_angle() % 360) < 3.0f){
        //     wallrot.set_position(0);
        // }
        targetpos = (stage == 1) * targetWait + (stage == 2) * targetTop + (stage == 3) * targetDown + (stage == 4) * targetHold;
        goto skp2;
        skp:;


        skp2:;

        bool clampbutton = master.get_digital_new_press(DIGITAL_R1);

        bool wallmech = false;
        bool aaaaaaa = false;

        bool toptwo = master.get_digital(DIGITAL_UP);

        bool downwall = master.get_digital(DIGITAL_DOWN);

        doinkdoink2 = master.get_digital_new_press(DIGITAL_A);

        if(clampbutton){
            clamped2 = !clamped2;
            clamp.set_value(clamped2);
            if(clamped2){
                intaking = false; outtaking = false;
            }
        }

        if(master.get_digital_new_press(DIGITAL_UP)){
            raised = !raised;
            raiseasdasd.set_value(raised);
        }

        if(doinkdoink2){
            doinkdoinkdoinkdoinkdoink = !doinkdoinkdoinkdoinkdoink;
            doinker.set_value(doinkdoinkdoinkdoinkdoink);
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

        

        if (!clamped2){
            master.set_text(3, 0, ".");
        }
        else{
            master.set_text(3, 0, "");
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
        timer += 10;
	}
}