#include "main.h"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include <cstdio>

#include "subsystems/Intake.h"
#include "subsystems/Ladybrown.h"


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

/*
DRIVE MOTORS
*/

pros::Motor left_front_motor(1, pros::E_MOTOR_GEAR_BLUE, true);
pros::Motor left_center_motor(2, pros::E_MOTOR_GEAR_BLUE	, false);
pros::Motor left_back_motor(3, pros::E_MOTOR_GEAR_BLUE	, true);    
pros::Motor right_front_motor(4, pros::E_MOTOR_GEAR_BLUE	, false);
pros::Motor right_center_motor(5, pros::E_MOTOR_GEAR_BLUE	, true);
pros::Motor right_back_motor(6, pros::E_MOTOR_GEAR_BLUE	, false);

pros::Imu inertial_sensor(20);


/*
SUBSYSTEMS
*/

pros::Motor intake(7, pros::E_MOTOR_GEAR_BLUE, false); //reversed
pros::Motor intake2(0);

Intake intake_class;

// pros::Motor wallmotor(8);

Ladybrown ladybrown_class;

/*
PNEUMATICS
*/

pros::ADIDigitalOut clamp('A'); //backwings H, F
pros::ADIDigitalOut hangs('G');
pros::ADIDigitalOut doinker('B');
pros::ADIDigitalOut raiseasdasd('C');

/*
DRIVE GROUPS
*/

pros::MotorGroup left_side_motors({left_front_motor, left_center_motor, left_back_motor});
pros::MotorGroup right_side_motors({right_front_motor, right_center_motor, right_back_motor});

lemlib::Drivetrain drivetrain {
    &left_side_motors, // left drivetrain motors
    &right_side_motors, // right drivetrain motors
    12, // track width
    lemlib::Omniwheel::NEW_325_HALF, // wheel diameter
    450, // wheel rpm
    0
};

/*
SENSORS
*/

// pros::Rotation wallrot(19, false); 

pros::Rotation horizontal_rot(10); // port 1, not reversed
pros::Rotation vertical_rot(9); // port 1, not reversed

lemlib::TrackingWheel horizontal_track(&horizontal_rot, lemlib::Omniwheel::NEW_275 , -3.6f); // 0.6 -0.9
lemlib::TrackingWheel vertical_track(&vertical_rot, lemlib::Omniwheel::NEW_2 , 0.0f); // 0.6 -0.9

lemlib::OdomSensors sensors {
        &vertical_track, // vertical tracking wheel 1
        nullptr, // vertical tracking wheel 2
        // nullptr,
        &horizontal_track, // horizontal tracking wheel 1
        nullptr, // we don't have a second tracking wheel, so we set it to nullptr
        &inertial_sensor // inertial sensor
};

/*
PID CONTROLLERS
*/

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
pros::Controller master(pros::E_CONTROLLER_MASTER);

/*
GLOBAL VARIABLES
*/

bool intaking = false;
bool outtaking = false;
int wallstage = 0;
bool raised = false;
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

/*
INTAKE THREAD
*/

void intake_thread(){
    while(true){
    // printf("%f \n",intake.get_torque());
    intake_class.update(intake.get_torque());
    intake.move(intake_class.get_velocity());
    pros::delay(10);
    }
}

/*
LADYBROWN THREAD
*/

void Ladybrown_thread(){
    while(true){
    ladybrown_class.update(wallrot.get_position());
    wallmotor.move(ladybrown_class.get_velocity());
    pros::delay(10);
    }
}

/*
SCREEN THREAD
*/

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
    pros::Task intakeTask(intake_thread);
    pros::Task ladybrownTask(Ladybrown_thread);

}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
}

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

void skillsAutonWallStakes() {
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
    chassis.turnToHeading(14.3, 500, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 60});
    chassis.moveToPoint(43.2, 63.5, 800, {.forwards=false, .maxSpeed = 80}, false);
    int tmt = 0;
    while(tmt < 200000){
        tmt += fabsf(wallrot.get_position()/100.0f - 36) < 3;
        wallmotor.move((wallrot.get_position()/100.0f - 36) * -3);
        ++tmt;
    }
    wallmotor.move_velocity(0);
    chassis.turnToHeading(90, 900, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 60});
    chassis.moveToPoint(62.8, 61, 700, {.forwards=true, .maxSpeed = 70}, false);
    intake.move(0);
    pros::delay(1000);
    wallmotor.move(127);
    intake.move(0);
    pros::delay(870);
    chassis.moveToPoint(48, 62, 700, {.forwards=false, .maxSpeed = 100}, false);
    wallmotor.move(-100);
    pros::delay(900); 
    chassis.turnToHeading(185, 850, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 60}); 
    chassis.moveToPoint(52, 1, 4500, {.forwards=true, .maxSpeed = 40});  
    chassis.moveToPoint(43, 10, 700, {.forwards=false, .maxSpeed = 100});  
    chassis.turnToHeading(320, 750, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 60}); 
    chassis.moveToPoint(58.7, -6.0, 1000, {.forwards=false, .maxSpeed = 100});  
    pros::delay(300);
    intake.move(-50);
    clamp.set_value(true);

    pros::delay(300);
    intake.move(127);
    chassis.moveToPoint(-25, 23, 1000, {.forwards=true, .maxSpeed = 50});
    chassis.turnToHeading(90, 1000, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 55});
    // pros::delay(2294929429429429242442);
    intake.move(0);
    chassis.moveToPoint(-25, 11, 2400, {.forwards=false, .maxSpeed = 60});
    pros::delay(1750);
    clamp.set_value(false);
    intake.move(127);

    chassis.turnToHeading(367, 800, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 50});
    chassis.moveToPoint(-24, 35.3, 1000, {.forwards=true, .maxSpeed = 80});
    // pros::delay(124225252525254254254254245524452);

    chassis.turnToHeading(-53.5, 600, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 50});
    
    chassis.moveToPoint(-55, 54, 1500, {.forwards=true, .maxSpeed = 80});

    // pros::delay(294929429494924924);

    chassis.moveToPoint(-51, 56.3, 1000, {.forwards=false, .maxSpeed = 80});

    chassis.turnToHeading(0, 600, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 80});

    chassis.moveToPoint(-50.3, 81.9, 1000, {.forwards=true, .maxSpeed = 50});


    
    //cross auton line & get ring to store in lb
    //go to auton line & turn towards wall stake while intaking ring
    //score wall stake
    chassis.turnToHeading(180, 600, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 55});
    chassis.moveToPoint(-52, 6.4, 4200, {.forwards=true, .maxSpeed = 50});
    chassis.moveToPoint(-57.3, 14.9, 800, {.forwards=false, .maxSpeed = 100});

    chassis.turnToHeading(48.5, 500, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 90});

    chassis.moveToPoint(-64.2, -3, 1000, {.forwards=false, .maxSpeed = 100}, false);
    clamp.set_value(true);
    intake.move(-50);
    pros::delay(200);



    //chassis.turnToHeading(158, 800, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 60});
    chassis.moveToPoint(-49, 56.7, 1000, {.forwards=true, .maxSpeed = 100}, false);
    intake.move(0);
    //chassis.turnToHeading(9, 600, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 60});
    chassis.turnToHeading(30, 700, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 60});
    intake.move(127);
    chassis.moveToPoint(-25.8, 88, 1000, {.forwards=true, .maxSpeed = 100});
    
    
    //gkk
    chassis.turnToHeading(-129, 900, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 60});
    intake.move(0);
    chassis.moveToPoint(6, 114.2, 900, {.forwards=false, .maxSpeed = 70});
    pros::delay(800);

    // pros::delay(924924294929494294924949494424);

    clamp.set_value(false);  

    // chassis.turnToHeading(-18, 600, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 60});
    intake.move(-50);
    // wallmotor.move_voltage(127);

    // wallmotor.move_voltage(-127);

    // wallmotor.move_voltage(0);

    intake.move(127); 
    chassis.turnToHeading(137.7, 900, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 80});
    chassis.moveToPoint(28, 81.3, 800, {.forwards=true, .maxSpeed = 70});
    pros::delay(1200);    

    chassis.turnToHeading(41.3, 800, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 80});

    chassis.moveToPoint(52.4, 114.7, 1000, {.forwards=true, .maxSpeed = 50});   
    
    chassis.turnToHeading(90, 800, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 50}, false);
    intake.move(0);
    chassis.moveToPoint(80, 115.8, 800, {.forwards=true, .maxSpeed = 40});   

    chassis.turnToHeading(-139, 800, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 80}, false);
    clamp.set_value(true);
    chassis.moveToPoint(61.2, 114.6, 1000, {.forwards=false, .maxSpeed = 127});   
    chassis.moveToPoint(48.4, 105.7, 800, {.forwards=true, .maxSpeed = 40});   
    chassis.turnToHeading(-259, 800, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 80}, false);
    chassis.moveToPoint(-54.8, 127.5, 4000, {.forwards=false, .maxSpeed = 100});   

}

void skillsAutonNoWallStakes() {
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
    chassis.moveToPoint(46.7, 69.5, 800, {.forwards=false, .maxSpeed = 100}, false);
    
    chassis.turnToHeading(90.63, 900, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 60});
    chassis.moveToPoint(61, 66, 700, {.forwards=true, .maxSpeed = 100});
    // pros::delay(1200301020002300);
    chassis.moveToPoint(48, 62, 700, {.forwards=false, .maxSpeed = 100}); 
    chassis.turnToHeading(185, 850, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 60}); 
    chassis.moveToPoint(52, 1, 4500, {.forwards=true, .maxSpeed = 40});  
    chassis.moveToPoint(43, 10, 700, {.forwards=false, .maxSpeed = 100});  
    chassis.turnToHeading(320, 750, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 60}); 
    chassis.moveToPoint(58.7, -6.0, 1000, {.forwards=false, .maxSpeed = 100});  
    pros::delay(300);
    intake.move(-50);
    clamp.set_value(true);

    pros::delay(300);
    intake.move(127);
    chassis.moveToPoint(-25, 23, 1000, {.forwards=true, .maxSpeed = 50});
    chassis.turnToHeading(90, 1000, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 55});
    // pros::delay(2294929429429429242442);
    intake.move(0);
    chassis.moveToPoint(-25, 11, 2400, {.forwards=false, .maxSpeed = 60});
    pros::delay(1750);
    clamp.set_value(false);
    intake.move(127);

    chassis.turnToHeading(367, 800, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 50});
    chassis.moveToPoint(-24, 35.3, 1000, {.forwards=true, .maxSpeed = 80});
    // pros::delay(124225252525254254254254245524452);

    chassis.turnToHeading(-53.5, 600, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 50});
    
    chassis.moveToPoint(-55, 54, 1500, {.forwards=true, .maxSpeed = 80});

    // pros::delay(294929429494924924);

    chassis.moveToPoint(-51, 56.3, 1000, {.forwards=false, .maxSpeed = 80});

    chassis.turnToHeading(0, 600, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 80});

    chassis.moveToPoint(-50.3, 81.9, 1000, {.forwards=true, .maxSpeed = 50});


    
    //cross auton line & get ring to store in lb
    //go to auton line & turn towards wall stake while intaking ring
    //score wall stake
    chassis.turnToHeading(180, 600, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 55});
    chassis.moveToPoint(-52, 6.4, 4200, {.forwards=true, .maxSpeed = 50});
    chassis.moveToPoint(-57.3, 14.9, 800, {.forwards=false, .maxSpeed = 100});

    chassis.turnToHeading(48.5, 500, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 90});

    chassis.moveToPoint(-64.2, -3, 1000, {.forwards=false, .maxSpeed = 100}, false);
    clamp.set_value(true);
    intake.move(-50);
    pros::delay(200);



    //chassis.turnToHeading(158, 800, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 60});
    chassis.moveToPoint(-49, 56.7, 1000, {.forwards=true, .maxSpeed = 100}, false);
    intake.move(0);
    //chassis.turnToHeading(9, 600, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 60});
    chassis.turnToHeading(30, 700, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 60});
    intake.move(127);
    chassis.moveToPoint(-25.8, 88, 1000, {.forwards=true, .maxSpeed = 100});
    
    
    //gkk
    chassis.turnToHeading(-129, 900, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 60});
    intake.move(0);
    chassis.moveToPoint(6, 114.2, 900, {.forwards=false, .maxSpeed = 70});
    pros::delay(800);

    // pros::delay(924924294929494294924949494424);

    clamp.set_value(false);  

    // chassis.turnToHeading(-18, 600, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 60});
    intake.move(-50);
    // wallmotor.move_voltage(127);

    // wallmotor.move_voltage(-127);

    // wallmotor.move_voltage(0);

    intake.move(127); 
    chassis.turnToHeading(137.7, 900, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 80});
    chassis.moveToPoint(27, 80, 800, {.forwards=true, .maxSpeed = 70});
    pros::delay(1400);    

    chassis.turnToHeading(41.3, 800, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 80});

    chassis.moveToPoint(52.4, 114.7, 1000, {.forwards=true, .maxSpeed = 50});   
    
    chassis.turnToHeading(90, 800, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 50}, false);
    intake.move(0);
    chassis.moveToPoint(80, 115.8, 800, {.forwards=true, .maxSpeed = 40});   

    chassis.turnToHeading(-139, 800, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 80}, false);
    clamp.set_value(true);
    chassis.moveToPoint(61.2, 114.6, 1000, {.forwards=false, .maxSpeed = 127});   
    chassis.moveToPoint(48.4, 105.7, 800, {.forwards=true, .maxSpeed = 40});   
    chassis.turnToHeading(-259, 800, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 80}, false);
    chassis.moveToPoint(-54.8, 127.5, 4000, {.forwards=false, .maxSpeed = 100});   

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

void BlueRingC(){
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    clamp.set_value(true);
    raiseasdasd.set_value(false);
    //wallmotor.move(-127);
    chassis.moveToPoint(0, 7.5, 700, {.forwards=true, .maxSpeed = 100});  
    wallmotor.move(72);
    chassis.turnToHeading(5, 700, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 127});
    // pros::delay(100);
    pros::delay(175);
    pros::delay(150);
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
    pros::delay(600);
    raiseasdasd.set_value(false);
    pros::delay(200);
    intake.move(0);
    chassis.turnToHeading(-16, 800, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 80}); 
    wallmotor.move(0);
    raiseasdasd.set_value(false);

    //move to mogo to clamp, turn towards ring in front of center rings to intake 
    chassis.moveToPoint(15.9, -25, 1000, {.forwards=false, .maxSpeed = 80});  
    pros::delay(800);
    clamp.set_value(false);
    chassis.turnToHeading(-124, 800, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 80}); 
    //pros::delay(10000);
    chassis.moveToPoint(2.5, -43, 900, {.forwards=true, .maxSpeed = 127});  
    intake.move(127);
    //pros::delay(10000);

    //gets the ring in front of center rings, then gets rings on center
    chassis.moveToPoint(18, -35.4, 800, {.forwards=false, .maxSpeed = 127});
    chassis.turnToHeading(-181, 800, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 80}); 
    chassis.moveToPoint(17.8, -47.5, 800, {.forwards=true, .maxSpeed = 55});
    chassis.moveToPoint(17.6, -42.4, 500, {.forwards=false, .maxSpeed = 90});
    chassis.turnToHeading(-97, 600, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 80}); 
    chassis.moveToPoint(3.4, -41.9, 900, {.forwards=true, .maxSpeed = 55});
    intake.move(-127);
    chassis.turnToHeading(-208, 800, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 80}); 
    pros::delay(200);
    intake.move(127);
    chassis.moveToPoint(9, -53.5, 900, {.forwards=true, .maxSpeed = 127}, false);    intake.move(127);

    chassis.turnToHeading(-217, 500, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 127}); 

    chassis.moveToPoint(-18, -32.1, 1000, {.forwards=false, .maxSpeed = 127});

    chassis.turnToHeading(-88, 500, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 90}); 

    // chassis.moveToPoint(-35.6, -24.4, 700, {.forwards=true, .maxSpeed = 127});

    // chassis.moveToPoint(-18, -32.1, 1000, {.forwards=false, .maxSpeed = 127});


    //corner
    //raiseasdasd.set_value(false);
    // chassis.turnToHeading(39, 800, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 80}, false);
    // chassis.moveToPoint(12.5, -24.5, 1000, {.forwards=true, .maxSpeed = 40}, false);
    // chassis.turnToHeading(75.7, 700, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 80}, false);
    // chassis.moveToPoint(-35, -25, 1000, {.forwards=false, .maxSpeed = 100});  
    // wallmotor.move(30);
}

void RedRingC(){
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    clamp.set_value(true);
    raiseasdasd.set_value(false);
    chassis.moveToPoint(0, 7.5, 700, {.forwards=true, .maxSpeed = 100});  
    wallmotor.move(72);
    chassis.turnToHeading(-5, 700, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 127});
    pros::delay(175);
    pros::delay(150);
    wallmotor.move(-127);
    pros::delay(100);
    wallmotor.move(0);
    chassis.moveToPoint(0, 0, 700, {.forwards=false, .maxSpeed = 127}, false);  
    wallmotor.move(-127);

    chassis.turnToHeading(-68, 700, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 127}); 
    raiseasdasd.set_value(true);
    intake.move(127);
    chassis.moveToPoint(-11.7, 6.6, 700, {.forwards=true, .maxSpeed = 127});  
    wallmotor.move(-30);
    pros::delay(600);
    raiseasdasd.set_value(false);
    pros::delay(200);
    intake.move(0);
    chassis.turnToHeading(16, 800, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 80}); 
    wallmotor.move(0);
    raiseasdasd.set_value(false);

    chassis.moveToPoint(-15.9, -25, 1000, {.forwards=false, .maxSpeed = 80});  
    pros::delay(800);
    clamp.set_value(false);
    chassis.turnToHeading(124, 800, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 80}); 
    chassis.moveToPoint(-2.5, -43, 900, {.forwards=true, .maxSpeed = 127});  
    intake.move(127);

    chassis.moveToPoint(-18, -35.4, 800, {.forwards=false, .maxSpeed = 127});
    chassis.turnToHeading(181, 800, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 80}); 
    chassis.moveToPoint(-17.8, -47.5, 800, {.forwards=true, .maxSpeed = 55});
    chassis.moveToPoint(-17.6, -42.4, 500, {.forwards=false, .maxSpeed = 90});
    chassis.turnToHeading(97, 600, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 80}); 
    chassis.moveToPoint(-3.4, -41.9, 900, {.forwards=true, .maxSpeed = 55});
    intake.move(-127);
    chassis.turnToHeading(208, 800, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 80}); 
    pros::delay(200);
    intake.move(127);
    chassis.moveToPoint(-9, -53.5, 900, {.forwards=true, .maxSpeed = 127}, false);
    intake.move(127);

    chassis.turnToHeading(217, 500, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 127}); 

    chassis.moveToPoint(18, -32.1, 1000, {.forwards=false, .maxSpeed = 127});

    chassis.turnToHeading(88, 500, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 90}); 
}


void BlueRingL(){
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    clamp.set_value(true);
    raiseasdasd.set_value(false);
    //wallmotor.move(-127);
    chassis.moveToPoint(0, 7.4, 800, {.forwards=true, .maxSpeed = 100});  
    wallmotor.move(80);
    // chassis.turnToHeading(-5, 700, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 127});
    pros::delay(250);
    pros::delay(160);
    wallmotor.move(40);
    pros::delay(200);
    wallmotor.move(-40);
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
    pros::delay(600);
    raiseasdasd.set_value(false);
    pros::delay(200);
    intake.move(0);
    chassis.turnToHeading(-16, 800, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 80}); 
    wallmotor.move(0);
    raiseasdasd.set_value(false);

    //move to mogo to clamp, turn towards ring in front of center rings to intake 
    chassis.moveToPoint(15.9, -25, 1000, {.forwards=false, .maxSpeed = 80});  
    pros::delay(800);
    clamp.set_value(false);
    chassis.turnToHeading(-124, 800, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 80}); 
    //pros::delay(10000);
    chassis.moveToPoint(2.5, -43, 900, {.forwards=true, .maxSpeed = 127});  
    intake.move(127);
    //pros::delay(10000);

    //gets the ring in front of center rings, then gets rings on center
    chassis.moveToPoint(18, -35.4, 800, {.forwards=false, .maxSpeed = 127});
    chassis.turnToHeading(-181, 800, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 80}); 
    chassis.moveToPoint(17.8, -47.5, 800, {.forwards=true, .maxSpeed = 55});
    chassis.moveToPoint(17.6, -42.4, 500, {.forwards=false, .maxSpeed = 90});
    chassis.turnToHeading(-97, 600, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 80}); 
    chassis.moveToPoint(3.4, -41.9, 900, {.forwards=true, .maxSpeed = 55});
    intake.move(-127);
    chassis.turnToHeading(-208, 800, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 80}); 
    pros::delay(200);
    intake.move(127);
    chassis.moveToPoint(9, -53.5, 900, {.forwards=true, .maxSpeed = 127}, false);

    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    
    pros::delay(300);

    chassis.moveToPoint(40, -22, 900, {.forwards=false, .maxSpeed = 100}, false);

    wallmotor.move(127);

    move(-50, 0);

    pros::delay(200);

    wallmotor.move_velocity(0);

    pros::delay(130);

    move(0,0);

    //corner
    //raiseasdasd.set_value(false);
    // chassis.turnToHeading(39, 800, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 80}, false);
    // chassis.moveToPoint(12.5, -24.5, 1000, {.forwards=true, .maxSpeed = 40}, false);
    // chassis.turnToHeading(75.7, 700, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 80}, false);
    // chassis.moveToPoint(-35, -25, 1000, {.forwards=false, .maxSpeed = 100});  
    // wallmotor.move(30);
}

void RedRingL(){
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    clamp.set_value(true);
    raiseasdasd.set_value(false);
    //wallmotor.move(-127);
    chassis.moveToPoint(0, 7.4, 800, {.forwards=true, .maxSpeed = 100});  
    wallmotor.move(80);
    // chassis.turnToHeading(-5, 700, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 127});
    pros::delay(250);
    pros::delay(160);
    wallmotor.move(40);
    pros::delay(200);
    wallmotor.move(-40);
    pros::delay(100);
    wallmotor.move(0);
    chassis.moveToPoint(0, 0, 700, {.forwards=false, .maxSpeed = 127}, false);  
    wallmotor.move(-127);

    chassis.turnToHeading(-68, 700, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 127}); 
    raiseasdasd.set_value(true);
    intake.move(127);
    chassis.moveToPoint(-11.7, 6.6, 700, {.forwards=true, .maxSpeed = 127});  
    wallmotor.move(-30);
    pros::delay(600);
    raiseasdasd.set_value(false);
    pros::delay(200);
    intake.move(0);
    chassis.turnToHeading(16, 800, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 80}); 
    wallmotor.move(0);
    raiseasdasd.set_value(false);

    chassis.moveToPoint(-15.9, -25, 1000, {.forwards=false, .maxSpeed = 80});  
    pros::delay(800);
    clamp.set_value(false);
    chassis.turnToHeading(124, 800, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 80}); 
    chassis.moveToPoint(-2.5, -43, 900, {.forwards=true, .maxSpeed = 127});  
    intake.move(127);

    chassis.moveToPoint(-18, -35.4, 800, {.forwards=false, .maxSpeed = 127});
    chassis.turnToHeading(181, 800, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 80}); 
    chassis.moveToPoint(-17.8, -47.5, 800, {.forwards=true, .maxSpeed = 55});
    chassis.moveToPoint(-17.6, -42.4, 500, {.forwards=false, .maxSpeed = 90});
    chassis.turnToHeading(97, 600, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 80}); 
    chassis.moveToPoint(-3.4, -41.9, 900, {.forwards=true, .maxSpeed = 55});
    intake.move(-127);
    chassis.turnToHeading(208, 800, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 80}); 
    pros::delay(200);
    intake.move(127);
    chassis.moveToPoint(-9, -53.5, 900, {.forwards=true, .maxSpeed = 127}, false);

    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    
    pros::delay(300);

    chassis.moveToPoint(-40, -22, 900, {.forwards=false, .maxSpeed = 127}, false);

    wallmotor.move(127);

    move(-80, 0);

    pros::delay(200);

    wallmotor.move_velocity(0);

    pros::delay(130);

    move(0,0);
}

void RedMogoQUAL(){
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    clamp.set_value(true);
    raiseasdasd.set_value(false);
    //wallmotor.move(-127);
    chassis.moveToPoint(0, 7.4, 800, {.forwards=true, .maxSpeed = 100});  
    wallmotor.move(80);
    // chassis.turnToHeading(-5, 700, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 127});
    pros::delay(250);
    pros::delay(160);
    wallmotor.move(40);
    pros::delay(200);
    wallmotor.move(-40);
    pros::delay(100);
    wallmotor.move(0);
    chassis.moveToPoint(0, 0, 700, {.forwards=false, .maxSpeed = 127}, false);  
    wallmotor.move(-127);

    //turn towards ring in front of alliance stake, moves to get it, 
    chassis.turnToHeading(60, 700, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 127}); 
    raiseasdasd.set_value(true);
    intake.move(127);
    chassis.moveToPoint(13.7, 4.6, 700, {.forwards=true, .maxSpeed = 127});  
    wallmotor.move(-30);
    pros::delay(600);
    raiseasdasd.set_value(false);
    pros::delay(400);
    intake.move(0);
    chassis.turnToHeading(-16, 800, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 80}); 
    wallmotor.move(0);
    //move to mogo to clamp, turn towards ring in front of center rings to intake 
    chassis.moveToPoint(15.9, -25, 1000, {.forwards=false, .maxSpeed = 80});  
    pros::delay(800);
    clamp.set_value(false);
    chassis.turnToHeading(-124, 800, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 80}); 
    //pros::delay(10000);
    chassis.moveToPoint(2.5, -43, 900, {.forwards=true, .maxSpeed = 127});  
    intake.move(127);

    chassis.turnToHeading(-45.7, 890, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 40}, false);
    
    pros::delay(600);

    chassis.moveToPoint(-22.2, -19.7, 600, {.forwards=true, .maxSpeed = 127});  
    intake.move(-127);
    chassis.turnToHeading(0, 700, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 80}); 


    chassis.moveToPoint(-17.8, -30.7, 1100, {.forwards=false, .maxSpeed = 90});  

    chassis.turnToHeading(-85, 700, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 80}); 

    chassis.moveToPoint(-43, -29.7, 800, {.forwards=true, .maxSpeed = 100});  

    intake.move(127);
    pros::delay(800);
    move(0.4f, 0.0f);

    // chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

    chassis.moveToPoint(33.8, -25.5, 2000, {.forwards=false, .maxSpeed = 75});

    wallmotor.move(127);

    pros::delay(160);

    wallmotor.move_velocity(0);




}

void RedMogoELIM(){
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    clamp.set_value(true);
    raiseasdasd.set_value(false);
    //wallmotor.move(-127);
    chassis.moveToPoint(0, 7.4, 800, {.forwards=true, .maxSpeed = 100});  
    wallmotor.move(80);
    // chassis.turnToHeading(-5, 700, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 127});
    pros::delay(250);
    pros::delay(160);
    wallmotor.move(40);
    pros::delay(200);
    wallmotor.move(-40);
    pros::delay(100);
    wallmotor.move(0);
    chassis.moveToPoint(0, 0, 700, {.forwards=false, .maxSpeed = 127}, false);  
    wallmotor.move(-127);

    //turn towards ring in front of alliance stake, moves to get it, 

    chassis.turnToHeading(66.2, 700, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 127}); 

    intake.move(0);
    raiseasdasd.set_value(true);

    // pros::delay(9329432943294932943243);
    intake.move(127);
    chassis.moveToPoint(10.1, 7.3, 700, {.forwards=true, .maxSpeed = 90});  
    wallmotor.move(-30);
    pros::delay(400);
    raiseasdasd.set_value(false);
    pros::delay(600);
    intake.move(0);
    chassis.turnToHeading(-16, 800, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 70}); 
    wallmotor.move(0);
    //move to mogo to clamp, turn towards ring in front of center rings to intake 
    chassis.moveToPoint(15.9, -25, 1000, {.forwards=false, .maxSpeed = 80});  
    pros::delay(800);
    clamp.set_value(false);
    chassis.turnToHeading(-124, 800, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 80}); 
    //pros::delay(10000);
    chassis.moveToPoint(2.5, -43, 900, {.forwards=true, .maxSpeed = 127});  
    intake.move(127);

    chassis.turnToHeading(-45.7, 890, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 40}, false);
    
    pros::delay(600);

    chassis.moveToPoint(-22.2, -19.7, 600, {.forwards=true, .maxSpeed = 127});  
    intake.move(-127);
    chassis.turnToHeading(0, 700, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 80}); 


    chassis.moveToPoint(-17.8, -30.7, 1100, {.forwards=false, .maxSpeed = 90});  

    chassis.turnToHeading(-85, 700, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 80}); 

    chassis.moveToPoint(-43, -29.7, 800, {.forwards=true, .maxSpeed = 100});  

    intake.move(127);
    pros::delay(800);

    // pros::delay(9394329432493249324923424324324);

    chassis.moveToPoint(-15, -31, 700, {.forwards=false, .maxSpeed = 100}, false); 

    pros::delay(500);

    chassis.turnToHeading(-228, 700, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 127});

    pros::delay(500);

    clamp.set_value(true); 

    pros::delay(200);

    move(30, 0);

    pros::delay(200);

    move(0,0);

    // chassis.moveToPoint(1.9, -44.2 , 700, {.forwards=true, .maxSpeed = 100}); 

    chassis.turnToHeading(-402, 700, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 127});

    // angle: 

    // move(0.4f, 0.0f);

    // // chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

    // chassis.moveToPoint(33.8, -25.5, 2000, {.forwards=false, .maxSpeed = 75});

    // wallmotor.move(127);

    // pros::delay(160);

    // wallmotor.move_velocity(0);
}

void BlueMogoELIM(){
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    clamp.set_value(true);
    raiseasdasd.set_value(false);
    //wallmotor.move(-127);
    chassis.moveToPoint(0, 7.4, 800, {.forwards=true, .maxSpeed = 100});  
    wallmotor.move(80);
    // chassis.turnToHeading(5, 700, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 127});
    pros::delay(250);
    pros::delay(160);
    wallmotor.move(40);
    pros::delay(200);
    wallmotor.move(-40);
    pros::delay(100);
    wallmotor.move(0);
    chassis.moveToPoint(0, 0, 700, {.forwards=false, .maxSpeed = 127}, false);  
    wallmotor.move(-127);

    //turn towards ring in front of alliance stake, moves to get it, 

    chassis.turnToHeading(-66.2, 700, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 127}); 

    intake.move(0);
    raiseasdasd.set_value(true);

    // pros::delay(9329432943294932943243);
    intake.move(110);
    chassis.moveToPoint(-10.1, 7.3, 700, {.forwards=true, .maxSpeed = 90});  
    wallmotor.move(-30);
    pros::delay(400);
    intake.move(50);
    raiseasdasd.set_value(false);
    pros::delay(600);
    intake.move(0);
    chassis.turnToHeading(16, 800, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 70}); 
    wallmotor.move(0);
    //move to mogo to clamp, turn towards ring in front of center rings to intake 
    chassis.moveToPoint(-15.9, -25, 1000, {.forwards=false, .maxSpeed = 80});  
    pros::delay(800);
    clamp.set_value(false);
    chassis.turnToHeading(124, 800, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 80}); 
    //pros::delay(10000);
    chassis.moveToPoint(-2.5, -43, 900, {.forwards=true, .maxSpeed = 127});  
    intake.move(127);

    chassis.turnToHeading(45.7, 890, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 40}, false);
    
    pros::delay(600);

    chassis.moveToPoint(22.2, -19.7, 600, {.forwards=true, .maxSpeed = 127});  
    intake.move(-127);
    chassis.turnToHeading(0, 700, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 80}); 


    chassis.moveToPoint(17.8, -30.7, 1100, {.forwards=false, .maxSpeed = 90});  

    chassis.turnToHeading(85, 700, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 80}); 

    chassis.moveToPoint(43, -29.7, 800, {.forwards=true, .maxSpeed = 100});  

    intake.move(127);
    pros::delay(800);

    // pros::delay(9394329432493249324923424324324);

    chassis.moveToPoint(15, -31, 700, {.forwards=false, .maxSpeed = 100}, false); 

    pros::delay(500);

    // doinker.set_value(true); 
 
    chassis.turnToHeading(228, 700, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 127});

    pros::delay(500);

    clamp.set_value(true); 

    pros::delay(200);

    move(30, 0);

    pros::delay(200);

    move(0,0);

    // chassis.moveToPoint(-1.9, -44.2 , 700, {.forwards=true, .maxSpeed = 100}); 

    chassis.turnToHeading(402, 700, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 127});

    // angle: 

    // move(0.4f, 0.0f);

    // // chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

    // chassis.moveToPoint(-33.8, -25.5, 2000, {.forwards=false, .maxSpeed = 75});

    // wallmotor.move(127);

    // pros::delay(160);

    // wallmotor.move_velocity(0);
}

void RedMogoDoink(){
    chassis.moveToPoint(0, 42.4, 1000, {.forwards=true, .maxSpeed = 127}, true); 
    pros::delay(650);
    doinker.set_value(true);
    chassis.moveToPoint(0, 35, 500, {.forwards=false, .maxSpeed = 127});
    // intake_class.set_velocity(127);

}

void autonomous() {

    pros::delay(20);
    clamp.set_value(true);
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

    // BlueRingC();
    // BlueRingL();
    // RedMogo();

    // skillsAuton();
    // RedMogoDoink();

    // clamp.set_value(true);

    // intake_class.set_velocity(127);

    // pros::delay(5000);

    // clamp.set_value(false);



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
#define targetTop 164
#define targetDown 210
#define targetHold 70

void opcontrol() {

    // pros::delay(50);
    wallrot.set_position(0);
    bool doinker_state = false;
    bool doinker_button = false;
    doinker.set_value(false);

    left_front_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    left_back_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    left_center_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    right_front_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    right_back_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    right_center_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    wallmotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    
    // wallstage = 4;
    int pos = 0;
    bool clamped2 = false;
    float targetpos = targetWait;
    int stage = 0;
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

        if(manual){goto skp;}

        ladybrown_class.set_angle(targetpos);

        targetpos = (stage == 1) * targetWait + (stage == 2) * targetTop + (stage == 3) * targetDown + (stage == 4) * targetHold;
        
        goto skp2;
        
        skp:;

        skp2:;

        bool clampbutton = master.get_digital_new_press(DIGITAL_R1);

        doinker_button = master.get_digital_new_press(DIGITAL_A);

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

        if(doinker_button){
            doinker_state = !doinker_state;
            doinker.set_value(doinker_state);
        }        

        if (!clamped2){
            master.set_text(3, 0, ".");
        }
        else{
            master.set_text(3, 0, "");
        }
        
        chassis.arcade(power, turn);

        if (intakebutton){
            outtaking = false;
            intaking = !intaking; 
        }
        else if (outtakebutton){
            intaking = false;
            outtaking = !outtaking;
        }

        if (outtaking){
            intake_class.set_velocity(127);
            intake2.move(127);
        }
        if (intaking){
            intake_class.set_velocity(-127);
            intake2.move(-127);
        }
        if(!intaking && !outtaking){
            intake_class.set_velocity(0);
            intake2.move(0);
        }
    }
}