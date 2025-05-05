#include "main.h"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "lemlib/pose.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include <atomic>
#include <cmath>
#include <csignal>
#include <cstdio>

#include "subsystems/Intake.h"
#include "subsystems/Ladybrown.h"


// #define six_ball 0
// #define far_safe_awp 1
// #define close_rush 2
// #define close_safe_awp 3

// angles: stage 1 - 3000, 

// ASSET(path_txt);

#define RED_RING_RUSH 0
#define RED_MOGO 1
#define SKILLS_AUTO 2
#define BLUE_RING_NOT_DONE 3
#define BLUE_MOGO_ELIM 4

char autonRunning = RED_RING_RUSH;

const char* autonNames[] = {"RED RING", "RED_MOGO", "BLUE_RING", "BLUE_MOGO"};

/*
DRIVE MOTORS
*/

pros::Motor left_front_motor(6, pros::E_MOTOR_GEAR_BLUE, false);
pros::Motor left_center_motor(17, pros::E_MOTOR_GEAR_BLUE , false);
pros::Motor left_back_motor(3, pros::E_MOTOR_GEAR_BLUE , false); 
pros::Motor right_front_motor(10, pros::E_MOTOR_GEAR_BLUE , true);
pros::Motor right_center_motor(2, pros::E_MOTOR_GEAR_BLUE , true);
pros::Motor right_back_motor(1, pros::E_MOTOR_GEAR_BLUE , true);

pros::Motor intake(20, pros::E_MOTOR_GEAR_BLUE, false);

pros::Imu inertial_sensor(5);
pros::Optical colorSensor(15);
pros::Distance intakeDistSensor(16);


/*
SUBSYSTEMS
*/

pros::Motor intake2(0);

Intake intake_class;

pros::Motor wallmotor(11, pros::E_MOTOR_GEAR_RED , true);

Ladybrown ladybrown_class;

/*
PNEUMATICS
*/

/*
PNEUMATICS
*/

pros::ADIDigitalOut clamp('A'); //backwings H, F
pros::ADIDigitalOut hangs('Z');
pros::ADIDigitalOut doinker('B');
pros::ADIDigitalOut doinker2('C');
pros::ADIDigitalOut raiseasdasd('D');
pros::ADIDigitalOut leftdoinker('D');
pros::ADIAnalogOut doinkerclamp('E');

/*
DRIVE GROUPS
*/

pros::MotorGroup left_side_motors({left_front_motor, left_center_motor, left_back_motor});
pros::MotorGroup right_side_motors({right_front_motor, right_center_motor, right_back_motor});

lemlib::Drivetrain drivetrain {
 &left_side_motors, // left drivetrain motors
 &right_side_motors, // right drivetrain motors
 14, // track width
 lemlib::Omniwheel::NEW_325_HALF, // wheel diameter
 600, // wheel rpm
 0
};

/*
SENSORS
*/

pros::Rotation wallrot(4, false); 

pros::Rotation horizontal_rot(19); // port 1, not reversed
pros::Rotation vertical_rot(18); // port 1, not reversed

lemlib::TrackingWheel horizontal_track(&horizontal_rot, lemlib::Omniwheel::NEW_275 , -0.8f); // 0.6 -0.9
lemlib::TrackingWheel vertical_track(&vertical_rot, lemlib::Omniwheel::NEW_2 , -0.53f); // 0.6 -0.

pros::Distance distance_sensor(18);
pros::Optical color_sort(15);

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

lemlib::ControllerSettings lateralController(5.8, // proportional gain (kP)
 0.0, // integral gain (kI)
 26, // derivative gain (kD)
 4, // anti windup
 0.2, // small error range, in inches
 1000, // small error range timeout, in milliseconds
 2, // large error range, in inches
 3000, // large error range timeout, in milliseconds
 0 // maximum acceleration (slew)
);




lemlib::ControllerSettings angularController(3.25, // proportional gain (kP)
 0, // integral gain (kI)
 38, // derivative gain (kD)
 3, // anti windup
 0.01, // small error range, in inches
 1000, // small error range timeout, in milliseconds
 0.2, // large error range, in inches
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
bool autoclamp_bool = false;
int counter;
lemlib::Pose bot_pos = lemlib::Pose(0,0,0);

void getPose(){
 bot_pos = chassis.getPose();
}

void setPose(double _x, double _y){
 chassis.setPose(lemlib::Pose(_x, _y, chassis.getPose().theta));
}

inline double getAngleToPoint(const double& x, const double& y){
 const lemlib::Pose p = chassis.getPose();

 const double dy = y - p.y;
 const double dx = x - p.x;

 if(dy == 0){return 0;}

 const double ref = atan((dx)/dy)/3.141592653 * 180;

 const double offset = (dy < 0) ? ((dx < 0) ? 180 : 90) : 0;

 return ref + offset;
}


void move(double power, double turn, bool swing=false) {
 int left = power + turn;
 int right = power - turn;

 if (swing && left < 0) {left = 0;}
 if (swing && right < 0) {right = 0;}

 left_front_motor.move(left);
 left_center_motor.move(left);
 left_back_motor.move(left);
 right_front_motor.move(right);
 right_center_motor.move(right);
 right_back_motor.move(right);
}

void clamp_thread(){
 while(true){
 if (autoclamp_bool && distance_sensor.get() < 38){
 clamp.set_value(false);
 }
 pros::delay(10);
 }
}

void forDistance() {
 int dist = distance_sensor.get();
 //if(dist <= )
}

/*
INTAKE THREAD
*/
#define RED 17
#define BLUE 212
#define DRIVER -6

std::atomic_int badcolor;

#define inColor(c) (c <= badcolor+5 && c >= badcolor-5)

void intake_thread(){
 while(true){
 intake_class.update(intake.get_torque());
 if(inColor(color_sort.get_hue())){
    
 intake.move(127);
    pros::delay(50);
 intake.move(0);
 pros::delay(200);
 continue;
 }
 intake.move(intake_class.get_velocity());
 pros::delay(10);
 }
}

void redColorSort() {
 colorSensor.set_integration_time(5);
 counter++;
 if(colorSensor.get_hue() >= 17 && counter >= 7)
 {
 if(intakeDistSensor.get() <= 0)
 {
 intake.move(0);
 pros::delay(300);
 }
 }
 //intakeDistSensor.get();
}

void blueColorSort() {
 //stop blue rings from being intaked
 //200-250
 colorSensor.set_integration_time(5);
 counter++;
 if(colorSensor.get_hue() <= 200 && counter >= 7)
 {
 if(intakeDistSensor.get() <= 0)
 {
 intake.move(0);
 pros::delay(300);
 }
 }
 //intakeDistSensor.get();
 // printf("Hue value: %lf \n", colorSensor.get_hue());
}

/*
LADYBROWN THREAD
*/

#define targetWait 40.2
#define targetTop 190
#define targetDown 237
#define targetHold 80
bool moveDown = false;

// bool ladybrown_manual = false;
std::atomic_bool ladybrown_manual(false);
std::atomic_bool ladybrown_slow(false);

inline double clamp123(const double val){
 return (val > 127) ? 127 : val;
}

void Ladybrown_thread(){
 while(true){
 ladybrown_class.update(wallrot.get_position());
 if (!ladybrown_manual.load()){
 wallmotor.move(ladybrown_class.get_velocity(false) * (ladybrown_slow ? 0.3f: 1.0f));
 }
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
 // std::cout << "curPos: " << curPos.toString() << ", targetHeading: " << targetHeading << ", "
 // << "turnCompleteBuff: " << turnCompleteBuff << ", "
 // << "headingErr: " << headingErr << ", turnSpeed: " << turnSpeed
 // << std::endl;
 // printf("turning turn function\n");
 // }

 if(std::abs(turnSpeed) > maxSpeed) {turnSpeed = turnSpeed < 0 ? -maxSpeed : maxSpeed; }

 move(0, turnSpeed, swing);
 }

 printf("turn done");
 move(0, 0);
}

void negaYogineni(){
    intake_class.set_velocity(127);
    clamp.set_value(true);
    chassis.moveToPoint(0, 25, 2000, {.maxSpeed = 80});
    badcolor = RED;

}

void bakerAuton(){
 ladybrown_slow = true;
 ladybrown_class.set_angle(155);
 ladybrown_slow = false;
 autoclamp_bool = false;
 clamp.set_value(false);
 pros::delay(600);
 chassis.moveToPoint(0, -7, 500);
 chassis.turnToHeading(52,600);
 chassis.moveToPoint(-18.6, -25.9, 600, {.forwards=false, .earlyExitRange = 8}); 
 chassis.moveToPoint(-25, -31.8, 1000, {.forwards=false, .maxSpeed = 60}); 
 pros::delay(500);
 clamp.set_value(true);
 ladybrown_class.set_angle(-40);
 pros::delay(600);
 wallrot.set_position(0);
 ladybrown_class.set_angle(0);
 chassis.turnToPoint(-33.3, -24.5,1000, {.forwards = true});
 chassis.moveToPoint(-33.3, -24.5, 1100, {.forwards = true}, false);
 //doinker.set_value(true);
 pros::delay(400);
 chassis.turnToPoint(-43, -25.4,1000, {.forwards = true});
 chassis.moveToPoint(-43, -25.4, 1100, {.forwards = true}, false);
 chassis.turnToPoint(-48, -26.9,1000, {.forwards = true});
 chassis.turnToHeading(-107.3, 1000);
 doinker.set_value(true);
 chassis.moveToPoint(-48, -26.9, 1100, {.forwards = true}, false);

 // -27.8 -23.6
 chassis.turnToPoint(-27.8, -23.6,1000, {.forwards = false});
 chassis.moveToPoint(-27.8 ,-23.6, 1100, {.forwards = false}, false);
 intake_class.set_velocity(127);
 // -37.5 -21.6
 chassis.turnToPoint(-37.5, -21.6,1000, {.forwards = true});
 chassis.moveToPoint(-37.5 ,-21.6, 1100, {.forwards = true}, false);
 // -26.4 ,-23.9
 chassis.turnToPoint(-26.4 ,-23.9,1000, {.forwards = false});
 chassis.moveToPoint(-26.4 ,-23.9, 1100, {.forwards = false}, false);
 // -13.4, -44.8 
 chassis.turnToPoint(-13.4, -44.8 ,1000, {.forwards = true});
 chassis.moveToPoint(-13.4, -44.8 , 1100, {.forwards = true}, false);
}

void initialize() {
 pros::lcd::initialize(); // initialize brain screen
 horizontal_rot.reset_position();
 wallmotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
 inertial_sensor.reset();
 wallrot.set_position(0);
 color_sort.set_led_pwm(100);
 color_sort.set_integration_time(3);
 chassis.calibrate(); // calibrate the chassis
 chassis.setPose(0, 0, 0); // X: 0, Y: 0, Heading: 0
 hangs.set_value(false);
 colorSensor.set_led_pwm(100);
 pros::Task screenTask(screen); 
 pros::Task intakeTask(intake_thread);
 pros::Task ladybrownTask(Ladybrown_thread);
 pros::Task clampTask(clamp_thread);
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


void soloWP(){
 ladybrown_slow = true;
 ladybrown_class.set_angle(155);
 ladybrown_slow = false;
 autoclamp_bool = false;
 clamp.set_value(false);
 pros::delay(600);
 chassis.moveToPoint(0, -7, 500);
 chassis.turnToHeading(-52,600);
 chassis.moveToPoint(18.6, -25.9, 600, {.forwards=false, .earlyExitRange = 8}); 
 chassis.moveToPoint(25, -31.8, 600, {.forwards=false, .maxSpeed = 90}); 
 pros::delay(300);
 clamp.set_value(true);
 ladybrown_class.set_angle(-200);
 chassis.turnToHeading(-144,600);
 pros::delay(600);
 wallrot.set_position(0);
 ladybrown_class.set_angle(0);
 intake_class.set_velocity(127);
 chassis.moveToPoint(13.7, -55.1, 1100);
 chassis.turnToHeading(-81.9,600);
 chassis.moveToPoint(-28,-43.5 , 1100);
 chassis.waitUntilDone();
 move(127, 0);
 pros::delay(700);
 chassis.moveToPoint(2.7, -44.7, 800, {.forwards=false, .maxSpeed = 127}, false);
 // chassis.turnToHeading(10,600);
 chassis.turnToPoint(27.2, 30.6, 600);
 // chassis.moveToPoint(22.6,-2.4 , 1100);
 pros::delay(500);
 clamp.set_value(false);
 chassis.moveToPoint(27.2, 30.6, 2000, {.maxSpeed = 80});


 pros::delay(200);
 colorSensor.set_led_pwm(100);
 while(colorSensor.get_hue() >= 20)
 {
 ;
 }
 intake_class.set_velocity(-60);
 pros::delay(100);
 intake_class.set_velocity(0);

 // chassis.turnToPoint(47.1,18.1,1000, {.forwards = false});
 chassis.turnToHeading(-61, 1000);
 chassis.moveToPoint(47.1,18.1, 500, {.forwards=false, .earlyExitRange = 8});
 chassis.moveToPoint(53.8,14.1, 2000, {.forwards=false, .maxSpeed = 60});


 pros::delay(500);
 clamp.set_value(true);

 intake_class.set_velocity(127);
 chassis.turnToPoint(70, 36.7,600);
 chassis.moveToPoint(70, 36.7, 1100);
 
 ladybrown_class.set_angle(60);
 
 chassis.turnToPoint(63.5, -2.1,600, {.forwards = false});
 chassis.moveToPoint(63.5, -2.1, 1100, {.forwards = false});











 

 // chassis.turnToHeading(45,600,{},false);
 //









}

void ladyBrownAutonNegativeRed(){
 ladybrown_slow = true;
 ladybrown_class.set_angle(80);
 chassis.moveToPoint(0,30, 2000, {.forwards=true, .earlyExitRange = 8});
 pros::delay(500);
 intake_class.set_velocity(127);
 colorSensor.set_led_pwm(100);
 int cnt = 0;
 while(colorSensor.get_hue() >= 20 && cnt < 100)
 {
 pros::delay(10);
 cnt++;
 }
 intake_class.set_velocity(-60);
 pros::delay(100);
 intake_class.set_velocity(0);

 chassis.turnToPoint(-15.5, 47.3, 1000);

 chassis.moveToPoint(-15.5,47.6, 2000, {.forwards=true, .maxSpeed = 70, .earlyExitRange = 8}, false);
 chassis.turnToHeading(-36.3, 300);
 ladybrown_slow = false;
 ladybrown_class.set_angle(160);

 pros::delay(600);
 ladybrown_class.set_angle(-40);

 chassis.moveToPoint(-5.2, 33.3, 2000, {.forwards=false, .maxSpeed = 127, .earlyExitRange = 8});

 pros::delay(800);
 wallrot.set_position(0);
 ladybrown_class.set_angle(0);
 
 chassis.turnToPoint(13.6, 30.3, 1000, {false});
 chassis.moveToPoint(13.6, 30.3, 2000, {.forwards=false, .maxSpeed = 127, .earlyExitRange = 8});
 chassis.moveToPoint(19, 30.4, 2000, {.forwards=false, .maxSpeed = 70, .earlyExitRange = 8}, false);
 clamp.set_value(true);
 pros::delay(200);
 intake_class.set_velocity(127);
 chassis.turnToPoint(8, 48.5, 1000);
 chassis.moveToPoint(8,48.5, 2000, {.forwards=true, .maxSpeed = 70, .earlyExitRange = 8}, false);

 chassis.moveToPoint(23.3,31.5, 2000, {.forwards=false, .maxSpeed = 127, .earlyExitRange = 8}, false);

 chassis.turnToPoint(-17.7, -10.7, 2000);
 chassis.moveToPoint(-17.7,-10.7, 2000, {.forwards=true, .maxSpeed = 80, .earlyExitRange = 8}, false);
 move(127, 0);
 pros::delay(500);

 chassis.moveToPoint(-6,2.1, 2000, {.forwards=false, .maxSpeed = 127, .earlyExitRange = 8}, false);

 chassis.turnToPoint(52.3, 5.8, 1000);
 raiseasdasd.set_value(true);
 intake_class.set_velocity(0);
 chassis.moveToPoint(52.3,5.8, 650, {.forwards=true, .maxSpeed = 127, .earlyExitRange = 8}, false);
 chassis.moveToPoint(52.3,6, 1000, {.forwards=true, .maxSpeed = 30, .earlyExitRange = 8});
 pros::delay(600);
 intake_class.set_velocity(127);
 raiseasdasd.set_value(false);
 pros::delay(700);
 chassis.moveToPoint(41.6,6, 1000, {.forwards=false, .maxSpeed = 30, .earlyExitRange = 8}, false);
 chassis.moveToPoint(52.3,6, 1000, {.forwards=true, .maxSpeed = 60, .earlyExitRange = 8}, false);
 

 chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
 //while(true){}
}

void SixRingBlueRingside(){
 ladybrown_slow = true;
 ladybrown_class.set_angle(155);
 ladybrown_slow = false;
 autoclamp_bool = false;
 clamp.set_value(false);
 pros::delay(600);
 chassis.moveToPoint(0, -7, 500);
 chassis.turnToHeading(52,600);
 chassis.moveToPoint(-18.6, -25.9, 600, {.forwards=false, .earlyExitRange = 8}); 
 chassis.moveToPoint(-25, -31.8, 1000, {.forwards=false, .maxSpeed = 60}); 
 pros::delay(500);
 clamp.set_value(true);
 ladybrown_class.set_angle(-40);
 intake_class.set_velocity(127);
 chassis.turnToPoint(-37.6, -48.7,1000);
 pros::delay(800);
 wallrot.set_position(0);
 ladybrown_class.set_angle(0);
 chassis.moveToPoint(-37.6, -46, 1000, {.forwards=true, .maxSpeed = 90}); 
 chassis.turnToPoint(-29, -56,1000);
 chassis.moveToPoint(-31, -56, 1000, {.forwards=true, .maxSpeed = 90}); 
 chassis.turnToPoint(-27, -66,1000);
 chassis.moveToPoint(-27, -66, 1000, {.forwards=true, .maxSpeed = 60}); 
}


void fivePlusOneBlue(){
 badcolor = RED;
 ladybrown_slow = true;
 ladybrown_class.set_angle(155);
 ladybrown_slow = false;
 autoclamp_bool = false;
 clamp.set_value(false);
 pros::delay(500);
 chassis.moveToPoint(0, -7, 500);
 chassis.turnToHeading(-52,600);
 chassis.moveToPoint(18.6, -25.9, 500, {.forwards=false, .earlyExitRange = 12}); 
 chassis.moveToPoint(25, -31.8, 400, {.forwards=false, .maxSpeed = 50}); 
 pros::delay(500);
 clamp.set_value(true);
 

 

 //pros::delay(10100101010101);
 chassis.turnToPoint(36, -50.5,800);
 ladybrown_class.set_angle(-40);
 pros::delay(200);
 intake_class.set_velocity(127);
 pros::delay(350);
 wallrot.set_position(0);
 pros::delay(20);
 //ladybrown_class.set_angle(0);
 ladybrown_class.rest();
 
 chassis.moveToPoint(36, -50.5, 800, {.forwards=true, .maxSpeed = 90}, false);

 // 23.1, -61.5
 chassis.turnToPoint(32.4, -62,500);
 
 chassis.moveToPoint(32.4, -62, 500, {.forwards=true, .maxSpeed = 80}, false);

 chassis.turnToPoint(37.3, -46.8,500, {false});
 
 chassis.moveToPoint(37.3, -46.8, 650, {.forwards=false, .maxSpeed = 100}, false);
 
 // 20.8 -52.2
 chassis.turnToPoint(20.8, -52.2,400);
 
 chassis.moveToPoint(20.8 ,-52.2, 600, {.forwards=true, .maxSpeed = 127}, false);
 
 // -10.5 -36.1

 chassis.turnToPoint(-4.7, -39.6, 600, {true});
 chassis.moveToPoint(-4.7, -39.6, 800, {.forwards=true, .maxSpeed = 127}); 

 
 // -105.3 hdg

 chassis.turnToPoint(-25, -46, 600);
 // -30 -39
 chassis.moveToPoint(-25, -46, 600, {.forwards=true, .maxSpeed = 127}, false);

 move(90, 0);
 pros::delay(1000);

 chassis.moveToPoint(-5, -39, 700, {.forwards=false, .maxSpeed = 127}); 


 //23.1, -1
 chassis.turnToPoint(18, -2, 850, {true}, false);
 raiseasdasd.set_value(true);
 chassis.moveToPoint(18, -2.5, 450, {.forwards=true, .maxSpeed = 127}, false);
 chassis.moveToPoint(18, -2.5, 450, {.forwards=true, .maxSpeed = 60}, false);
 raiseasdasd.set_value(false);
 pros::delay(300);
 chassis.moveToPoint(18, -8, 1000, {.forwards=false, .maxSpeed = 60}, false);
 
}

void fivePlusOneRed(){
 badcolor = BLUE;
 ladybrown_slow = true;
 ladybrown_class.set_angle(155);
 ladybrown_slow = false;
 autoclamp_bool = false;
 clamp.set_value(false);
 pros::delay(500);
 chassis.moveToPoint(0, -7, 500);
 chassis.turnToHeading(52,600);
 chassis.moveToPoint(-18.6, -25.9, 500, {.forwards=false, .earlyExitRange = 12}); 
 chassis.moveToPoint(-25, -31.8, 400, {.forwards=false, .maxSpeed = 50}); 
 pros::delay(500);
 clamp.set_value(true);
 

 

 //pros::delay(10100101010101);
 chassis.turnToPoint(-26.3, -52.5,800);
 ladybrown_class.set_angle(-40);
 pros::delay(200);
 intake_class.set_velocity(127);
 pros::delay(500);
 wallrot.set_position(0);
 ladybrown_class.set_angle(0);
 
 chassis.moveToPoint(-26.3, -52.5, 800, {.forwards=true, .maxSpeed = 90}, false);

 // 23.1, -61.5
 chassis.turnToPoint(-24.8, -59.7,500);
 
 chassis.moveToPoint(-24.8, -59.7, 500, {.forwards=true, .maxSpeed = 80}, false);

 chassis.turnToPoint(-29.3, -46.8,500, {false});
 
 chassis.moveToPoint(-29.3, -46.8, 650, {.forwards=false, .maxSpeed = 100}, false);
 
 // 20.8 -52.2
 chassis.turnToPoint(-12.6, -48,400);
 
 chassis.moveToPoint(-12.6, -48, 600, {.forwards=true, .maxSpeed = 127}, false);
 
 // -10.5 -36.1

 chassis.turnToPoint(20, -39.6, 600, {true});
 chassis.moveToPoint(20, -39.6, 800, {.forwards=true, .maxSpeed = 127}); 

 
 // -105.3 hdg

 chassis.turnToPoint(44, -46, 600);
 // -30 -39
 chassis.moveToPoint(44, -46, 600, {.forwards=true, .maxSpeed = 127}, false);

 move(90, 0);
 pros::delay(1000);

 chassis.moveToPoint(14, -39, 700, {.forwards=false, .maxSpeed = 127}); 


 //23.1, -1
 chassis.turnToPoint(-13, -1, 850, {true}, false);
 raiseasdasd.set_value(true);
 chassis.moveToPoint(-13.5, -1.5, 450, {.forwards=true, .maxSpeed = 127}, false);
 chassis.moveToPoint(-13.5, -1.5, 450, {.forwards=true, .maxSpeed = 60}, false);
 raiseasdasd.set_value(false);
 pros::delay(300);
 chassis.moveToPoint(-15, -7, 1000, {.forwards=false, .maxSpeed = 60}, false);
 
}


void bakerRed(){
 badcolor = BLUE;
 ladybrown_slow = true;
 ladybrown_class.set_angle(155);
 ladybrown_slow = false;
 autoclamp_bool = false;
 clamp.set_value(false);
 pros::delay(500);
 chassis.moveToPoint(0, -7, 500);
 chassis.turnToHeading(-52,600);
 chassis.moveToPoint(18.6, -25.9, 500, {.forwards=false, .earlyExitRange = 12}); 
 chassis.moveToPoint(25, -31.8, 400, {.forwards=false, .maxSpeed = 50}); 
 pros::delay(500);
 clamp.set_value(true);
 ladybrown_class.set_angle(-40);
 pros::delay(200);
 intake_class.set_velocity(127);
 pros::delay(500);
 wallrot.set_position(0);
 ladybrown_class.set_angle(0);
 intake_class.set_velocity(0);
 // 47.9, -23.4
 chassis.turnToPoint(47.1, -23.5, 700, {true}, false);
 
 chassis.moveToPoint(47.1, -23.5, 900, {.forwards=true, .maxSpeed = 127}, false);
 doinker.set_value(true);
 // 51.6, -24.3
 // turn first then doink
 // 22.5, -27.9
 chassis.turnToPoint(22.5, -27.9, 600, {false}, false);
 
 chassis.moveToPoint(22.5, -27.9, 800, {.forwards=false, .maxSpeed = 127}, false);
 doinker.set_value(false);
 pros::delay(200);
 intake_class.set_velocity(127);
 // BACKWARDS
 // un doink after
 // start intaking
 // 37.2, -30.5
 chassis.turnToPoint(37.2, -30.5, 700, {true}, false);
 
 chassis.moveToPoint(37.2, -30.5, 1000, {.forwards=true, .maxSpeed = 127}, false);
 // 16.7, -51.3
 chassis.turnToPoint(16.7, -53.5, 700, {true}, false);
 
 chassis.moveToPoint(16.7, -53.5, 1000, {.forwards=true, .maxSpeed = 127}, false);
 // -8.6, -36.3
 chassis.turnToPoint(-20, -48, 700, {true}, false);
 
 chassis.moveToPoint(-20, -48, 1000, {.forwards=true, .maxSpeed = 127}, false);
 ladybrown_class.set_angle(40);
 move(127, 0);
 pros::delay(700);
 move(0,0);
 
 intake_class.set_velocity(90);
 

 //-7.2, -37.8
 chassis.turnToPoint(12.3, -39, 500, {false}, false);
 
 chassis.moveToPoint(12.3, -39, 1000, {.forwards=false, .maxSpeed = 127});
 
 
 

 //21.2, -73.2
 chassis.turnToPoint(24.1, -71.7, 800, {true}, false);
 
 chassis.moveToPoint(24.1, -71.7, 1000, {.forwards=true, .maxSpeed = 60}, false);

 chassis.turnToHeading(173, 200);
 intake_class.set_velocity(0);
 ladybrown_class.set_angle(149);



 // ram

}

void bakerBlueL(){
 badcolor = RED;
 raiseasdasd.set_value(false);
 ladybrown_class.set_angle(155);
 autoclamp_bool = false;
 clamp.set_value(false);
 
 chassis.moveToPoint(0, 7, 500);
 pros::delay(500);
 
 chassis.moveToPoint(0, -25, 500, {.forwards=false, .maxSpeed = 80, .earlyExitRange = 12}); 
 chassis.moveToPoint(0, -27, 800, {.forwards=false, .maxSpeed = 50, .minSpeed = 10}); 
 
 pros::delay(550);
 clamp.set_value(true);
 ladybrown_class.set_angle(-40);

 

 // -19, -39
 //-106
 //chassis.turnToPoint(-20.4, -37.2, 800);

 // pros::delay(3949249234992434342343434234);

 chassis.turnToPoint(-18.9, -35.7, 700);
 //chassis.turnToPoint(-20.5, -36.3, 700);
 pros::delay(500);
 wallrot.set_position(0);
 ladybrown_class.set_angle(0);
 chassis.waitUntilDone();
 chassis.angularPID.reset();
 
 chassis.moveToPoint(-18.9, -35.7, 750, {.maxSpeed = 80}, false);
 // -1, 11
 doinker2.set_value(true);
 pros::delay(300);
 // pros::delay(342493294234923432249242394324234424234);
 // chassis.turnToHeading(-128, 500);
 // chassis.moveToPoint(-22, -39.4, 400, {.maxSpeed = 127, .minSpeed = 50}, false);
 // doinker.set_value(true);
 
 
 // pros::delay(300);
 // chassis.turnToPoint(13.9, -19.9, 200, {false});
 // chassis.waitUntilDone();

 chassis.moveToPoint(13.9, -19.9, 975, {.forwards = false, .maxSpeed = 95}, false);

 chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
 
 doinker2.set_value(false);
 doinker.set_value(false);
 pros::delay(110101010101010);
 pros::delay(300);

 intake_class.set_velocity(127);

 
 chassis.turnToPoint(2, -34, 500);
 chassis.moveToPoint(2, -34, 800, {}, false);
 // chassis.turnToHeading(-220, 500, {}, false);
 // doinker.set_value(false);
 // pros::delay(400);
 // // 9, -40.3
 // chassis.turnToPoint(7.5, -44, 500, {.minSpeed = 50});
 // chassis.moveToPoint(7.5, -44, 700, {}, false);
 // 26.2, -49.7

 // pros::delay(324249324324324234);

 chassis.turnToPoint(18, -43.7, 700);
 chassis.moveToPoint(18, -43.7, 900, {}, false);


 // pros::delay(32942394943294242342342423);

 chassis.turnToPoint(33.8, -23.1, 600);
 chassis.moveToPoint(33.8, -23.1, 950, {}, false);
 
 // -23.9, -40.8
 chassis.turnToPoint(51.3, -17.6, 700);

 // pros::delay(329494942423434234234224);

 chassis.moveToPoint(51.3, -17.6, 700, {.maxSpeed = 60, .minSpeed = 22}, false);

 pros::delay(300);

 move(60, 0);

 pros::delay(600);

 move(0,0);

 // pros::delay(0123021302021032132132132321323);
 chassis.moveToPoint(46.6, -18.7, 500, {.forwards = false, .maxSpeed = 30, .minSpeed = 10}, false);
 move(-40, 0);
 pros::delay(300);
 move(0, 0);
 raiseasdasd.set_value(true);
 intake_class.set_velocity(127);

 chassis.waitUntilDone();

 pros::delay(200);

 move(80, 0);

 pros::delay(250);

 raiseasdasd.set_value(false);

 pros::delay(250);

 move(-50, 0);

 pros::delay(500);

 move(0,0);

 doinker.set_value(true);

 move(80,0);

 pros::delay(200);

 move(0,0);

 pros::delay(400);
 
 // pros::delay(9942934923934924394432424);

 chassis.turnToHeading(-108, 1200, {}, false);

 doinker.set_value(false);

 clamp.set_value(false);

 chassis.moveToPoint(-4.7, -36, 1200, {.minSpeed = 20});

 ladybrown_class.set_angle(170);

 pros::delay(9942934923934924394432424);

}

void doublebakerBlueL(){

 badcolor = RED;
 raiseasdasd.set_value(false);
 ladybrown_class.set_angle(150);
 autoclamp_bool = false;
 clamp.set_value(false);
 
 chassis.moveToPoint(0.3, 7, 500);
 pros::delay(500);
 
 chassis.moveToPoint(0, -25, 400, {.forwards=false, .maxSpeed = 80}); 
 chassis.moveToPoint(0, -27, 800, {.forwards=false, .maxSpeed = 50, .minSpeed = 10}); 
 
 pros::delay(550);
 clamp.set_value(true);
 ladybrown_class.set_angle(-40);

 

 // -19, -39
 //-106
 //chassis.turnToPoint(-20.4, -37.2, 800);

 // pros::delay(3949249234992434342343434234);

 chassis.turnToPoint(-20, -36, 800, {.maxSpeed = 80});
 //chassis.turnToPoint(-20.5, -36.3, 700);
 pros::delay(500);
 wallrot.set_position(0);
 ladybrown_class.set_angle(0);
 chassis.waitUntilDone();
 
 chassis.moveToPoint(-20, -36, 850, {.maxSpeed = 60}, false);

 // pros::delay(3294923942394234234242344);
 // -1, 11
 doinker2.set_value(true);
 pros::delay(300);
 // pros::delay(342493294234923432249242394324234424234);
 chassis.turnToHeading(-134, 550);
 chassis.moveToPoint(-24.5, -39.5, 450, {.maxSpeed = 127, .minSpeed = 50}, true);
 pros::delay(150);
 doinker.set_value(true);

 // pros::delay(92394932493424324234234);
 
 // pros::delay(4392349234324234234234234);
 // pros::delay(300);
 // chassis.turnToPoint(13.9, -19.9, 200, {false});
 // chassis.waitUntilDone();

 chassis.turnToPoint(11.44, -9.2, 400, {.forwards = false});

 chassis.moveToPoint(11.44, -9.2, 1500, {.forwards = false, .maxSpeed = 80}, false);

 doinker2.set_value(false);
 doinker.set_value(false);

 intake_class.set_velocity(127);
 pros::delay(300);
 

 // -12.4, 3.6

 // 20.7, -44.1


 chassis.turnToPoint(-11.5, 2, 700);



 chassis.moveToPoint(-11.5, 2, 400, {.maxSpeed = 100, .earlyExitRange = 7});

 chassis.moveToPoint(-20.7, 2.7, 800, {.maxSpeed = 40});
    
    
 chassis.turnToPoint(-3.6, -17.8, 1000);
 chassis.moveToPoint(20.7, -44.1, 2000, {.maxSpeed = 45});
// pros::delay(1010101010101010101010010101011);
 // pros::delay(32949234923423242342434234);

 chassis.turnToPoint(35.6, -29.3, 650);
 chassis.moveToPoint(35.6, -29.3, 900, {.maxSpeed = 127}, false);
  chassis.turnToPoint(53.8, -23, 650);
 chassis.moveToPoint(53.8, -23, 900, {.maxSpeed = 127}, false);

 move(90, 0);

 pros::delay(500);

 move(-60,0);

 pros::delay(300);

 move(0,0);

 intake_class.set_velocity(0);

 pros::delay(10101010111);

 pros::delay(600);

 chassis.turnToHeading(-102.8, 800);

 doinker.set_value(false);

 pros::delay(400);

 chassis.turnToPoint(-1.4, -9.1, 700);

 intake_class.set_velocity(127);

 chassis.moveToPoint(-1.4, -9.1, 975, {.forwards = true, .maxSpeed = 95});

 // 0.46, -27.1

 chassis.turnToPoint(0.46, -27.1, 600);
 chassis.moveToPoint(0.46, -27.1, 900);
 chassis.turnToPoint(18, -43.7, 700);
 chassis.moveToPoint(18, -43.7, 900, {}, false);


 // pros::delay(32942394943294242342342423);

 chassis.turnToPoint(33.8, -23.1, 600);
 chassis.moveToPoint(33.8, -23.1, 950, {}, false);
 
 // -23.9, -40.8
 chassis.turnToPoint(51.3, -17.6, 700);

 // pros::delay(329494942423434234234224);

 chassis.moveToPoint(51.3, -17.6, 700, {.maxSpeed = 60, .minSpeed = 22}, false);

 pros::delay(300);

 move(60, 0);

 pros::delay(600);

 move(0,0);

 // pros::delay(0123021302021032132132132321323);
 chassis.moveToPoint(46.6, -18.7, 500, {.forwards = false, .maxSpeed = 30, .minSpeed = 10}, false);
 move(-40, 0);
 pros::delay(300);
 move(0, 0);
 raiseasdasd.set_value(true);
 intake_class.set_velocity(127);

 chassis.waitUntilDone();

 pros::delay(200);

 move(50, 0);

 pros::delay(250);

 raiseasdasd.set_value(false);

 pros::delay(250);

 move(-40, 0);

 pros::delay(500);

 move(0,0);

 doinker.set_value(true);

 move(40,0);

 pros::delay(200);

 move(0,0);

 pros::delay(400);
 
 // pros::delay(9942934923934924394432424);

 chassis.turnToHeading(-108, 1200, {}, false);

 doinker.set_value(false);

 clamp.set_value(false);

 chassis.moveToPoint(-4.7, -36, 1200, {.minSpeed = 20});

 ladybrown_class.set_angle(170);

 pros::delay(29234923432234324234);

 chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
 
 doinker2.set_value(false);
 pros::delay(300);

 intake_class.set_velocity(127);

 
 chassis.turnToPoint(2, -34, 500);
 chassis.moveToPoint(2, -34, 800, {}, false);
 // chassis.turnToHeading(-220, 500, {}, false);
 // doinker.set_value(false);
 // pros::delay(400);
 // // 9, -40.3
 // chassis.turnToPoint(7.5, -44, 500, {.minSpeed = 50});
 // chassis.moveToPoint(7.5, -44, 700, {}, false);
 // 26.2, -49.7

 pros::delay(324249324324324234);

 chassis.turnToPoint(18, -43.7, 700);
 chassis.moveToPoint(18, -43.7, 900, {}, false);


 // pros::delay(32942394943294242342342423);

 chassis.turnToPoint(33.8, -23.1, 600);
 chassis.moveToPoint(33.8, -23.1, 950, {}, false);
 
 // -23.9, -40.8
 chassis.turnToPoint(51.3, -17.6, 700);

 // pros::delay(329494942423434234234224);

 chassis.moveToPoint(51.3, -17.6, 700, {.maxSpeed = 60, .minSpeed = 22}, false);

 pros::delay(300);

 move(60, 0);

 pros::delay(600);

 move(0,0);

 // pros::delay(0123021302021032132132132321323);
 chassis.moveToPoint(46.6, -18.7, 500, {.forwards = false, .maxSpeed = 30, .minSpeed = 10}, false);

 raiseasdasd.set_value(true);
 intake_class.set_velocity(127);

 chassis.waitUntilDone();

 pros::delay(200);

 move(50, 0);

 pros::delay(250);

 raiseasdasd.set_value(false);

 pros::delay(250);

 move(-40, 0);

 pros::delay(500);

 move(0,0);

 doinker.set_value(true);

 move(40,0);

 pros::delay(200);

 move(0,0);

 pros::delay(400);
 
 // pros::delay(9942934923934924394432424);

 chassis.turnToHeading(-108, 1200, {}, false);

 doinker.set_value(false);

 clamp.set_value(false);

 chassis.moveToPoint(-4.7, -36, 1200, {.minSpeed = 20});

 ladybrown_class.set_angle(170);

 pros::delay(9942934923934924394432424);

}

void goGoGadgetRedRingSide(){

 //5 + 1 
 badcolor = BLUE;
 raiseasdasd.set_value(false);
 ladybrown_class.set_angle(155);
 autoclamp_bool = false;
 clamp.set_value(false);
 
 chassis.moveToPoint(0, 7, 500);
 pros::delay(500);
 
 chassis.moveToPoint(0, -25, 500, {.forwards=false, .maxSpeed = 80, .earlyExitRange = 12}); 
 chassis.moveToPoint(0, -27, 800, {.forwards=false, .maxSpeed = 50, .minSpeed = 10}); 
 
 pros::delay(550);
 clamp.set_value(true);
 ladybrown_class.set_angle(-40);

 chassis.turnToPoint(4.75, -56.4, 1000, {.maxSpeed = 60});

 // pros::delay(2949423424392432424232323);

 chassis.moveToPoint(4.75, -56.4, 1100, {.maxSpeed = 70});

 doinker2.set_value(true);

 intake_class.set_velocity(127);

 chassis.moveToPoint(-1.72, -23.2, 1000, {.forwards = false, .maxSpeed = 110}, false);

 doinker2.set_value(false);

 //4.5, -32.5

 chassis.turnToPoint(7, -38, 360);
 chassis.moveToPoint(7, -38, 700);
 //16.5, -43.8
 chassis.turnToPoint(16.5, -43.8, 600);
 chassis.moveToPoint(16.5, -43.8, 700);


 chassis.turnToPoint(33.8, -23.1, 600);
 chassis.moveToPoint(33.8, -23.1, 950, {}, false);
 
 // -23.9, -40.8
 chassis.turnToPoint(51.3, -17.6, 550);

 // pros::delay(329494942423434234234224);

 chassis.moveToPoint(51.3, -17.6, 700, {.maxSpeed = 60, .minSpeed = 22}, false);

 pros::delay(300);

 move(60, 0);

 pros::delay(600);

 move(0,0);

 // pros::delay(0123021302021032132132132321323);
 chassis.moveToPoint(46.6, -18.7, 500, {.forwards = false, .maxSpeed = 30, .minSpeed = 10}, false);

 raiseasdasd.set_value(true);
 intake_class.set_velocity(127);

 chassis.waitUntilDone();

 pros::delay(200);

 move(50, 0);

 pros::delay(250);

 raiseasdasd.set_value(false);

 pros::delay(250);

 move(-40, 0);

 pros::delay(500);

 move(0,0);


 // -2.7, -2.7

 chassis.turnToPoint(-2.7, -2.77, 1000);
 chassis.moveToPoint(-2.7, -2.77, 600, {.earlyExitRange = 12}, false);
 raiseasdasd.set_value(true);
 intake_class.set_velocity(127);
 chassis.moveToPoint(-2.7, -2.77, 1000, {.maxSpeed = 60}, false);
 raiseasdasd.set_value(false);
 move(-50, 0);
 pros::delay(300);
 move(0, 0);

 chassis.turnToPoint(-10.5, -15.1, 600);
 chassis.moveToPoint(-10.5, -15.1, 700, {.earlyExitRange = 12});

 ladybrown_class.set_angle(170);

 pros::delay(9942934923934924394432424);
}

void bakerBlueLMirror(){
 badcolor = BLUE;
 raiseasdasd.set_value(false);
 ladybrown_class.set_angle(155);
 autoclamp_bool = false;
 clamp.set_value(false);
 
 chassis.moveToPoint(0, 7, 500);
 pros::delay(500);
 
 chassis.moveToPoint(0, -25, 500, {.forwards=false, .maxSpeed = 80, .earlyExitRange = 12}); 
 chassis.moveToPoint(0, -27, 800, {.forwards=false, .maxSpeed = 50, .minSpeed = 10}); 
 
 pros::delay(550);
 clamp.set_value(true);
 ladybrown_class.set_angle(-40);

 

 // -19, -39
 //-106
 //chassis.turnToPoint(-20.4, -37.2, 800);

 // pros::delay(3949249234992434342343434234);

 chassis.turnToPoint(18.9, -36.7, 700);
 //chassis.turnToPoint(-20.5, -36.3, 700);
 pros::delay(500);
 wallrot.set_position(0);
 ladybrown_class.set_angle(0);
 chassis.waitUntilDone();
 chassis.angularPID.reset();
 
 chassis.moveToPoint(18.9, -36.7, 750, {.maxSpeed = 80}, false);
 // -1, 11
 doinker.set_value(true);
 pros::delay(300);
 // pros::delay(342493294234923432249242394324234424234);
 // chassis.turnToHeading(-128, 500);
 // chassis.moveToPoint(-22, -39.4, 400, {.maxSpeed = 127, .minSpeed = 50}, false);
 // doinker.set_value(true);
 
 
 // pros::delay(300);
 // chassis.turnToPoint(13.9, -19.9, 200, {false});
 // chassis.waitUntilDone();

 chassis.moveToPoint(-13.9, -19.9, 975, {.forwards = false, .maxSpeed = 95}, false);

 chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
 
 doinker.set_value(false);
 pros::delay(300);

 intake_class.set_velocity(127);

 
 chassis.turnToPoint(-2, -34, 500);
 chassis.moveToPoint(-2, -34, 800, {}, false);
 // chassis.turnToHeading(-220, 500, {}, false);
 // doinker.set_value(false);
 // pros::delay(400);
 // // 9, -40.3
 // chassis.turnToPoint(7.5, -44, 500, {.minSpeed = 50});
 // chassis.moveToPoint(7.5, -44, 700, {}, false);
 // 26.2, -49.7

 // pros::delay(324249324324324234);

 chassis.turnToPoint(-18, -43.7, 700);
 chassis.moveToPoint(-18, -43.7, 900, {}, false);


 // pros::delay(32942394943294242342342423);

 chassis.turnToPoint(-33.8, -23.1, 600);
 chassis.moveToPoint(-33.8, -23.1, 950, {}, false);
 
 // -23.9, -40.8
 chassis.turnToPoint(-51.3, -17.6, 700);

 // pros::delay(329494942423434234234224);

 chassis.moveToPoint(-51.3, -17.6, 700, {.maxSpeed = 60, .minSpeed = 22}, false);

 pros::delay(300);

 move(60, 0);

 pros::delay(600);

 move(0,0);

 // pros::delay(0123021302021032132132132321323);
 chassis.moveToPoint(-46.6, -18.7, 500, {.forwards = false, .maxSpeed = 30, .minSpeed = 10}, false);

 raiseasdasd.set_value(true);
 intake_class.set_velocity(127);

 chassis.waitUntilDone();

 pros::delay(200);

 move(50, 0);

 pros::delay(250);

 raiseasdasd.set_value(false);

 pros::delay(250);

 move(-40, 0);

 pros::delay(500);

 move(0,0);

 doinker.set_value(true);

 move(40,0);

 pros::delay(200);

 move(0,0);

 pros::delay(400);
 
 // pros::delay(9942934923934924394432424);

 chassis.turnToHeading(108, 1200, {}, false);

 doinker.set_value(false);

 clamp.set_value(false);

 chassis.moveToPoint(4.7, -36, 1200, {.minSpeed = 20});

 ladybrown_class.set_angle(170);

 pros::delay(9942934923934924394432424);

}

void bakerBlueM(){
 badcolor = RED;
 raiseasdasd.set_value(false);
 ladybrown_class.set_angle(155);
 autoclamp_bool = false;
 clamp.set_value(false);
 
 chassis.moveToPoint(0, 6, 500);
 pros::delay(500);
 
 chassis.moveToPoint(0, -25, 500, {.forwards=false, .maxSpeed = 80, .earlyExitRange = 12}); 
 chassis.moveToPoint(0, -27, 800, {.forwards=false, .maxSpeed = 50, .minSpeed = 10}); 
 
 pros::delay(550);
 clamp.set_value(true);
 ladybrown_class.set_angle(-40);

 

 // -19, -39
 //-106
 //chassis.turnToPoint(-20.4, -37.2, 800);

 // pros::delay(3949249234992434342343434234);

 chassis.turnToPoint(-18.9, -36.7, 700);
 //chassis.turnToPoint(-20.5, -36.3, 700);
 pros::delay(500);
 wallrot.set_position(0);
 ladybrown_class.set_angle(0);
 chassis.waitUntilDone();
 chassis.angularPID.reset();
 
 chassis.moveToPoint(-18.9, -36.7, 750, {.maxSpeed = 80}, false);
 // -1, 11
 doinker2.set_value(true);
 pros::delay(300);
 // pros::delay(342493294234923432249242394324234424234);
 // chassis.turnToHeading(-128, 500);
 // chassis.moveToPoint(-22, -39.4, 400, {.maxSpeed = 127, .minSpeed = 50}, false);
 // doinker.set_value(true);
 
 
 // pros::delay(300);
 // chassis.turnToPoint(13.9, -19.9, 200, {false});
 // chassis.waitUntilDone();

 chassis.moveToPoint(13.9, -19.9, 975, {.forwards = false, .maxSpeed = 95}, false);

 chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
 
 doinker2.set_value(false);
 pros::delay(300);

 intake_class.set_velocity(127);

 
 chassis.turnToPoint(2, -34, 500);
 chassis.moveToPoint(2, -34, 800, {}, false);
 // chassis.turnToHeading(-220, 500, {}, false);
 // doinker.set_value(false);
 // pros::delay(400);
 // // 9, -40.3
 // chassis.turnToPoint(7.5, -44, 500, {.minSpeed = 50});
 // chassis.moveToPoint(7.5, -44, 700, {}, false);
 // 26.2, -49.7

 // pros::delay(324249324324324234);

 chassis.turnToPoint(18, -43.7, 700);
 chassis.moveToPoint(18, -43.7, 900, {}, false);


 // pros::delay(32942394943294242342342423);

 chassis.turnToPoint(33.8, -23.1, 600);
 chassis.moveToPoint(33.8, -23.1, 950, {}, false);
 
 // -23.9, -40.8
 chassis.turnToPoint(51.3, -17.6, 700);

 // pros::delay(329494942423434234234224);

 chassis.moveToPoint(51.3, -17.6, 700, {.maxSpeed = 60, .minSpeed = 22}, false);

 pros::delay(300);

 move(60, 0);

 pros::delay(500);

 move(0,0);

 // pros::delay(0123021302021032132132132321323);
 chassis.moveToPoint(46.6, -18.7, 500, {.forwards = false, .maxSpeed = 30, .minSpeed = 10}, false);

 raiseasdasd.set_value(true);
 intake_class.set_velocity(127);

 chassis.waitUntilDone();

 pros::delay(200);

 move(50, 0);

 pros::delay(250);

 raiseasdasd.set_value(false);

 intake_class.set_velocity(0);

 pros::delay(250);

 move(-40, 0);

 pros::delay(200);

 intake_class.set_velocity(127);

 pros::delay(300);

 move(0,0);

 doinker2.set_value(true);

 move(40,0);

 pros::delay(200);

 move(0,0);

 pros::delay(300);
 
 // pros::delay(9942934923934924394432424);

 chassis.turnToHeading(-108, 800, {}, false);

 doinker2.set_value(false);

 clamp.set_value(false);

 chassis.moveToPoint(10.9, -24.7, 800, {.minSpeed = 20}); 

 chassis.turnToHeading(5.8, 1200, {}, false);

 pros::delay(3949234992349429423434244);

 chassis.moveToPoint(-4.7, -36, 1200, {.minSpeed = 20});

 ladybrown_class.set_angle(170);

 pros::delay(9942934923934924394432424);

}

void gurt_yo_yo_gurt(){
 ladybrown_slow = true;
 ladybrown_class.set_angle(155);
 ladybrown_slow = false;
 autoclamp_bool = false;
 clamp.set_value(false);
 pros::delay(500);
 chassis.moveToPoint(0, -7, 500);
 chassis.turnToHeading(-52,600);
 chassis.moveToPoint(18.6, -25.9, 500, {.forwards=false, .earlyExitRange = 12}); 
 chassis.moveToPoint(25, -31.8, 400, {.forwards=false, .maxSpeed = 50}); 
 pros::delay(500);
 clamp.set_value(true);
 ladybrown_class.set_angle(-40);
 pros::delay(200);
 intake_class.set_velocity(127);
 pros::delay(500);
 wallrot.set_position(0);
 ladybrown_class.set_angle(0);
 //19.9, -51.8
 // -3.3, -41.9
 // -20.2, -46
 // back -2.3, -43.5

 chassis.turnToPoint(18, -2, 1000, {true}, false);
 raiseasdasd.set_value(true);
 chassis.moveToPoint(18, -2.5, 300, {.forwards=true, .maxSpeed = 127}, false);
 chassis.moveToPoint(18, -2.5, 800, {.forwards=true, .maxSpeed = 60}, false);
 raiseasdasd.set_value(false);
 pros::delay(600);
 chassis.moveToPoint(18, -8, 1000, {.forwards=false, .maxSpeed = 60}, false);

 // drop off mogo

 // 50.9, 5.9 clamp
 // 67, 29.1

 // 69.2, 4.8 lb down for ladder

}

void mogoRush(){
 chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
 doinker.set_value(true);
 intake_class.set_velocity(127);
 chassis.turnToPoint(-2, 32.5, 300);
 chassis.moveToPoint(-2, 32.5, 800, {.forwards=true});
 pros::delay(700);
 doinker.set_value(false);
 pros::delay(200);
 intake_class.set_velocity(0);

 chassis.turnToPoint(-8.4, 23.9, 100, {false});
 chassis.moveToPoint(-8.4, 23.9, 900, {.forwards=false}, false);
 doinker.set_value(true);
 pros::delay(300);
 chassis.moveToPoint(-8.4, 19, 1000, {.forwards=false}, false);
 doinker.set_value(false);

 // 3.8, 30.7
 chassis.turnToPoint(3.3, 31.6, 1000, {false});
 chassis.moveToPoint(3.3, 31.6, 900, {.forwards=false, .maxSpeed = 60}, false);
 clamp.set_value(true);
 intake_class.set_velocity(127);

 // 4.2, -7.1
 chassis.turnToPoint(4.2, -7.1, 1000);
 chassis.moveToPoint(4.2, -7.1, 2000);
 pros::delay(650);
 clamp.set_value(false);
 pros::delay(2200);
 move(127, 0);
 pros::delay(600);
 move(0,0);
 intake_class.set_velocity(0);



 // chassis.turnToPoint(3, 23.5, 200);

 // chassis.moveToPoint(3, 45, 1000, {.forwards=false});
 //0.47, 23.5
 //3,45

 //-2.4,38
 //-2.8,27.2

 
 // // // chassis.turnToPoint(-1.3, 33.5, 200);
 // // // chassis.moveToPoint(-1.3, 33.5, 3000);
 // // // pros::delay(700);
 // // // doinker.set_value(false);
 // // // pros::delay(200);

 // // // chassis.turnToPoint(-2.3, 20.7, 200);
 // // // chassis.moveToPoint(-2.3, 20.7, 2000, {.forwards = false});
 // // // pros::delay(100);
 // // // doinker.set_value(true);
 // // // pros::delay(100);

 // // // chassis.turnToPoint(7.2, 34.1, 200);
 // // // chassis.moveToPoint(7.2, 34.1, 2000, {.forwards=false});

 // hi

 // chassis.turnToPoint(7.6, 30.3, 200);
 // chassis.moveToPoint(7.6, 30.3, 2000, {.forwards=false, .maxSpeed = 60});

 // chassis.turnToPoint(-0.9, 36.3, 200);
 // chassis.moveToPoint(-0.9, 36.3, 3000);
 // doinker.set_value(true);
 // ladybrown_class.set_angle(50);
 // pros::delay(200);
 // intake_class.set_velocity(127);
 // pros::delay(600);
 // intake_class.set_velocity(0);
 // pros::delay(400);
 // doinker.set_value(false);
 
 // //-3.4, 22.1
 // chassis.turnToPoint(-3.4, 22.1, 1000, {false});
 // chassis.moveToPoint(-3.4, 22.1, 1000, {false}, false);
 // doinker.set_value(true);
 // // 5.6, 33.6
 // chassis.turnToPoint(5.6, 33.6, 1000, {false});
 // chassis.moveToPoint(5.6, 33.6, 1000, {false}, false);
 // clamp.set_value(true);
 // doinker.set_value(false);
 // intake_class.set_velocity(127);
 // // 6.2, 11.6
 // chassis.turnToPoint(6.2, 11.6, 1000, {true});
 // chassis.moveToPoint(6.2, 11.6, 1000, {true}, false);
 // clamp.set_value(false);
 // // 5, -3.2
 // chassis.turnToPoint(5, -3.2, 1000, {true});
 // chassis.moveToPoint(5, -3.2, 1000, {true}, false);
 // move(127, 0);
 // pros::delay(700);
 // // 0.8, -5.5
 // chassis.turnToPoint(0.8, -5.5, 300, {false});
 
 // chassis.moveToPoint(0.8, -5.5, 500, {false});
 // pros::delay(200);
 // intake_class.set_velocity(0);
 
 // // -15.3, 1.1
 // chassis.turnToPoint(-15.3, 1.1, 1000, {false});
 // chassis.moveToPoint(-15.3, 1.1, 1000, {false}, false);
 // // -22.3, -33.2
 // pros::delay(1010101010);
 // chassis.turnToPoint(-22.3, 33.2, 1000, {false});
 // chassis.moveToPoint(-22.3, 33.2, 1000, {false}, false);
 // clamp.set_value(true);

}

void WorldsAWP() {

 badcolor = RED;
 raiseasdasd.set_value(false);
 ladybrown_class.set_angle(150);
 autoclamp_bool = false;
 clamp.set_value(false);
 
 chassis.moveToPoint(0.3, 7, 500);
 pros::delay(500);
 
 chassis.moveToPoint(0, -25, 400, {.forwards=false, .maxSpeed = 80}); 
 chassis.moveToPoint(0, -27, 800, {.forwards=false, .maxSpeed = 50, .minSpeed = 10}); 
 
 pros::delay(550);
 clamp.set_value(true);

 ladybrown_class.set_angle(-40);

 intake_class.set_velocity(127);

 chassis.turnToPoint(17.8, -44.9, 800);

 chassis.moveToPoint(17.8, -44.9, 800, {.forwards=true, .maxSpeed = 80}); 

 chassis.turnToPoint(32.8, -23.8, 800);

 chassis.moveToPoint(32.8, -23.8, 800, {.forwards=true, .maxSpeed = 80}); 

 chassis.turnToPoint(45.5, -20.2, 800, {}, false);

 // chassis.moveToPoint(45.5, -20.2, 200, {.forwards=true, .maxSpeed = 80}, false); 

 move(115, 0);

 pros::delay(300);

 move(40, 0);

 pros::delay(600);

 move(0, 0);

 chassis.moveToPoint(33.5, -24.2, 700, {.forwards = false, .maxSpeed = 90});

 chassis.turnToPoint(-9.1, -3.2, 800, {.forwards = true});

 // pros::delay(1000302403242343244234234);


 chassis.moveToPoint(-9.1, -3.2, 450, {.maxSpeed = 100}, false);

 raiseasdasd.set_value(true);

 // pros::delay(932943249294323423);

 chassis.moveToPoint(-9.1, -3.2, 900, {.maxSpeed = 40}, false);

 // chassis.moveToPoint(-2.8, -2.3, 800, {.forwards = false}, false);

 raiseasdasd.set_value(false);

 pros::delay(300);

 move(-40, 0);

 pros::delay(500);

 move(0,0);

 pros::delay(500);

 clamp.set_value(false);

 pros::delay(200);

 chassis.moveToPoint(-0.8, -3.5, 800, {}, false);

 move(40, 0);

 pros::delay(200);

 move(0,0);

 doinker2.set_value(true);

 chassis.turnToPoint(-39.2, -7.3, 1000, {.forwards = false});

 chassis.moveToPoint(-39.2, -7.3, 450, {.forwards=false, .maxSpeed = 80}); 

 chassis.moveToPoint(-43.3, -8.1, 800, {.forwards=false, .maxSpeed = 50, .minSpeed = 10}); 

 doinker2.set_value(false);

 pros::delay(500);

 clamp.set_value(true);

 pros::delay(400);

 // pros::delay(9000304230432423423424234);

 chassis.turnToPoint(-64.6, 0.3, 800);

 chassis.moveToPoint(-64.6, 0.3, 900, {}, false);

 pros::delay(400);

 chassis.turnToPoint(-53, -13, 800);

 chassis.moveToPoint(-53, -13, 900);

 pros::delay(500);

 ladybrown_class.set_angle(160);

}

void worldsRingRush() {

    badcolor = BLUE;

    ladybrown_class.set_angle(65);

    chassis.moveToPoint(0, 54, 1200, {.maxSpeed = 85, .minSpeed = 10, .earlyExitRange = 1});
    pros::delay(450);
    doinker2.set_value(true);
    intake_class.set_velocity(127);
    chassis.waitUntilDone();
    intake_class.set_velocity(0);

    chassis.turnToPoint(-1.3, 20.3, 800, {.forwards = false});
    chassis.moveToPoint(-1.3, 20.3, 1400, {.forwards = false, .maxSpeed = 70});

    chassis.waitUntilDone();

    doinker2.set_value(false);

    pros::delay(400);

    chassis.turnToPoint(9.7, 27.2, 900, {.forwards = false});
    
    chassis.moveToPoint(9.7, 27.2, 1200, {.forwards = false, .maxSpeed = 60});

    

    

    chassis.waitUntilDone();
    clamp.set_value(true);

    // pros::delay(9924949442424424424424);

    pros::delay(200);

    // chassis.turnToPoint(-0.8, 37.5, 900, {.forwards = true});

    // chassis.moveToPoint(-0.8, 37.5, 1000);

    intake_class.set_velocity(127);

    chassis.turnToPoint(-2, 33.4, 900, {.forwards = true});

    chassis.moveToPoint(-2, 33.4, 1000);

    chassis.turnToPoint(-15.9, 30.7, 900, {.forwards = true});

    chassis.moveToPoint(-15.9, 30.7, 1000);

    chassis.turnToPoint(-19.3, 11.6, 900, {.forwards = true});

    chassis.moveToPoint(-19.3, 11.6, 1000, {}, false);

    chassis.turnToPoint(-35, 9.6, 900, {.forwards = true});

    chassis.moveToPoint(-35, 9.6, 500, {}, false);

    move(80, 0);
    
    pros::delay(500);

    move(0, 0);
    
    chassis.moveToPoint(-23.9, 18.3, 800, {.forwards = false, .maxSpeed = 100});

    chassis.turnToPoint(22.6, 1.66, 900, {.forwards = true});

    // pros::delay(92492924949249249422424);

    chassis.moveToPoint(22.6, 1.66, 600, {.maxSpeed = 110}, true);

    chassis.moveToPoint(22.6, 1.66, 500, {.maxSpeed = 50}, true);

    raiseasdasd.set_value(true);

    chassis.waitUntilDone();

    raiseasdasd.set_value(false);

    pros::delay(300);

    chassis.moveToPoint(18, 3.7, 800, {.forwards = false, .maxSpeed = 70});

    chassis.turnToPoint(18.1, -15.1, 800, {.forwards = true, .maxSpeed = 70});

    // pros::delay(949249249224244424);

    // ladybrown_class.set_angle(100);

    chassis.moveToPoint(18.1, -15.1, 800, {.forwards = true, .maxSpeed = 70});

    chassis.setPose(0,0,0);

    chassis.moveToPoint(0, -5, 600);

    chassis.waitUntilDone();

    ladybrown_class.set_angle(160);

    // ladybrown_class.set_angle(140);

    pros::delay(200);

    // ladybrown_class.set_angle(160);
    

}
 
void autonomous() {
 pros::delay(20);
 clamp.set_value(false);
 
 // autonRunning = BLUE_RING;

 doinker.set_value(false);
 //clamp.set_value

 left_front_motor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
 left_back_motor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
 left_center_motor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
 right_front_motor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
 right_back_motor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
 right_center_motor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

 hangs.set_value(false);
 intake_class.maxTorque=0.35;

 //!!!one of these!!!

 // bakerBlueLMirror();
 // bakerBlueL();
 doublebakerBlueL();
 //bakerRed();
 //fivePlusOneBlue();
 //fivePlusOneRed();
}
void opcontrol() {

 color_sort.set_led_pwm(100);

 intake_class.maxTorque = 0.5; 
 pros::delay(50);
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
 int timer = 0;
 int pos = 0;
 float pwall, iwall, dwall;
 bool doinker2toggle = false;
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
 bool speedbool = false;
 bool raised = false;
 int colorsortdrivercounter = 0;
 int rumblecounter = 0;
 ladybrown_manual.store(false);
 
//  bakerBlueL();
//  WorldsAWP();
//worldsRingRush();
// negaYogineni();
    doublebakerBlueL();

 // goGoGadgetRedRingSide();

 // wallrot.set_position(1);

 wallmotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
 bool manual = false;
 
 //chassis.turnToHeading(getAngleToPoint(10, 0), 1000);
 //chassis.moveToPoint(10, 0, 1000);

 badcolor = RED;

 //soloWP();

 intake_class.maxTorque = 0.3;


 pros::delay(394923493423949234324343224234234);

 chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
 
 while (true) {
 int power = master.get_analog(ANALOG_LEFT_Y);
 int turn = master.get_analog(ANALOG_RIGHT_X);
 bool manualswitch = false;
 wallmotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

 if(manualswitch){
 manual = !manual;
 }

 bool outtakebutton = master.get_digital_new_press(DIGITAL_X);
 bool intakebutton = master.get_digital_new_press(DIGITAL_Y);
 bool wallcycler = master.get_digital_new_press(DIGITAL_L1);
 bool wallreturn = master.get_digital_new_press(DIGITAL_L2);
 bool wallMid = master.get_digital_new_press(DIGITAL_B);
 bool wallDown = master.get_digital_new_press(DIGITAL_DOWN);
 bool returnLB = master.get_digital_new_press(DIGITAL_RIGHT);
 bool doinkertwo = master.get_digital_new_press(DIGITAL_LEFT);
 bool rightBackHold = master.get_digital(DIGITAL_R2); // bool rightfront = master.get_digital_new_press(DIGITAL_R1);
 bool upbutton = master.get_digital_new_press(DIGITAL_UP);
 bool RB = master.get_digital_new_press(DIGITAL_R2);

 if (RB){
 raised = !raised;
 raiseasdasd.set_value(raised);
 }

 if(doinkertwo){
 doinker2toggle = !doinker2toggle;

 doinker2.set_value(doinker2toggle);
 }

 // if (rightBackHold){
 // colorsortdrivercounter++;
 // } else{
 // colorsortdrivercounter = 0;
 // }

 // if (upbutton){
 // badcolor = bad_color ? badcolor != DRIVER : DRIVER;
 // }

 // if (colorsortdrivercounter > 50){
 // badcolor = bad_color ? badcolor != DRIVER : DRIVER;
 // colorsortdrivercounter = 0;
 // master.rumble("...");
 // rumblecounter = 1;
 // // master.set_text(3, 0, "..");
 // }

 // if (rumblecounter > 0){
 // rumblecounter++;
 // }
 // else if (rumblecounter > 20){
 // rumblecounter = 0;
 // }

 // if (rightBack){
 // raised = !raised;
 // raiseasdasd.set_value(raised);
 // }

 // if (upbutton){
 // speedbool = !speedbool;
 // }

 bool leftClamp = false;

 if (leftClamp){
 ladybrown_manual.store(false);
 ladybrown_class.set_angle(190);
 pros::delay(500);
 move(-80, 0);
 // returnLB = true;
 ladybrown_class.set_angle(0);
 autoclamp_bool = true;
 pros::delay(700);
 autoclamp_bool = false;
 move(0,0);
 // chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
 // pros::delay(200);
 // chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
 returnLB = true;
 // chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
 // clamp.set_value(true);
 // intake_class.set_velocity(127);
 // pros::delay(700);
 // intake_class.set_velocity(0);
 // pros::delay(200);
 // chassis.moveToPoint(0, 12, 500, {.forwards=true, .maxSpeed = 80});
 // chassis.turnToHeading(-89, 550, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 55});
 // chassis.moveToPoint(21, 13, 700, {.forwards=false, .maxSpeed = 80});
 // pros::delay(600);
 // clamp.set_value(false);
 // ladybrown_manual = false;
 }
 if(returnLB){
 ladybrown_manual.store(true);
 wallmotor.move(-50);
 pros::delay(150);
 wallmotor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
 wallmotor.brake();
 pros::delay(300);
 wallmotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
 wallrot.reset_position();
 
 ladybrown_manual.store(false);
 }
 
 
 pos = wallrot.get_position()/100.0f;
 err = targetpos - pos;
 float derr = err-preverr;
 sumerrorwall += err;
 float ret = pwall * err + iwall * sumerrorwall + dwall * derr;
 if(manual){goto skp;}
 // wallmotor.move((ret * 127) > 127 ? 127 : (ret*127));
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
 intake_class.set_velocity(-127);
 pros::delay(50);
 intake_class.set_velocity(0);
 }
 }
 
 if(wallDown) {
 intake.move(0);
 intaking = false;
 outtaking = false;
 //moveDown = false;
 stage = (stage != 3) ? 3 : 0;
 sumerrorwall = 0.0f;
 }
 if(wallMid) {
 moveDown = false;
 if(stage != 4){
 intake_class.releasedMogo += 40;
 }
 stage = (stage != 4) ? 4 : 0;
 sumerrorwall = 0.0f;
 intaking = false;
 outtaking = false;
 intake_class.set_velocity(-127);
 pros::delay(50);
 intake_class.set_velocity(0);
 }
 if(wallcycler || wallreturn){
 moveDown = false;
 sumerrorwall = 0.0f;
 }
 if(wallreturn){
 moveDown = false;
 sumerrorwall = 0.0f;
 stage = 0;
 }
 // if(fabs(wallrot.get_angle() % 360) < 3.0f){
 // wallrot.set_position(0);
 // }

 

 targetpos = (stage == 1) * targetWait + (stage == 2) * targetTop + (stage == 3) * targetDown + (stage == 4) * targetHold;
 
 ladybrown_class.set_angle(targetpos);
 goto skp2;
 
 skp:;

 skp2:;

 bool clampbutton = master.get_digital_new_press(DIGITAL_R1);

 doinker_button = master.get_digital_new_press(DIGITAL_A);

 if(clampbutton){
 clamped2 = !clamped2;
 clamp.set_value(clamped2);
 if(clamped2){
 intake_class.releasedMogo+=40;
 }
 }

 // if(master.get_digital_new_press(DIGITAL_UP)){
 // }

 if(doinker_button){
 doinker_state = !doinker_state;
 doinker.set_value(doinker_state);
 } 
 
 // if (upbutton){
 // badcolor = bad_color ? badcolor != DRIVER : DRIVER;
 // }

 // if (!clamped2){
 // master.set_text(3, 0, ".");
 // }
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

 if (stage == 1){
 intake_class.maxTorque = 0.5;
 }
 else {
 intake_class.maxTorque = 0.5;
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
