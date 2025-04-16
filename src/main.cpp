#include "main.h"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include <atomic>
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
pros::Motor left_center_motor(12, pros::E_MOTOR_GEAR_BLUE	, false);
pros::Motor left_back_motor(13, pros::E_MOTOR_GEAR_BLUE	, false);    
pros::Motor right_front_motor(1, pros::E_MOTOR_GEAR_BLUE	, true);
pros::Motor right_center_motor(2, pros::E_MOTOR_GEAR_BLUE	, true);
pros::Motor right_back_motor(11, pros::E_MOTOR_GEAR_BLUE	, true);

pros::Motor intake(3, pros::E_MOTOR_GEAR_BLUE, false);

pros::Imu inertial_sensor(7);
pros::Optical colorSensor(4);
pros::Distance intakeDistSensor(16);


/*
SUBSYSTEMS
*/

pros::Motor intake2(0);

Intake intake_class;

pros::Motor wallmotor(14, pros::E_MOTOR_GEAR_RED	, true);

Ladybrown ladybrown_class;

/*
PNEUMATICS
*/

/*
PNEUMATICS
*/

pros::ADIDigitalOut clamp('H'); //backwings H, F
pros::ADIDigitalOut hangs('Z');
pros::ADIDigitalOut doinker('G');
pros::ADIDigitalOut raiseasdasd('F');
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
        12, // track width
        lemlib::Omniwheel::NEW_325_HALF, // wheel diameter
        600, // wheel rpm
        0
};

/*
SENSORS
*/

pros::Rotation wallrot(5, false); 

pros::Rotation horizontal_rot(15); // port 1, not reversed
pros::Rotation vertical_rot(9); // port 1, not reversed

lemlib::TrackingWheel horizontal_track(&horizontal_rot, lemlib::Omniwheel::NEW_275 , -3.6f); // 0.6 -0.9
lemlib::TrackingWheel vertical_track(&vertical_rot, lemlib::Omniwheel::NEW_2 , 0.0f); // 0.6 -0.

pros::Distance distance_sensor(16);
pros::Optical color_sort(18);

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

lemlib::ControllerSettings lateralController(9, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              55, // derivative gain (kD)
                                              10, // anti windup
                                              0.2, // small error range, in inches
                                              1000, // small error range timeout, in milliseconds
                                              2, // large error range, in inches
                                              3000, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);




lemlib::ControllerSettings angularController(4, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              45, // derivative gain (kD)
                                              5, // anti windup
                                              0.5, // small error range, in inches
                                              1000, // small error range timeout, in milliseconds
                                              2, // large error range, in inches
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

/*
INTAKE THREAD
*/
#define RED 17
#define BLUE 212
#define DRIVER -6


#define bad_color BLUE;

int badcolor = bad_color;

#define inColor(c) (c <= badcolor+5 && c >= badcolor-5)

void intake_thread(){
    while(true){
    // printf("%f \n",intake.get_torque());
        intake_class.update(intake.get_torque());
        printf("Intake velocity: %f \n", intake_class.get_velocity());
        if(inColor(color_sort.get_hue())){
            intake.move(127);
            pros::delay(50);
            intake.move(0);
            pros::delay(250);
            continue;
        }
        intake.move(intake_class.get_velocity());
        pros::delay(10);
    }
}

void redColorSort() {
    colorSensor.set_integration_time(10);
    counter++;
    if(colorSensor.get_hue() >= 17 && counter >= 5)
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
    colorSensor.set_integration_time(10);
    counter++;
    if(colorSensor.get_hue() <= 200 && counter >= 5)
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

#define targetWait 23
#define targetTop 169
#define targetDown 225
#define targetHold 54
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
            wallmotor.move(ladybrown_class.get_velocity(false) * (ladybrown_slow ? 0.5f: 1.0f));
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
    color_sort.set_led_pwm(100);
    color_sort.set_integration_time(3);
    chassis.calibrate(); // calibrate the chassis
    chassis.setPose(0, 0, 0); // X: 0, Y: 0, Heading: 0
    hangs.set_value(false);
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


void autonomous() {

    pros::delay(20);
    clamp.set_value(true);
    // autonRunning = BLUE_RING;

    doinker.set_value(false);
    //clamp.set_value

    left_front_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    left_back_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    left_center_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    right_front_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    right_back_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    right_center_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    // left_front_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    // left_back_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    // left_center_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    // right_front_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    // right_back_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    // right_center_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

    hangs.set_value(false);
    intake_class.maxTorque=0.35;


    // clamp.set_value(true);
    // switch (autonRunning) {
    //     case RED_RING_RUSH:
    //         MirrorBlueRingRush();
    //     case BLUE_RING_NOT_DONE:
    //         BlueRingRush();
    //     case BLUE_MOGO_ELIM:
    //         BlueMogoELIM();
    //     case RED_MOGO:
    //         RedMogoELIM();
    //     case SKILLS_AUTO:
    //         skillsAutonStatesWallStakes();
    // }

    // pros::task_t my_task = task_create(pidUpdate, (void*)"PROS", TASK_PRIORITY_DEFAULT,
    //                            TASK_STACK_DEPTH_DEFAULT, "My Task");
    // pros::Task my_cpp_task (my_task);
}

void opcontrol() {
    pros::delay(50);
    wallrot.set_position(0);
    bool doinker_state = false;
    bool doinker_button = false;
    doinker.set_value(false);

    // left_front_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    // left_back_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    // left_center_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    // right_front_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    // right_back_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    // right_center_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

    left_front_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    left_back_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    left_center_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    right_front_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    right_back_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    right_center_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

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
    bool speedbool = false;
    bool raised = false;
    int colorsortdrivercounter = 0;
    int rumblecounter = 0;
    ladybrown_manual.store(false);

    // wallrot.set_position(1);

    clamp.set_value(true);

    wallmotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    bool manual = false;
    intake_class.maxTorque=0.5;

    badcolor = BLUE;

    soloWP();

	// while (true) {
    //     int power = master.get_analog(ANALOG_LEFT_Y) * (1-0.6*speedbool);
    //     int turn = master.get_analog(ANALOG_RIGHT_X);
    //     bool manualswitch = false;
    //     wallmotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    //     if(manualswitch){
    //         manual = !manual;
    //     }

    //     bool outtakebutton = master.get_digital_new_press(DIGITAL_X);
    //     bool intakebutton = master.get_digital_new_press(DIGITAL_Y);
    //     bool wallcycler = master.get_digital_new_press(DIGITAL_L1);
    //     bool wallreturn = master.get_digital_new_press(DIGITAL_L2);
    //     bool wallMid = master.get_digital_new_press(DIGITAL_B);
    //     bool wallDown = master.get_digital_new_press(DIGITAL_DOWN);
    //     bool returnLB = master.get_digital_new_press(DIGITAL_RIGHT);
    //     bool rightBackHold = master.get_digital(DIGITAL_R2);       // bool rightfront = master.get_digital_new_press(DIGITAL_R1);
    //     bool upbutton = master.get_digital_new_press(DIGITAL_UP);
    //     bool RB = master.get_digital_new_press(DIGITAL_R2);

    //     if (RB){
    //         raised = !raised;
    //         raiseasdasd.set_value(raised);
    //     }

    //     // if (rightBackHold){
    //     //     colorsortdrivercounter++;
    //     // } else{
    //     //     colorsortdrivercounter = 0;
    //     // }

    //     // if (upbutton){
    //     //     badcolor = bad_color ? badcolor != DRIVER : DRIVER;
    //     // }

    //     // if (colorsortdrivercounter > 50){
    //     //     badcolor = bad_color ? badcolor != DRIVER : DRIVER;
    //     //     colorsortdrivercounter = 0;
    //     //     master.rumble("...");
    //     //     rumblecounter = 1;
    //     //     // master.set_text(3, 0, "..");
    //     // }

    //     // if (rumblecounter > 0){
    //     //     rumblecounter++;
    //     // }
    //     // else if (rumblecounter > 20){
    //     //     rumblecounter = 0;
    //     // }

    //     // if (rightBack){
    //     //     raised = !raised;
    //     //     raiseasdasd.set_value(raised);
    //     // }

    //     // if (upbutton){
    //     //     speedbool = !speedbool;
    //     // }

    //     bool leftClamp = master.get_digital_new_press(DIGITAL_LEFT);

    //     if (leftClamp){
    //         ladybrown_manual.store(false);
    //         ladybrown_class.set_angle(190);
    //         pros::delay(500);
    //         move(-80, 0);
    //         // returnLB = true;
    //         ladybrown_class.set_angle(0);
    //         autoclamp_bool = true;
    //         pros::delay(700);
    //         autoclamp_bool = false;
    //         move(0,0);
    //         // chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    //         // pros::delay(200);
    //         // chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    //         returnLB = true;
    //         // chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    //         // clamp.set_value(true);
    //         // intake_class.set_velocity(127);
    //         // pros::delay(700);
    //         // intake_class.set_velocity(0);
    //         // pros::delay(200);
    //         // chassis.moveToPoint(0, 12, 500, {.forwards=true, .maxSpeed = 80});
    //         // chassis.turnToHeading(-89, 550, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 55});
    //         // chassis.moveToPoint(21, 13, 700, {.forwards=false, .maxSpeed = 80});
    //         // pros::delay(600);
    //         // clamp.set_value(false);
    //         // ladybrown_manual = false;
    //     }
    //     if(returnLB){
    //         ladybrown_manual.store(true);
    //         wallmotor.move(-127);
    //         pros::delay(400);
    //         wallrot.reset_position();
    //         wallmotor.move(0);
    //         ladybrown_manual.store(false);
    //     }
        
    //     pos = wallrot.get_position()/100.0f;
    //     err = targetpos - pos;
    //     float derr = err-preverr;
    //     sumerrorwall += err;
    //     float ret = pwall * err + iwall * sumerrorwall + dwall * derr;
    //     if(manual){goto skp;}
    //     // wallmotor.move((ret * 127) > 127 ? 127 : (ret*127));
    //     // bool yesredirect = master.get_digital(DIGITAL_L1);
    //     // bool notredirect = master.get_digital(DIGITAL_L2);

        
    //     if(wallcycler && !lastCycle){
    //         if(stage == 4){
    //             stage = 2;
    //         goto skprest;
    //         }
    //         stage = ((stage <= 1) ? stage+1 : 1);
    //         skprest:
    //         if(stage == 2){
    //             intaking = false;
    //             outtaking = false;
    //             intake_class.set_velocity(-127);
    //             pros::delay(50);
    //             intake_class.set_velocity(0);
    //         }
    //     }
        
    //     if(wallDown) {
    //         intake.move(0);
    //         intaking = false;
    //         outtaking = false;
    //         chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    //         move(-70, 0);
    //         pros::delay(150);
    //         move(0, 0);
    //         pros::delay(100);
    //         chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    //         //moveDown = false;
    //         if(stage != 3){
    //             //moveDown = true;
    //         }
    //         stage = (stage != 3) ? 3 : 0;
    //         sumerrorwall = 0.0f;
    //     }
    //     if(wallMid) {
    //         moveDown = false;
    //         if(stage != 4){
    //             intake_class.releasedMogo += 40;
    //         }
    //         stage = (stage != 4) ? 4 : 0;
    //         sumerrorwall = 0.0f;
    //         intaking = false;
    //         outtaking = false;
    //         intake_class.set_velocity(-127);
    //         pros::delay(50);
    //         intake_class.set_velocity(0);
    //     }
    //     if(wallcycler || wallreturn){
    //         moveDown = false;
    //         sumerrorwall = 0.0f;
    //     }
    //     if(wallreturn){
    //         moveDown = false;
    //         sumerrorwall = 0.0f;
    //         stage = 0;
    //     }
    //     // if(fabs(wallrot.get_angle() % 360) < 3.0f){
    //     //     wallrot.set_position(0);
    //     // }

        

    //     targetpos = (stage == 1) * targetWait + (stage == 2) * targetTop + (stage == 3) * targetDown + (stage == 4) * targetHold;
        
    //     ladybrown_class.set_angle(targetpos);
    //     goto skp2;
        
    //     skp:;

    //     skp2:;

    //     bool clampbutton = master.get_digital_new_press(DIGITAL_R1);

    //     doinker_button = master.get_digital_new_press(DIGITAL_A);

    //     if(clampbutton){
    //         clamped2 = !clamped2;
    //         clamp.set_value(clamped2);
    //         if(clamped2){
    //             intake_class.releasedMogo+=40;
    //         }
    //     }

    //     // if(master.get_digital_new_press(DIGITAL_UP)){
    //     // }

    //     if(doinker_button){
    //         doinker_state = !doinker_state;
    //         doinker.set_value(doinker_state);
    //     }   
        
    //     // if (upbutton){
    //     //     badcolor = bad_color ? badcolor != DRIVER : DRIVER;
    //     // }

    //     // if (!clamped2){
    //     //     master.set_text(3, 0, ".");
    //     // }
    //     else{
    //         master.set_text(3, 0, "");
    //     }
        
    //     chassis.arcade(power, turn);

    //     if (intakebutton){
    //         outtaking = false;
    //         intaking = !intaking; 
    //     }
    //     else if (outtakebutton){
    //         intaking = false;
    //         outtaking = !outtaking;
    //     }

    //     if (stage == 1){
    //         intake_class.maxTorque = 0.5;
    //     }
    //     else {
    //         intake_class.maxTorque = 0.5;
    //     }
    //     if (outtaking){
    //         intake_class.set_velocity(127);
    //         intake2.move(127);
    //     }
    //     if (intaking){
    //         intake_class.set_velocity(-127);
    //         intake2.move(-127);
    //     }
    //     if(!intaking && !outtaking){
    //         intake_class.set_velocity(0);
    //         intake2.move(0);
    //     }
    // }
}