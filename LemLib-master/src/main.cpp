#include "main.h"
#include "lemlib/api.hpp"
#include <algorithm>
#include <cmath>

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// drive motors
pros::Motor lF(-9, pros::E_MOTOR_GEARSET_06); // left front motor. port 12, reversed
pros::Motor lM(-19, pros::E_MOTOR_GEARSET_06); // left middle motor. port 11, reversed
pros::Motor lB(-20, pros::E_MOTOR_GEARSET_06); // left back motor. port 1, reversed
pros::Motor rF(2, pros::E_MOTOR_GEARSET_06); // right front motor. port 2
pros::Motor rM(12, pros::E_MOTOR_GEARSET_06); // right middle motor. port 11
pros::Motor rB(13, pros::E_MOTOR_GEARSET_06); // right back motor. port 13

// other motors
pros::Motor intake(5,pros::E_MOTOR_GEARSET_18);
pros::Motor kicker(6, pros::E_MOTOR_GEARSET_18);

pros::ADIDigitalOut LWing('D');
pros::ADIDigitalOut RWing('B');

pros::ADIDigitalOut LBWing('E');
pros::ADIDigitalOut RBWing('A');

pros::ADIDigitalOut Hang('C');


// motor groups
pros::MotorGroup leftMotors({lF, lM, lB}); // left motor group
pros::MotorGroup rightMotors({rF, rM, rB}); // right motor group

// Inertial Sensor on port 1
pros::Imu imu(1);

// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 15, reversed (negative signs don't work due to a pros bug)
pros::Rotation horizontalEnc(15, true);
// horizontal tracking wheel. 2.75" diameter, 3.7" offset, back of the robot (negative)
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_325, -3.7);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              10.2, // 10.2 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
                              360, // drivetrain rpm is 360
                              2 // chase power is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(14, // proportional gain (kP)
                                            1, // integral gain (kI)
                                            70, // derivative gain (kD)
                                            3, // anti windup
                                            0.1, // small error range, in inches
                                            300, // small error range timeout, in milliseconds
                                            0.3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(4, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             27, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             300, // small error range timeout, in milliseconds
                                             1.5, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
// note that in this example we use internal motor encoders (IMEs), so we don't pass vertical tracking wheels
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            nullptr, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors);

int easy = 90;
int medium = 100;
int hard = 127;


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors

    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });
}

/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(example_txt); // '.' replaced with "_" to make c++ happy

void nearSide()
{
    pros::delay(3200);
    chassis.setPose(0, 0, 45);
    //bakc it up
    chassis.moveToPoint(-16, -12, 2000, {.forwards = false});
    //trick pid to get more speed
    chassis.moveToPoint(-60, -12, 2000, {.forwards = false});
    chassis.waitUntil(10);
    chassis.cancelMotion();

    chassis.moveToPoint(-16, -10, 2000);
    chassis.turnToHeading(-90, 1200);


    // chassis.moveToPoint(-14, -8, 2000, {.forwards = false});
    chassis.moveToPoint(-6, 2, 2000, {.forwards = false});
    // chassis.waitUntil(1);
    LBWing.set_value(true);
    chassis.waitUntil(10);
    LBWing.set_value(false);

    chassis.moveToPoint(-1, 8, 1000, {.forwards = false});
    chassis.turnToHeading(180, 1200);
    chassis.waitUntilDone();
    chassis.setPose(0,0,180);
    chassis.moveToPoint(-1, 32, 3000, {.forwards = false});
    // chassis.turnToHeading(180, 1200);
    // LBWing.set_value(true);
    // chassis.turnToHeading(190, 1200);
    // chassis.turnToHeading(30, 1200);
    // chassis.turnToHeading(45, 1200);
    // LBWing.set_value(true);


}

void farSideSafe()
{
    pros::delay(3200);
    intake = 60;
    Hang.set_value(true);
    pros::delay(500);
    Hang.set_value(false);
    chassis.moveToPoint(-2, 33, 1500);

    //outtake matchload
    chassis.turnToHeading(48, 1200);
    chassis.waitUntilDone();
    intake = -70;
    pros::delay(500);
    // intake = 0

    //grab first ball
    intake = 70;
    chassis.turnToHeading(270, 1200);
    chassis.moveToPoint(-29, 33, 1500);
    chassis.moveToPoint(-14, 33, 1000, {.forwards = false});


    //outtake first ball
    chassis.turnToHeading(35, 1200);
    chassis.waitUntilDone();
    intake = -80;
    pros::delay(1000);

    //prep for second ball

    //grab second ball
    intake = 60;
    chassis.turnToHeading(-45, 1200);
    chassis.moveToPoint(-35, 54, 1500);
    // chassis.moveToPoint(-29, 47, 1500);
    chassis.turnToHeading(90, 1200);
    chassis.waitUntilDone();

    //outtake + wings
    LWing.set_value(true);
    RWing.set_value(true);
    chassis.moveToPoint(5, 52, 1500);
    chassis.waitUntil(10);
    intake = -100;
    chassis.waitUntilDone();
    LWing.set_value(false);
    RWing.set_value(false);
    intake = 0;
    chassis.moveToPoint(-25, 25, 1200, {.forwards = false});
}

void farSide()
{
    pros::delay(3200);
    Hang.set_value(true);
    pros::delay(500);
    Hang.set_value(false);

    chassis.setPose(0, 0, 180);

    intake = 70;

    chassis.moveToPoint(0, -7, 1000);

    chassis.moveToPoint(-3.5, 29.5, 1500, {.forwards = false});
    
    //align properly
    chassis.turnToHeading(145, 1000);
    
    //chassis.moveToPoint(-10, 44, 2000, {.forwards = false});

    LBWing.set_value(true);

    // chassis.moveToPoint(-13, 44, 1200, {.forwards = false});
    
    chassis.moveToPoint(-19.5, 53, 1350, {.forwards = false});
    chassis.waitUntil(14);
    LBWing.set_value(false);
    //chassis.moveToPoint(-20.3, 52.3, 1500, {.forwards = false});
    //chassis.moveToPose(-15, 58, 90, 2000, {.forwards = false});
    //chassis.waitUntil(10);
    //LBWing.set_value(true);
    // chassis.waitUntil(20);
    // LBWing.set_value(false);
    
    chassis.turnToHeading(115, 1000);
    chassis.moveToPoint(-60, 51.5, 1500, {.forwards = false});
    chassis.waitUntil(8.75);
    chassis.cancelMotion();
    //push in goal

    chassis.turnToHeading(180, 900);
    chassis.moveToPoint(-22, 40, 650);
    chassis.moveToPoint(-33, 24, 650);
    //get out of goal

    chassis.turnToHeading(327, 1000);
    chassis.waitUntilDone();
    intake = -127;
    pros::delay(450);
    intake = 70;
    //outake ball
    chassis.turnToHeading(178, 900);
    chassis.moveToPoint(-36, 0, 1500);
    // chassis.turnToHeading(-90, 1200);
    // chassis.moveToPoint(-38, 6, 1000);
    //pick up ball
    chassis.turnToHeading(-50, 1000);
    chassis.waitUntilDone();
    LWing.set_value(true);
    // RWing.set_value(true)
    //score balls maybe
    
    chassis.moveToPoint(-66, 54, 1400);
    chassis.waitUntil(8);
    intake = -70;
    chassis.waitUntil(47);
    LWing.set_value(false);
    // RWing.set_value(false);
    chassis.moveToPoint(-30, 25, 1200, {.forwards = false});
}


void skills(){
    chassis.setPose(0, 0, 45);
    Hang.set_value(true);
    pros::delay(100);
    
    intake = 70;
    chassis.moveToPoint(18, 19, 2000);
    chassis.waitUntil(3);
    Hang.set_value(false);

    chassis.turnToHeading(90, 1200);
    intake = -127;
    chassis.moveToPoint(27, 19, 1000);
    pros::delay(800);
    intake = 0;

    // //back it up
    // chassis.moveToPoint(16, 19, 5000, {.forwards = false});
    // chassis.turnToHeading(270, 1200);
    // //push it in
    // chassis.moveToPoint(27, 19, 2000, {.forwards = false});
    // chassis.moveToPoint(24, 19, 5000);
    // chassis.turnToHeading(90, 1200);

    //move to matchload
    chassis.moveToPoint(14, 19, 2000, {.forwards = false});
    chassis.turnToHeading(155, 1200);
    chassis.moveToPoint(11, 20, 1000, {.forwards = false});
    chassis.turnToHeading(160, 1200);
    chassis.setBrakeMode(MOTOR_BRAKE_HOLD);
    LBWing.set_value(true);

    //but heres the kicker
    //30 seconds
    kicker = -(easy+3) ;
    pros::delay(30000);
    kicker = 0;

    LBWing.set_value(false);
    chassis.setBrakeMode(MOTOR_BRAKE_COAST);
    chassis.moveToPoint(0,-5, 1000);
    chassis.moveToPoint(2, -74, 1500);
    chassis.turnToHeading(90, 1200);
   

    //prepare scooping
    chassis.moveToPoint(25, -74, 2000);
    RWing.set_value(true);
    chassis.waitUntil(15);
    RWing.set_value(false);
    chassis.turnToHeading(0, 1200);
    chassis.moveToPoint(25, -55, 2000);
    // chassis.turnToHeading(0, 1200);


    chassis.moveToPose(55, -30, 180, 2000);//center
    RWing.set_value(false);
    // chassis.moveToPoint(48, -38, 5000);
    
    
    // chassis.turnToHeading(180, 1200);
    chassis.turnToHeading(5, 1000);
    // RWing.set_value(true);
    // LWing.set_value(true);
    chassis.waitUntilDone();
    LBWing.set_value(true);

    chassis.moveToPoint(63, -82, 2000, {.forwards = false});//it goes in you feel it
    chassis.waitUntilDone();
    LWing.set_value(false);
    RWing.set_value(false);
    
    chassis.moveToPoint(63, -30, 2000 /*.forwards = false}*/);

    chassis.waitUntil(10);
    LBWing.set_value(false);

    chassis.turnToHeading(190, 1200);
    chassis.waitUntilDone();
    LWing.set_value(true);
    RWing.set_value(true);
    

    chassis.moveToPoint(67, -82, 2000);
    chassis.turnToHeading(180, 1200);
    chassis.waitUntilDone();

    // chassis.moveToPoint(68, -50, 2000, {.forwards = false});
    LWing.set_value(false);
    RWing.set_value(false);
    chassis.moveToPoint(10, -40, 1000, {.forwards = false});
}

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
void autonomous() {

    // nearSide();
    // farSide();  
    //farSideSafe();
    skills();
}

void arcade(){
    int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    float turnScalingFactor = abs(leftY) <= 10 ? 1 : 0.7; 

    float leftPower = leftY + rightX * turnScalingFactor;
    float rightPower = leftY - rightX * turnScalingFactor;

    leftMotors.move(leftPower);
    rightMotors.move(rightPower);
    //chassis.curvature(leftY, rightX);
}

void intakeControl(int power){
    intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
        intake = power;
    }
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
        intake = -127;
    }
    else {
        intake.brake();
    }
}

bool hangToggle;

void wingControl(){
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)){
        LWing.set_value(true);
        RWing.set_value(true);
    }
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
        LWing.set_value(false);
        RWing.set_value(false);
    }

    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)){
        LBWing.set_value(true);
    }
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)){
        LBWing.set_value(false);
    }   

    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)){
        hangToggle = !hangToggle;
        Hang.set_value(hangToggle);
    }
}

void kickerControl(int power){
    kicker.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
        kicker = -power;
    }
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
        kicker.brake();
    }
}



/**
 * Runs in driver control
 */
void opcontrol() {
    LBWing.set_value(false);
    LWing.set_value(false);
    RWing.set_value(false);
    // controller
    // loop to continuously update motors
    while (true) {
        arcade();
        intakeControl(80);
        kickerControl(easy+5);
        wingControl();

        // delay to save resources
        pros::delay(10);
    }
}
