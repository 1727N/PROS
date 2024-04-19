#include "main.h"
#include "lemlib/api.hpp"
#include <algorithm>
#include <cmath>

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// drive motors
pros::Motor lF(-20, pros::E_MOTOR_GEARSET_06); // left front mot or. port 12, reversed
pros::Motor lM(-18, pros::E_MOTOR_GEARSET_06); // left middle motor. port 11, reversed
pros::Motor lB(-16, pros::E_MOTOR_GEARSET_06); // left back motor. port 1, reversed
pros::Motor rF(11, pros::E_MOTOR_GEARSET_06); // right front motor. port 2
pros::Motor rM(13, pros::E_MOTOR_GEARSET_06); // right middle motor. port 11
pros::Motor rB(15, pros::E_MOTOR_GEARSET_06); // right back motor. port 13

// other motors
pros::Motor intakeLeft(9, pros::E_MOTOR_GEARSET_18);
pros::Motor intakeRight(-2, pros::E_MOTOR_GEARSET_18);

pros::ADIDigitalOut LWing('A');
pros::ADIDigitalOut RWing('B');

pros::ADIDigitalOut PTO('C');
pros::ADIDigitalOut Clamp('D');

// motor groups
pros::MotorGroup leftMotors({lF, lM, lB}); // left motor group
pros::MotorGroup rightMotors({rF, rM, rB}); // right motor group
pros::MotorGroup intake({intakeLeft, intakeRight});

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
lemlib::ControllerSettings linearController(13, // proportional gain (kP)
                                            0.3, // integral gain (kI)
                                            39, // derivative gain (kD)
                                            1, // anti windup
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

bool ptoToggle;

void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors

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

ASSET(example_txt); // '.' replaced with "_" to make c++ happy

void autonWait(){
    pros::delay(3000);
}

void nearSide()
{
    chassis.setPose(0, 0, 45);
    //bakc it up
    chassis.moveToPoint(-16, -14, 2000, lemlib::MoveToPointParams{.forwards = false, .maxSpeed = 80});
    //trick pid to get more speed
    chassis.moveToPoint(-60, -16, 2000, {.forwards = false});
    chassis.waitUntil(6);
    chassis.cancelMotion();

    //chassis.moveToPoint(-14, -14, 2000);


    chassis.moveToPoint(-7, -4, 2000);
    chassis.waitUntil(3);

    RWing.set_value(true);
    chassis.moveToPose(-1, 27.5, 0, 3000);
    chassis.waitUntil(2);
    RWing.set_value(false);


    //chassis.moveToPoint(-1, 32, 3000, {.forwards = false});
}

void DNUfarSideSafe()
{
    intake = 60;
    pros::delay(500);
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

void DNUfarSide()
{

    chassis.setPose(0, 0, 180);

    intake = 70;

    chassis.moveToPoint(0, -7, 1000);

    chassis.moveToPoint(-3.5, 29.5, 1500, {.forwards = false});
    
    //align properly
    chassis.turnToHeading(145, 1000);
    
    //LBWing.set_value(true);

    chassis.moveToPoint(-19.5, 53, 1350, {.forwards = false});
    chassis.waitUntil(14);
    //LBWing.set_value(false);
    
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
    //pick up ball
    chassis.turnToHeading(-50, 1000);
    chassis.waitUntilDone();
    LWing.set_value(true);
    //score balls maybe
    
    chassis.moveToPoint(-66, 54, 1400);
    chassis.waitUntil(8);
    intake = -70;
    chassis.waitUntil(47);
    LWing.set_value(false);
    // RWing.set_value(false);
    chassis.moveToPoint(-30, 25, 1200, {.forwards = false});
}

void farSafe(){
    intake = 60;

    chassis.moveToPoint(0, 36, 2000, lemlib::MoveToPointParams{.maxSpeed = 80});
    chassis.waitUntil(10);
    intake = 20;

    /* LEFT SIDE -- TESTING ONLY 
    chassis.turnToHeading(-90, 1200);
    chassis.waitUntilDone();

    intake = -127;
    chassis.moveToPoint(-8, 36, 2000);
    chassis.waitUntil(6);
    intake = 0;

    chassis.moveToPoint(0, 36, 2000, {.forwards = false});
    */

    // RIGHT SIDE -- REAL MATCH
    chassis.turnToHeading(90, 1200);
    chassis.waitUntilDone();

    intake = -127;
    chassis.moveToPoint(8, 36, 2000);
    chassis.waitUntil(6);
    intake = 0;

    chassis.moveToPoint(0, 36, 2000, {.forwards = false});
}

void autonomous() {
    //autonWait();
    
    //nearSide();
    farSafe();


    //farSide();  
    //farSideSafe();
}

// DRIVE CONTROL -------------------------------------------------------------------------------------------- // 
//              -------------------------------------------------------------------------------------------- //
//             -------------------------------------------------------------------------------------------- //

void arcade(){
    int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    float turnScalingFactor = abs(leftY) <= 10 ? 1 : 0.7; 

    float leftPower = leftY + rightX * turnScalingFactor;
    float rightPower = leftY - rightX * turnScalingFactor;

    if (ptoToggle){
        if (leftPower > 0) leftPower = 0;
        if (rightPower > 0) rightPower = 0;
    }

    leftMotors.move(leftPower);
    rightMotors.move(rightPower);
    //chassis.curvature(leftY, rightX);
}

void intakeControl(int power){
    intakeLeft.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    intakeRight.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

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

void wingControl(){
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)){
        LWing.set_value(true);
        RWing.set_value(true);
    }
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
        LWing.set_value(false);
        RWing.set_value(false);
    }  
}

void ptoControl(){
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT) && controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)){
        ptoToggle = !ptoToggle;
        PTO.set_value(ptoToggle);
    }
}

bool unclamped;

void clampControl(){
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN) && controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
        unclamped = true;
    }
    Clamp.set_value(unclamped);
}


/**
 * Runs in driver control
 */
void opcontrol() {
    LWing.set_value(false);
    RWing.set_value(false);
    // controller
    // loop to continuously update motors
    while (true) {
        arcade();
        intakeControl(80);
        wingControl();
        clampControl();
        ptoControl();

        // delay to save resources
        pros::delay(10);
    }
}
