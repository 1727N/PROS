#include "main.h"
#include "lemlib/api.hpp"
#include <algorithm>

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// drive motors
pros::Motor lF(-9, pros::E_MOTOR_GEARSET_06); // left front motor. port 12, reversed
pros::Motor lM(-19, pros::E_MOTOR_GEARSET_06); // left middle motor. port 11, reversed
pros::Motor lB(-20, pros::E_MOTOR_GEARSET_06); // left back motor. port 1, reversed
pros::Motor rF(2, pros::E_MOTOR_GEARSET_06); // right front motor. port 2
pros::Motor rM(12, pros::E_MOTOR_GEARSET_06); // right middle motor. port 11
pros::Motor rB(11, pros::E_MOTOR_GEARSET_06); // right back motor. port 13

// other motors
pros::Motor intake(5,pros::E_MOTOR_GEARSET_18);
pros::Motor kicker(6, pros::E_MOTOR_GEARSET_18);

pros::ADIDigitalOut LWing('D');
pros::ADIDigitalOut RWing('B');

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
                              10, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
                              360, // drivetrain rpm is 360
                              2 // chase power is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(20, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            45, // derivative gain (kD)
                                            1, // anti windup
                                            0.1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            0.3, // large error range, in inches
                                            400, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(3.5, // proportional gain (kP)
                                             0.1, // integral gain (kI)
                                             23, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             2, // large error range, in degrees
                                             400, // large error range timeout, in milliseconds
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
        lemlib::Pose pose(0, 0, 45);
        chassis.setPose(0, 0, 45);
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
    chassis.setPose(0, 0, 45);
    intake.move_voltage(3000);
    chassis.moveToPoint(0, 22, 5000, {.minSpeed = 80});

    chassis.turnToHeading(90, 1200);
    intake.move_voltage(-3000);
    pros::delay(800);
    intake = 0;
    chassis.turnToHeading(-90, 1200);
    
    chassis.moveToPoint(6, 25, 10000, {.forwards = false});
    //chassis.moveToPoint(6, 25, 10000, false, 127, false);

}

void farSide()
{

}

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
void autonomous() {
    nearSide();
    //chassis.moveToPoint(0, 10, 10000);
    //chassis.turnToHeading(90, 10000);
    //chassis.moveToPoint(0, 0, 10000);

    // example movement: Move to x: 20 and y: 15, and face heading 90. Timeout set to 4000 ms
        //chassis.moveToPose(20, 20, 15, 800);
        //chassis.moveToPose(20, 20, 30, 800);
        //chassis.moveToPose(20, 20, 45, 800);

    // example movement: Move to x: 0 and y: 0 and face heading 270, going backwards. Timeout set to 4000ms
        //chassis.moveToPose(0, 0, 0, 2000, {.forwards = false});
    // cancel the movement after it has travelled 10 inches
        //chassis.waitUntil(10);
        //chassis.cancelMotion();
    // example movement: Turn to face the point x:45, y:-45. Timeout set to 1000
    // dont turn faster than 60 (out of a maximum of 127)
        //chassis.turnToPoint(45, -45, 1000, {.maxSpeed = 60});
    // example movement: Turn to face a direction of 90º. Timeout set to 1000
    // will always be faster than 100 (out of a maximum of 127)
        //chassis.turnToHeading(90, 1000, {.minSpeed = 100});
    // example movement: Follow the path in path.txt. Lookahead at 15, Timeout set to 4000
    // following the path with the back of the robot (forwards = false)
    // see line 116 to see how to define a path
        //chassis.follow(example_txt, 15, 4000, false);
    // wait until the chassis has travelled 10 inches. Otherwise the code directly after
    // the movement will run immediately
    // Unless its another movement, in which case it will wait
        //chassis.waitUntil(10);
        //pros::lcd::print(4, "Travelled 10 inches during pure pursuit!");
    // wait until the movement is done
        //chassis.waitUntilDone();
        //pros::lcd::print(4, "pure pursuit finished!");
}

void arcade(){
    // get joystick positions
    int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
    // move the chassis with curvature drive
    chassis.curvature(leftY, rightX);
}

void intakeControl(float volts){
    intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

    volts = volts * 1000;

    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
        intake.move_voltage(volts);
    }
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
        intake.move_velocity(-10000);
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

void kickerControl(float volts){
    kicker.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

    volts = volts * 1000;

    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
        kicker.move_voltage(-volts);
    }
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
        kicker.brake();
    }
}

/**
 * Runs in driver control
 */
void opcontrol() {
    // controller
    // loop to continuously update motors
    while (true) {
        arcade();
        intakeControl(8);
        kickerControl(8.5);
        wingControl();

        // delay to save resources
        pros::delay(10);
    }
}
