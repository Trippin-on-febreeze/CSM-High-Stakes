#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep

using namespace pros;

// define global variables here
int autonSelect = 1;

// (more here)

// VEXcode device constructors
Controller Controller1(CONTROLLER_MASTER);

Motor  
	Intake(7, v5::MotorGears::blue, v5::MotorUnits::degrees),
	ClawMotor(-8, v5::MotorGears::blue, v5::MotorUnits::degrees);

ADIDigitalOut 
	clawTiltPiston('E'),
	clawPiston('D'),
	mogoPiston('C');

// Drivetrain configuration
Imu imu(12); 

MotorGroup 
	leftMotors({-1, -3, -5}, MotorGearset::blue),
	rightMotors({2, 4, 6}, MotorGearset::blue);

lemlib::Drivetrain drivetrain(
	&leftMotors,
    &rightMotors,
    12.75, // 12.75 inch track width
	lemlib::Omniwheel::NEW_275, // using new 2.75" omnis
    450, // drivetrain rpm is 450
    2 // horizontal drift is 2 (for now)
);

// horizontal tracking wheel encoder
Rotation odometryH(-11);
// vertical tracking wheel encoder
Rotation odometryV(-10);
// horizontal tracking wheel
lemlib::TrackingWheel horizontalTrackingWheel(&odometryH, lemlib::Omniwheel::NEW_2, -2.0); 
// vertical tracking wheel
lemlib::TrackingWheel verticalTrackingWheel(&odometryV, lemlib::Omniwheel::NEW_2, 5.0625); //keep right tracking wheel

lemlib::OdomSensors sensors(
	&verticalTrackingWheel, // vertical tracking wheel 1, set to null
    nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
    &horizontalTrackingWheel, // horizontal tracking wheel 1
	nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
    &imu // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateralController(
	7.45, // proportional gain (kP)
    0, // integral gain (kI)
    6, // derivative gain (kD)
	3, // anti windup
    1, // small error range, in inches
    100, // small error range timeout, in milliseconds
    3, // large error range, in inches
    500, // large error range timeout, in milliseconds
    0 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angularController(
	2.15, // proportional gain (kP)
    0, // integral gain (kI)
    13.2, // derivative gain (kD)
    3, // anti windup
    1, // small error range, in inches
    100, // small error range timeout, in milliseconds
    3, // large error range, in inches
    500, // large error range timeout, in milliseconds
    0 // maximum acceleration (slew)
);

// create the chassis
lemlib::Chassis chassis(
	drivetrain, // drivetrain settings
    lateralController, // lateral PID settings
    angularController, // angular PID settings
    sensors // odometry sensors
);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

// callback functions for lcd buttons
// void on_left_button () {
//     autonSelect = 1;
// }

// void on_center_button () {
//     autonSelect = 2;
// }

// void on_right_button () {
//     autonSelect = 3;
// }

// this runs at the start of the program
void initialize() {
    pros::lcd::initialize(); // initialize brain screen

    chassis.calibrate(); // calibrate sensors

    // print position to brain screen
    pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            
            pros::delay(20); // delay to save resources
        }
    });

    if(std::isnan(chassis.getPose().x) || std::isnan(chassis.getPose().y)) {
        chassis.calibrate();
    }

    ClawMotor.set_brake_mode(MOTOR_BRAKE_HOLD);
    ClawMotor.move(-127);
    delay(260);
    ClawMotor.brake();
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
void competition_initialize() {
    // if (pros::lcd::read_buttons() == 100) {
    //     on_left_button();
    // } else if (pros::lcd::read_buttons() == 010) {
    //     on_center_button();
    // } else if (pros::lcd::read_buttons() == 001) {
    //     on_right_button;
    // }

    // pros::lcd::register_btn0_cb(on_left_button);
    // pros::lcd::register_btn1_cb(on_center_button);
    // pros::lcd::register_btn2_cb(on_right_button);
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
    switch(autonSelect) {
        case 1: // right side red auton
            // chassis.setPose(93.85 + 7, 12.0, 180);
                    
            chassis.setPose(93.85, 12.0, 180);
            chassis.moveToPoint(93.85, 32.0, 4000, {.forwards = false});
            chassis.moveToPoint(93.85, 48, 4000, {.forwards = false, .maxSpeed = 60});
            delay(600);
            mogoPiston.set_value(true); //clamp stake
            delay(200);

            chassis.turnToHeading(90, 4000);
            Intake.move(127);
            chassis.moveToPoint(120, 48, 4000);
            delay(1000); 

            chassis.moveToPoint(103.84, 46.64, 4000, {.forwards = false});
            delay(1000);
            mogoPiston.set_value(false); // let go of first stake

            chassis.moveToPose(90, 28, -90, 4000, {.minSpeed = 40}); //intake stack
            chassis.moveToPose(25, 28, -90, 4000, {.maxSpeed = 60});
            delay(2330);
            Intake.move(0);

            chassis.moveToPoint(36, 42, 10000, {.forwards = false, .maxSpeed = 60});
            chassis.moveToPoint(36, 53, 10000, {.forwards = false, .maxSpeed = 40});
            delay(500);
            mogoPiston.set_value(true);
            delay(300);

            Intake.move(127);
            chassis.moveToPose(44, 32, 0, 4000, {.maxSpeed = 60, .minSpeed = 30});
            delay(500);
            mogoPiston.set_value(false);

            chassis.moveToPose(53, 68, 45, 100000, {.forwards = false});

            break;
        case 3:
        //skills auton
            chassis.setPose(70.20, 9.7, 0);

    }
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
    ClawMotor.move(-127);
    delay(260);
    ClawMotor.brake();

    int 
    speed = 0, 
    turn = 0, 
    driveDirection = 1, 
    leftDrive = 0, 
    rightDrive = 0;

    bool 
    clawToggle1 = true, 
    clawToggle2 = true, 
    mogoToggle = true;

  while(true) {
    // Arcade control scheme
        int leftY = Controller1.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);  // get left y and right x positions
        int rightX = Controller1.get_analog(E_CONTROLLER_ANALOG_RIGHT_X);
        chassis.arcade(leftY, rightX); // move the robot

    // Intake motor control
    if(Controller1.get_digital(E_CONTROLLER_DIGITAL_R1)) {
      Intake.move(110);
    } else if (Controller1.get_digital(E_CONTROLLER_DIGITAL_R2)){
      Intake.move(-110);
    } else Intake.brake(); 

    // Reverse drive direction
    if(Controller1.get_digital(E_CONTROLLER_DIGITAL_UP)) {
      driveDirection = 1;
    } else if (Controller1.get_digital(E_CONTROLLER_DIGITAL_DOWN)){
      driveDirection = -1;
    }

    // Claw arm motor control
    if(Controller1.get_digital(E_CONTROLLER_DIGITAL_L1)) {
      ClawMotor.move(70);
    } else if(Controller1.get_digital(E_CONTROLLER_DIGITAL_L2)) {
      ClawMotor.move(-70);
    } else ClawMotor.brake(); 

    // Toggle for claw piston
    if(Controller1.get_digital(E_CONTROLLER_DIGITAL_A)) {
      clawPiston.set_value(clawToggle1);
      clawToggle1 = !clawToggle1;
      delay(200);
    }

    // Toggle for tilting claw piston
    if(Controller1.get_digital(E_CONTROLLER_DIGITAL_X)) {
      clawTiltPiston.set_value(clawToggle2);
      clawToggle2 = !clawToggle2;
      delay(200);
    }

    // Toggle for mogo piston
    if(Controller1.get_digital(E_CONTROLLER_DIGITAL_Y)) {
      mogoPiston.set_value(mogoToggle);
      mogoToggle = !mogoToggle;
      delay(200);
    } 

    if(Controller1.get_digital(E_CONTROLLER_DIGITAL_A) && Controller1.get_digital(E_CONTROLLER_DIGITAL_B)) {
            //initial positions of the robot
            chassis.setPose(70.20, 9.7, 0);

            chassis.moveToPose(70.20, 27, 180, 10000);
            chassis.moveToPose(80, 15, -120, 10000, {.forwards = false});
        }

    delay(20);
  }                  
}