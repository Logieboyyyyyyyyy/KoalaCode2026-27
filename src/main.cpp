// Include the vector library
#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "lemlib/pid.hpp"
#include "liblvgl/llemu.h"
#include "liblvgl/llemu.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/imu.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"
#include "pros/rtos.hpp"
#include <cstdio>
#include <sys/_intsup.h>
#include <vector>


// =========================================================
// Robot configuration
// =========================================================

// The main controller used to drive the robot
pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::MotorGroup cascade({10, -11}, pros::MotorGearset::blue);

// Left and right drive motors for the drivetrain.
pros::MotorGroup leftDt({1,2,3}, pros::MotorGearset::blue);
pros::MotorGroup rightDt({4,5,6}, pros::MotorGearset::blue);

// Sensors for odometry
pros::Imu imu(7);
pros::Rotation horizontal(8);
pros::Rotation vertical(9);

// Define the rotaion sensors as tracking wheels for odometry.
lemlib::TrackingWheel horizontalTracker(
	&horizontal, 
	lemlib::Omniwheel::NEW_2, 
	0
);
lemlib::TrackingWheel verticalTracker(
	&vertical, 
	lemlib::Omniwheel::NEW_2, 
	0
);

// Tell LemLib which sensors are being used for odometry.
lemlib::OdomSensors sensors(
	&verticalTracker, 
	nullptr,
	&horizontalTracker, 
	nullptr,
	&imu
);

// Drivetrain values for the robot's physical dimensions and drive setup.
lemlib::Drivetrain drive(
	&leftDt, 
	&rightDt,
	10,
	lemlib::Omniwheel::NEW_275,
	450,
	2
);

// PID settings for lateral movement
lemlib::ControllerSettings lateral(
	10,
	0,
	3,
	3,
	1,
	100,
	3,
	500,
	20
);

// PID settings for angular movement
lemlib::ControllerSettings angular(
	3,
	0,
	10,
	3,
	1,
	100,
	3,
	500,
	0
);

// PID settings for the cascade mechanism
lemlib::PID cascadePID(
	1,
	0,
	0,
	0,
	false
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttle_curve(
	3, // joystick deadband out of 127
    10, // minimum output where drivetrain will move out of 127
    1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steer_curve(
	3, // joystick deadband out of 127
    10, // minimum output where drivetrain will move out of 127
    1.019 // expo curve gain
);

// Build the chassis object using the drivetrain, PID settings, and sensors.
lemlib::Chassis chassis(
	drive,
	lateral,
	angular,
	sensors,
	&throttle_curve,
	&steer_curve
);

//Static variables
bool DEBUG = true;
std::vector<std::string> AUTOS = {"left", "right", "skills"};

//Non Static variables
int auton = 0;
bool inputsEnabled = false;

// Hot Motor Detection
std::string hotMotors() {
	if (DEBUG) {printf("hotMotors\n");}
	std::string hotMotors = "";
	for (int i = 0; i < 3; i++) {
		if (leftDt.is_over_temp(i)) {
			hotMotors += "Left-Drivetrain " + std::to_string(i) + " ";
		}
		if (rightDt.is_over_temp(i)) {
			hotMotors += "Right-Drivetrain " + std::to_string(i) + " ";
		}
	}
	if (cascade.is_over_temp(0)) {
		hotMotors += "Cascade 0";
	}
	if (cascade.is_over_temp(1)) {
		hotMotors += "Cascade 1";
	}

	return hotMotors;
}

void autoSelector(){
	if (DEBUG) {printf("autoSelector\n");}
	if (pros::lcd::read_buttons() == LCD_BTN_LEFT) {
		auton--;
		if (auton < 0) {
			auton = AUTOS.size()-1;
		}
		while (pros::lcd::read_buttons() == LCD_BTN_LEFT) {
			pros::delay(50);
		}
	} else if (pros::lcd::read_buttons() == LCD_BTN_RIGHT) {
		auton++;
		if (auton > AUTOS.size()-1) {
			auton = 0;
		}
		while (pros::lcd::read_buttons() == LCD_BTN_RIGHT) {
			pros::delay(50);
		}
	}
}

// Robot startup
void initialize() {
	if (DEBUG) {printf("initialize\n");}
	// Initialize the V5 LCD screen for DEBUG output.
	pros::lcd::initialize();

	// Calibrate the chassis before autonomous/opcontrol begins.
	chassis.calibrate();
	cascade.set_zero_position(cascade.get_position());

	// Background task that continuously prints the robot's current field position.
	pros::Task displayInfo([&]() {
		while (true) {
			// printf("X: %f Y: %f Theta: %f\n", chassis.getPose().x,chassis.getPose().y,chassis.getPose().theta);
			pros::lcd::print(0, "X: %f", chassis.getPose().x);
			pros::lcd::print(1, "Y: %f", chassis.getPose().y);
			pros::lcd::print(2, "Theta: %f", fmod(chassis.getPose().theta, 360));
			pros::lcd::print(3, "Hot Motors: %s", hotMotors().c_str());
			if (pros::competition::is_disabled()) {
				autoSelector();
				pros::lcd::print(4, "Auton: %s", AUTOS[auton]);
			} else {pros::lcd::clear_line(4);}
			pros::delay(500);
		}
	});
}

// Autonomous routines
void leftAuton() {
	if (DEBUG) {printf("leftAuton\n");}
}
void rightAuton() {
	if (DEBUG) {printf("rightAuton\n");}
}
void skillsAuton() {
	if (DEBUG) {printf("skillsAuton\n");}
}

//Run the selected autonimous
void runAuto() {
	if (DEBUG) {printf("runAuto\n");}
	inputsEnabled = false;
	switch (auton) {
		case 0: leftAuton(); break;
		case 1: rightAuton(); break;
		case 2: skillsAuton(); break;
		default: break;
	}
	master.rumble(". . .");
	inputsEnabled = true;
}

// Display inportent information on the controler
void controlerDisplay() {
	if (DEBUG) {printf("controlerDisplay\n");}
	int timer = 105000;
	while (true) {
		if (pros::competition::is_connected()) {
			if (
				!pros::competition::is_disabled() &&
				!pros::competition::is_autonomous()
			) {
				master.clear_line(2);
				master.print(2, 0, "Time (seconds): %d", timer/1000);
				pros::delay(1000);
				timer -= 1000;
				if (timer == 15000) {
					master.rumble("---");
				}
			} 
		} else {
			timer = 105000;
			pros::delay(50);
		}
	}
}

// =========================================================
// Driver control
// =========================================================
void opcontrol() { 
	if (DEBUG) {printf("opcontrol\n");}
	inputsEnabled = true;
	pros::Task drive([&]() {  
		while (true) {
			if (inputsEnabled) {
				chassis.tank(
					master.get_analog(ANALOG_LEFT_Y), 
					master.get_analog(ANALOG_RIGHT_Y));
			}
			pros::delay(20);
		}
	});

	pros::Task condisplay(controlerDisplay);

	pros::Task startAutoShortcut([&](){
		while (true) {		
			if (inputsEnabled) {
				if (
					!pros::competition::is_connected() &&
					master.get_digital(pros::E_CONTROLLER_DIGITAL_UP) && 
					master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)
				) {

					while (
						master.get_digital(pros::E_CONTROLLER_DIGITAL_UP) && 
						master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)
					) {

						pros::delay(20);
					}
					runAuto();
				}
			}
			pros::delay(20);
		}
	});

	pros::Task runCascade([&](){
		int target = 0;
		while (true) {
			int error = target - cascade.get_position();
			cascade.move_voltage(cascadePID.update(error)*1000);
		}
	});
}

void autonomous() {
	if (DEBUG) {printf("autonomous\n");}
	runAuto();
}