// Include the vector library
#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "liblvgl/llemu.hpp"
#include "pros/imu.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"
#include "pros/rtos.hpp"
#include <cstdio>
#include <sys/_intsup.h>
#include <string.h>

// =========================================================
// Robot configuration
// =========================================================

// The main controller used to drive the robot
pros::Controller master(pros::E_CONTROLLER_MASTER);

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

int auton = 0;

// =========================================================
// Autonomous selector
// =========================================================
void selectorHelper() { //TODO: Change this to use brain screen becasue buttons disabled when on feild
	std::string autoNames[3] = {"Left", "Right", "Skills"};
	master.clear();
	master.print(0, 0, "Auton: %s", autoNames[auton].c_str());
	if (master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
		auton++;
		if (auton > 2) auton = 0;
		while (master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
			pros::delay(20);
		}
	}
	if (master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) {
		auton--;
		if (auton < 0) auton = 2;
		while (master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) {
			pros::delay(20);
		}
	}

	pros::delay(500);
}

void autoSelector(bool manual = false) {
	if (!manual) {
		while (!pros::competition::is_autonomous() && pros::competition::is_connected()) {
			selectorHelper();
		}
	} else {
		while (!(
			master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT) &&
			master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT))
		) {
			selectorHelper();
		}
	}
	

}

// =========================================================
// Hot Motor Detection
// =========================================================
std::string hotMotors() {
	std::string hotMotors = "";
	for (int i = 0; i < 3; i++) {
		if (leftDt.is_over_temp(i)) {
			hotMotors += "Left-Drivetrain " + std::to_string(i) + " ";
		}
		if (rightDt.is_over_temp(i)) {
			hotMotors += "Right-Drivetrain " + std::to_string(i) + " ";
		}
	}
	return hotMotors;
}

// =========================================================
// Robot startup
// =========================================================
void initialize() {
	// Initialize the V5 LCD screen for debug output.
	pros::lcd::initialize();

	// Calibrate the chassis before autonomous/opcontrol begins.
	chassis.calibrate();

	// Background task that continuously prints the robot's current field position.
	pros::Task displayInfo([&]() {
		while (true) {
			printf("X: %f Y: %f Theta: %f\n", chassis.getPose().x,chassis.getPose().y,chassis.getPose().theta);
			pros::lcd::print(0, "X: %f", chassis.getPose().x);
			pros::lcd::print(1, "Y: %f", chassis.getPose().y);
			pros::lcd::print(2, "Theta: %f", chassis.getPose().theta);
			// pros::lcd::print(3, "Hot Motors: %s", hotMotors().c_str());
			pros::delay(500);
		}
	});


	// autoSelector(); // Not implemented yet
}

// =========================================================
// Autonomous routines
// =========================================================
void leftAuton() {
	
}
void rightAuton() {
	
}
void skillsAuton() {
	
}

void runAuto() {
	switch (auton) {
		case 0: leftAuton(); break;
		case 1: rightAuton(); break;
		case 2: skillsAuton(); break;
		default: break;
	}
	master.rumble(". . .");
}

// =========================================================
// Run match timer and display it along with
// important information on controller
// =========================================================
void controlerDisplay() {
	int timer = 105000;
	while (true) {
		if (pros::competition::is_connected()) {
			if (
				!pros::competition::is_disabled() &&
				!pros::competition::is_autonomous()
			) {
				master.clear_line(2);
				master.print(2, 0, "Time (seconds): %d", timer/1000);
				pros::delay(50);
				timer -= 50;
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
void opcontrol() { //TODO: clean all this up
	pros::Task drive([&]() { //Temporarly using a task 
		while (true) {
			chassis.tank(
				master.get_analog(ANALOG_LEFT_Y), 
				master.get_analog(ANALOG_RIGHT_Y));
			pros::delay(20);
		}
	});
	pros::Task condisplay(controlerDisplay);

	while (true) {		
		if (
			!pros::competition::is_connected() &&
			master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT) && 
			master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)
		) {

			while (
				master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT) && 
				master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)
			) {

				pros::delay(20);
			}

			autoSelector(true);
		}

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
		pros::delay(20);
	}
}

void autonomous() {
	runAuto();
}