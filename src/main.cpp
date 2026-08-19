#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/imu.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/motor_group.hpp"
#include "pros/rtos.hpp"
#include <sys/_intsup.h>

// =========================================================
// Robot configuration
// =========================================================

// The main controller used to drive the robot
pros::Controller master(pros::E_CONTROLLER_MASTER);

// Left and right drive motors for the drivetrain.
pros::MotorGroup leftDt({1,2,3}, pros::MotorGearset::green);
pros::MotorGroup rightDt({4,5,6}, pros::MotorGearset::green);

// Sensors used for odometry and orientation.
// The IMU provides heading data; the two rotations track wheel movement.
pros::Imu imu(7);
pros::Rotation horizontal(8);
pros::Rotation vertical(9);

// Tracking wheels attached to the horizontal and vertical odometry axes.
// These help LemLib estimate the robot's position on the field.
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

// Drivetrain tuning values for the robot's physical dimensions and drive setup.
// This defines the motor group, wheel size, gear ratio, and other chassis info.
lemlib::Drivetrain drive(
	&leftDt, 
	&rightDt,
	10,
	lemlib::Omniwheel::NEW_325,
	360,
	2
);

// PID settings for lateral movement (forward/backward positioning).
// These values tune how the robot corrects for drift and error while driving.
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

// PID settings for angular movement (turning and heading control).
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
// Robot startup
// =========================================================
void selectorHelper() {
	std::string autoNames[3] = {"Left", "Right", "Skills"};
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

		pros::delay(50);
}
void autoSelector(bool manual = false) {
	if (!manual) {
		while (!pros::competition::is_autonomous() && pros::competition::is_connected()) {
			selectorHelper();
		}
	} else {
		while (!(master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT) && master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT))) {
			selectorHelper();
		}
	}
	

}
void initialize() {
	// Initialize the V5 LCD screen for debug output.
	pros::lcd::initialize();

	// Calibrate the chassis before autonomous/opcontrol begins.
	chassis.calibrate();

	// Background task that continuously prints the robot's current field position.
	pros::Task displayInfo([&]() {
		while (true) {
			pros::lcd::print(0, "X: %f", chassis.getPose().x);
			pros::lcd::print(1, "Y: %f", chassis.getPose().y);
			pros::lcd::print(2, "Theta: %f", chassis.getPose().theta);
			pros::delay(20);
		}
	});

	autoSelector();
}

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
}

void opcontrol() {
	while (true) {
		// Drive the robot using tank controls from the master controller.
		chassis.tank(master.get_analog(ANALOG_LEFT_Y), master.get_analog(ANALOG_RIGHT_Y));
		pros::delay(20);

		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)&& master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
			while (master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT) && master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
				pros::delay(20);
			}
			autoSelector(true);
		}
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP) && master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
			while (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP) && master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
				pros::delay(20);
			}
			runAuto();
		}
	}
}

void autonomous() {
	runAuto();
}