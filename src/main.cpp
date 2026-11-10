#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "liblvgl/llemu.h"
#include "liblvgl/llemu.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/imu.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include <cstdint>
#include <cstdio>
#include <sys/_intsup.h>

// =========================================================
// Robot configuration
// =========================================================

// The main controller used to drive the robot
pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::MotorGroup cascade({-6,8}, pros::MotorGearset::green);
pros::Motor banshee(9, pros::MotorGearset::green);
pros::Motor shoulder(21, pros::MotorGearset::green);
pros::Motor intake(18, pros::MotorGearset::green);

// Left and right drive motors for the drivetrain.
pros::MotorGroup leftDt({-1,-2}, pros::MotorGearset::blue);
pros::MotorGroup rightDt({3,4}, pros::MotorGearset::blue);

// Sensors for odometry
pros::Imu imu(5);
// pros::Rotation horizontal(8);
pros::Rotation vertical(-11);

// Define the rotaion sensors as tracking wheels for odometry.
// lemlib::TrackingWheel horizontalTracker(
// 	&horizontal, 
// 	lemlib::Omniwheel::NEW_2, 
// 	0
// );
lemlib::TrackingWheel verticalTracker(
	&vertical, 
	lemlib::Omniwheel::NEW_2, 
	0
);

// Tell LemLib which sensors are being used for odometry.
lemlib::OdomSensors sensors(
	&verticalTracker, 
	nullptr,
	nullptr,
	// &horizontalTracker, 
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
	20,
	0,
	150,
	0,
	1,
	100,
	0.5,
	500,
	0
);

// PID settings for angular movement
lemlib::ControllerSettings angular(
	4,
	0,
	25,
	0,
	.5,
	100,
	2,
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
	2, // joystick deadband out of 127
    5, // minimum output where drivetrain will move out of 127
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

//Constant variables
const bool DEBUG = true;
const bool MANUAL_SHOULDER = true;
const bool MANUAL_CASCADE = true;
const int THROTTLE_MOD = 0;
const bool GREYSON = true;
const float CASCADE_IPD = ((21.0/32.0)*3.14159)*(1.0/360);
const float SHOULDER_RATIO = 12.0/36.0;
const int CASCADE_MAX = 36;
const int CASCADE_POS_COUNT = CASCADE_MAX/3;
const int CASCADE_SHOULDER_DELAY = 1000;
const int CASCADE_LOWER_DELAY = 500;

const int PIDTune = 0;
const std::string AUTOS[] = {"left", "right", "skills"};

//Non Constant variables
int timer = 105000;
int cascadeSetpoint = 0;
bool stationaryish = false;
int auton = 0;
bool inputsEnabled = false;
int shoulderMax = 120;
int cascadePositions[36/3];


// Hot Motor Detection
std::string hotMotors() {
	if (DEBUG) {printf("hotMotors\n");}
	std::string hotMotors = "";
	for (int i = 0; i < 3; i++) {
		if (leftDt.is_over_temp(i) && !(leftDt.get_voltage(i) == PROS_ERR)) {
			hotMotors += "Left-Drivetrain " + std::to_string(i) + " ";
		}
		if (rightDt.is_over_temp(i) && !(rightDt.get_voltage(i) == PROS_ERR)) {
			hotMotors += "Right-Drivetrain " + std::to_string(i) + " ";
		}
	}
	if (cascade.is_over_temp(0) && !(cascade.get_voltage(0) == PROS_ERR)) {
		hotMotors += "Cascade 0";
	}
	if (cascade.is_over_temp(1) && !(cascade.get_voltage(1) == PROS_ERR)) {
		hotMotors += "Cascade 1";
	}
	if (banshee.is_over_temp() && !(banshee.get_voltage() == PROS_ERR)) {
		hotMotors += "Banshee ";
	}
	if (shoulder.is_over_temp() && !(shoulder.get_voltage() == PROS_ERR)) {
		hotMotors += "Shoulder ";
	}
	if (!(hotMotors == "")) {
	}

	return hotMotors;
}

void autoSelector(){
	if (DEBUG) {printf("autoSelector\n");}
	if (pros::lcd::read_buttons() == LCD_BTN_LEFT) {
		auton--;
		if (auton < 0) {
			auton = sizeof(AUTOS)-1;
		}
		while (pros::lcd::read_buttons() == LCD_BTN_LEFT) {
			pros::delay(50);
		}
	} else if (pros::lcd::read_buttons() == LCD_BTN_RIGHT) {
		auton++;
		if (auton > sizeof(AUTOS)-1) {
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
	leftDt.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	rightDt.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

	// Set the zero position of the cascade and shoulder motors to their current positions.
	cascade.set_zero_position(0);
	cascade.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	shoulder.set_zero_position(0);
	shoulder.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

	for (int i = 0; i < CASCADE_POS_COUNT; ++i) {
		cascadePositions[i] = static_cast<int>((i * 3.0) / CASCADE_IPD);
	}

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

void moveCascade(int change) {
	cascadeSetpoint += change;
	if (cascadeSetpoint >= CASCADE_POS_COUNT) {
		cascadeSetpoint = CASCADE_POS_COUNT - 1;
	} else if (cascadeSetpoint < 0) {
		cascadeSetpoint = 0;
	}
	// cascade.move_absolute(cascadePositions[cascadeSetpoint], 200);
}

// Autonomous routines
void leftAuton() {
	if (DEBUG) {printf("leftAuton\n");}
	leftDt.move_velocity(200);
	rightDt.move_velocity(200);
	pros::delay(500);
	leftDt.brake();
	rightDt.brake();	
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
	pros::Task condisplay(controlerDisplay);

	leftDt.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	rightDt.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

	enum class CascadeSequenceState {
		idle,
		waitingToRaiseShoulder,
		waitingToLowerCascade
	};
	CascadeSequenceState cascadeSequence = CascadeSequenceState::idle;
	std::uint32_t cascadeSequenceStarted = 0;

	while (true) {
		const std::uint32_t now = pros::millis();
		if (cascadeSequence == CascadeSequenceState::waitingToRaiseShoulder &&
			now - cascadeSequenceStarted >= CASCADE_SHOULDER_DELAY) {
			shoulder.move_absolute(180 / SHOULDER_RATIO, 200);
			cascadeSequenceStarted = now;
			cascadeSequence = CascadeSequenceState::waitingToLowerCascade;
		} else if (cascadeSequence == CascadeSequenceState::waitingToLowerCascade &&
			now - cascadeSequenceStarted >= CASCADE_LOWER_DELAY) {
			moveCascade(-2);
			cascadeSequence = CascadeSequenceState::idle;
		}

		imu.get_pitch();
		if (inputsEnabled) {
			if (!GREYSON) {
				chassis.arcade(
					master.get_analog(ANALOG_LEFT_Y)+THROTTLE_MOD,
					master.get_analog(ANALOG_RIGHT_X)
				);
			} else {
				chassis.tank(
					master.get_analog(ANALOG_LEFT_Y)+THROTTLE_MOD, 
					master.get_analog(ANALOG_RIGHT_Y)+THROTTLE_MOD
				);
				if (
					!pros::competition::is_connected() &&
					master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP) && 
					master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)
				) {
					runAuto();
				}
			}
			if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
				banshee.move_velocity(200);
				intake.move_velocity(200);
			} else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
				banshee.move_velocity(-200);
				intake.move_velocity(-200);
			} else {
				banshee.move_velocity(0);
				intake.move_velocity(0);
			}
		}
		if(MANUAL_SHOULDER && cascadeSequence == CascadeSequenceState::idle){
			if (inputsEnabled) {
				if (master.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
					shoulder.move_velocity(200);
				} else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
					shoulder.move_velocity(-200);
				} else {
					shoulder.move_velocity(0);
				}
			}
		}
		
		if (inputsEnabled) {
			if (true) {
				if (cascadeSequence == CascadeSequenceState::idle &&
					master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
					master.rumble(".");
					if (cascadeSetpoint == 0) {
						moveCascade(3);
						cascadeSequenceStarted = pros::millis();
						cascadeSequence = CascadeSequenceState::waitingToRaiseShoulder;
					} else {
						moveCascade(1);
					}
				} else if (cascadeSequence == CascadeSequenceState::idle &&
					master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
					master.rumble(".");
					moveCascade(-1);					
				}
				if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
					cascade.move_velocity(200);
				} else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
					cascade.move_velocity(-200);
				} else {
					cascade.move_velocity(0);
				}
			}
		}
		pros::delay(20);
	}
}

void autonomous() {
	if (DEBUG) {printf("autonomous\n");}
	if (PIDTune == 0){ runAuto();} 
	else if (PIDTune == 1){
		chassis.turnToHeading(90, 2000);
		chassis.turnToHeading(0, 2000);
		chassis.turnToHeading(180, 2000);
		chassis.turnToHeading(0, 2000);
		
	} else {
		chassis.moveToPoint(0, 24, 2000);
		chassis.moveToPoint(0, 0, 2000, {.forwards = false});
		chassis.moveToPoint(0, 12, 2000);
		chassis.moveToPoint(0, 0, 2000, {.forwards = false});
	}
}