#include "main.h"
#include "lib.h"
#include "calibration.h"

// PNEUMATICS, ADI
#define LOAD_PORT 'A'

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
PID linear = {
	0, // kP
	0, // kI
	0, // kD
	0.1, // settle error
	200, // settle time
	2000 // timeout
};

PID angular = {
	0, // kP
	0, // kI
	0, // kD
	0.1, // settle error
	200, // settle time
	2000 // timeout
};

// SUBSYSTEMS
pros::Motor intake(19);
pros::Motor stick(20);
pros::adi::DigitalOut loader(LOAD_PORT);

// SENSORS
pros::Rotation left_tracker_sensor(10);  // Rotation sensor on port 14
pros::Distance left_dist_sensor(7);      // Distance sensor on port 7
pros::Distance right_dist_sensor(6);     // Distance sensor on port 6
pros::Distance front_dist_sensor(8);     // Distance sensor on port 8
pros::Distance back_dist_sensor(9);      // Distance sensor on port 9

// DRIVEMOTORS
std::vector<std::int8_t> left_mg({-11, 12, -15});
std::vector<std::int8_t> right_mg({16, -17, 18});

// OWDrivetrain constructor with sensor offsets (in mm):
// Parameters: left_motors, right_motors, linear_pid, angular_pid, motor_speed, 
//             imu_port, imu2_port, tracker_port, track_width, tracker_offset,
//             left_dist_port, right_dist_port, front_dist_port, back_dist_port,
//             front_x_offset, front_y_offset, back_x_offset, back_y_offset,
//             left_x_offset, left_y_offset, right_x_offset, right_y_offset

float offsets[8] = { // this is for the distance sensor
	// Right and Forward are Positive
	-5.4f, // Front X
	4.5118f, // Front Y
	1.75f, // Back X
	-6.2441, // Back Y
	-5.6142f, // Left X 
	0.2f, // Left Y
	5.2992f, // Right X
	1.0f // Right Y
};

// First create drivetrain without UKF
OWDrivetrain drivetrain(left_mg, right_mg, linear, angular, 600, 13, 14, &left_tracker_sensor, 11.5, 6.7, 
                        &left_dist_sensor, &right_dist_sensor, &front_dist_sensor, &back_dist_sensor, nullptr,
                        offsets[0], offsets[1],    // Front sensor: 0 inches X, 5.91 inches forward from center
                        offsets[2], offsets[3],   // Back sensor: 0 inches X, 5.91 inches backward from center  
                        offsets[4], offsets[5],   // Left sensor: 5.91 inches left, 0 inches Y from center
                        offsets[6], offsets[7]);   // Right sensor: 5.91 inches right, 0 inches Y from center

void setup_ukf() {
    // Create sample UKF instance with tunable parameters
    State initial_state = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}; // x, y, heading, lv, av, imu_bias, encoder_bias
    State mean_uncertainty = {1.0f, 1.0f, 0.1f, 0.5f, 0.1f, 0.01f, 0.01f}; // position, heading, velocity, bias uncertainties
    State std_dev = {0.5f, 0.5f, 0.05f, 0.2f, 0.05f, 0.005f, 0.005f};
    
    // Process noise matrix (diagonal) - tune these values
    std::array<std::array<float, STATE_DIMENSIONS>, STATE_DIMENSIONS> process_noise = {};
    // BALANCED (recommended start)
	process_noise = {{
		{2.0f, 0, 0, 0, 0, 0, 0},
		{0, 2.0f, 0, 0, 0, 0, 0},
		{0, 0, 0.01f, 0, 0, 0, 0},
		{0, 0, 0, 4.0f, 0, 0, 0},
		{0, 0, 0, 0, 0.05f, 0, 0},
		{0, 0, 0, 0, 0, 4.0f, 0},
		{0, 0, 0, 0, 0, 0, 4.0f}
	}};
    
    // Diagonal matrix - sensor variances (std_dev²)
	std::array<std::array<float, MEASUREMENT_DIMENSIONS>, MEASUREMENT_DIMENSIONS> measurement_noise = {{
		{64.0f,  0.0f,   0.0f,   0.0f,   0.0f,    0.0f},  // Front dist (8"²)
		{0.0f,   64.0f,  0.0f,   0.0f,   0.0f,    0.0f},  // Back dist (8"²)
		{0.0f,   0.0f,   64.0f,  0.0f,   0.0f,    0.0f},  // Left dist (8"²)
		{0.0f,   0.0f,   0.0f,   64.0f,  0.0f,    0.0f},  // Right dist (8"²)
		{0.0f,   0.0f,   0.0f,   0.0f,   0.001f,  0.0f},  // IMU heading (rad²)
		{0.0f,   0.0f,   0.0f,   0.0f,   0.0f,    4.0f}   // Encoder/IMU2 (in²)
	}};

    
    // Initial state covariance
    std::array<std::array<float, STATE_DIMENSIONS>, STATE_DIMENSIONS> initial_cov = {{
		{1.0f,  0.0f,  0.0f,   0.0f, 0.0f, 0.0f, 0.0f},  // x
		{0.0f,  1.0f,  0.0f,   0.0f, 0.0f, 0.0f, 0.0f},  // y
		{0.0f,  0.0f,  0.0025f, 0.0f, 0.0f, 0.0f, 0.0f}, // heading (0.05²)
		{0.0f,  0.0f,  0.0f,   4.0f, 0.0f, 0.0f, 0.0f},  // velocity
		{0.0f,  0.0f,  0.0f,   0.0f, 0.01f, 0.0f, 0.0f}, // angular vel
		{0.0f,  0.0f,  0.0f,   0.0f, 0.0f, 4.0f, 0.0f},  // left wheel
		{0.0f,  0.0f,  0.0f,   0.0f, 0.0f, 0.0f, 4.0f}   // right wheel
	}};
    
    // UKF tuning parameters
    float alpha = 0.001f;  // Spread of sigma points (smaller = more conservative)
    float beta = 2.0f;     // Prior knowledge of distribution (2 = optimal for Gaussian)  
    float kappa = 0.0f;    // Secondary scaling parameter
    
    // Create and assign UKF instance to drivetrain
    drivetrain.ukf = new UKF(drivetrain, initial_state, mean_uncertainty, std_dev, 
                            process_noise, measurement_noise, alpha, beta, kappa, initial_cov);
}

void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	while (drivetrain.imu.is_calibrating() || drivetrain.imu2.is_calibrating()) {
		pros::delay(10);
	}

	// Initialize UKF with tuned parameters
	setup_ukf();
	
	// Start odometry task to update position tracking
	drivetrain.start_odom(); // Temporarily commented to isolate crash

	pros::lcd::register_btn1_cb(on_center_button);
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
void competition_initialize() {}

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
	calibrateSensorsForUKF();
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
	pros::Controller master(pros::E_CONTROLLER_MASTER);

	bool loader_bool = false;

	while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);  // Prints status of the emulated screen LCDs
		
		// Subsytems
		if (master.get_digital(DIGITAL_L1)) stick.move(127);
		if (master.get_digital(DIGITAL_L2)) stick.move(-127);

		if (!master.get_digital(DIGITAL_L1) && !master.get_digital(DIGITAL_L2)) stick.move(0);
		
		if (master.get_digital(DIGITAL_R1)) intake.move(127);
		if (master.get_digital(DIGITAL_R2)) intake.move(-127);
		if (!master.get_digital(DIGITAL_R1) && !master.get_digital(DIGITAL_R2)) intake.move(0);
		
		// Pneumatics
		if (master.get_digital_new_press(DIGITAL_A)) loader.set_value(!loader_bool);

		// Arcade control scheme
		int dir = master.get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
		int turn = master.get_analog(ANALOG_RIGHT_X);  // Gets the turn left/right from right joystick
		
		// Scale joystick values to proper voltage range (±12000mV)
		int left_voltage = (dir + turn) * 94;  // 12000/127 ≈ 94
		int right_voltage = (dir - turn) * 94;
		drivetrain.move(left_voltage, right_voltage);
		drivetrain.test_lidar_display();
		
		pros::delay(20);                               // Run for 20 ms then update
	}
}