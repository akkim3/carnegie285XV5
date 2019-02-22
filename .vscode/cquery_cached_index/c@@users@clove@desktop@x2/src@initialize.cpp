#include "main.h"



/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

//	pros::lcd::initialize();
void initialize() {
	pros::lcd::initialize();

	pros::lcd::print(0, "Buttons Bitmap: %d\n", pros::lcd::read_buttons());
	 pros::delay(20);





}

//	pros::lcd::set_text(1, "Hello PROS User!");

//	pros::lcd::register_btn1_cb(on_center_button);


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
