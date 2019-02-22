#include "main.h"

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
 Controller controller;
 ControllerButton brakeToggleButton (ControllerDigital::B);
ControllerButton descorerMacro (ControllerDigital::right);
ControllerButton pidUp (ControllerDigital::X);
ControllerButton pidDown (ControllerDigital::Y);
ControllerButton pidDUp (ControllerDigital::up);
ControllerButton pidDDown (ControllerDigital::down);
 ControllerButton cataFire (ControllerDigital::R1);
ControllerButton descorer (ControllerDigital::R2);
 ControllerButton intakeFwd (ControllerDigital::L1);
 ControllerButton intakeRev (ControllerDigital::L2);
 bool cataToggle {false};




 auto intake = AsyncControllerFactory::velIntegrated(-20);
//auto descorerM = AsyncControllerFactory::velIntegrated(-1);

 okapi::MotorGroup cata({7,-8});
 okapi::Motor descorerMotor {-1};
 pros::ADILineSensor sensor(1);
 auto drive = ChassisControllerFactory::create(
   {11,9},
   {-12,-10},
   AbstractMotor::gearset::green,
   {4.125_in, 12_in}
 );

 void cataTask(void*param){
if((bool*)param){

  while(sensor.get_value() > 1700){
    cata.moveVelocity(200);

  }
  cata.setBrakeMode(AbstractMotor::brakeMode::hold);
  cata.moveVelocity(0);

  }
  else{
    cata.moveVelocity(200);
    pros::delay(1000);
    cata.setBrakeMode(AbstractMotor::brakeMode::coast);
    cata.moveVelocity(0);

  }

pros::delay(20);
}

float kd = 0.0001;
float kp = 0.00005; // was working at 0.005 -> dead on the target


auto driveTest = ChassisControllerFactory::create(
  {11, 9}, // Left motors
  {-12, -10},   // Right motors
  {0.0065, 0, 0.0001}, // distance controller
  {kp, 0, kd}, // angle controller (helps you drive straight)
  {kp, 0.00001, kd}, // turn controller
  AbstractMotor::gearset::green, // Torque gearset
  {4.125_in, 11.5_in} // 4 inch wheels, 11.5 inch wheelbase width
);
pros::ADIPotentiometer potA(2);
lv_obj_t* label;
auto string = "";
static lv_style_t style_txt;
void opcontrol() {
/*  label = lv_label_create(lv_scr_act(), NULL);
  lv_label_set_recolor(label, true);
  */

	while (true) {
/*
    lv_style_t style;
    lv_style_copy(&style,&lv_style_plain);

lv_style_copy(&style_txt, &lv_style_plain);
style_txt.text.letter_space = 10;

style_txt.text.line_space = 1;

    style.line.width = 8;
    *
    if( potA.get_value() > 10 && potA.get_value() < 827){
      string="Front Red";
      style_txt.text.color = LV_COLOR_RED;

    }
    if( potA.get_value() > 828 && potA.get_value() < 1644){
      string="Front Blue";
      style_txt.text.color = LV_COLOR_BLUE;
    }
    if( potA.get_value() > 1645 && potA.get_value() < 2461){
      string="Back Red";
      style_txt.text.color = LV_COLOR_RED;

    }
    if( potA.get_value() > 2462 && potA.get_value() < 3278){
      string="Back Blue";
      style_txt.text.color = LV_COLOR_BLUE;

    }
    if( potA.get_value() > 3279 && potA.get_value() < 4096){
      string="Skills";

      style.line.color = LV_COLOR_CYAN;
    }*/

		drive.arcade(controller.getAnalog(ControllerAnalog::leftY), controller.getAnalog(ControllerAnalog::rightX));
		if(cataFire.changedToPressed()){
			cataToggle = !cataToggle;
			pros::Task my_Task (cataTask, (void*)cataToggle,TASK_PRIORITY_DEFAULT,TASK_STACK_DEPTH_DEFAULT,"My Task");

		}
	if(descorerMacro.changedToPressed()){
    drive.turnAngle(90_deg);
}
  if(pidUp.changedToPressed()){
    drive.setMaxVelocity(100);
    drive.turnAngle(90_deg);
    drive.waitUntilSettled();
    drive.setMaxVelocity(200);
  }
  if(pidDUp.changedToPressed()){
    drive.setMaxVelocity(100);
    drive.turnAngle(80_deg);
    drive.waitUntilSettled();
    drive.setMaxVelocity(200);
  }
		if(intakeFwd.isPressed() && !intakeRev.isPressed() && !descorer.isPressed()){
			intake.setTarget(200);
			}
		else if(intakeRev.isPressed() && !intakeFwd.isPressed() && !descorer.isPressed()){
				intake.setTarget(-200);
				}
		else if(intakeRev.isPressed() && intakeFwd.isPressed() && !descorer.isPressed()){
					intake.setTarget(-120);
					}
	else if(!intakeRev.isPressed() && !intakeFwd.isPressed() && descorer.isPressed()){
					descorerMotor.moveVelocity(100);
					}
	else if(intakeRev.isPressed() && !intakeFwd.isPressed() && descorer.isPressed()){
					descorerMotor.moveVelocity(-100);
					}
		else{
				intake.setTarget(0);
        descorerMotor.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
        descorerMotor.moveVelocity(0);
			}

		}

		pros::delay(20);
}
