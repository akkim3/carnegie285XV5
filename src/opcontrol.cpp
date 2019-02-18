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
 ControllerButton cataFire (ControllerDigital::R1);
ControllerButton descorer (ControllerDigital::R2);
 ControllerButton intakeFwd (ControllerDigital::L1);
 ControllerButton intakeRev (ControllerDigital::L2);
 bool cataToggle {false};




 auto intake = AsyncControllerFactory::velIntegrated(-20);
auto descorerM = AsyncControllerFactory::velIntegrated(-19);

 okapi::MotorGroup cata({7,-8});
 pros::ADILineSensor sensor(1);
 auto drive = ChassisControllerFactory::create(
   {11,9},
   {-12,-10},
   AbstractMotor::gearset::green,
   {4.125_in,11.5_in}
 );

 void cataTask(void*param){
if((bool*)param){

  while(sensor.get_value() > 1700){
    cata.moveVelocity(100);

  }
  cata.setBrakeMode(AbstractMotor::brakeMode::hold);
  cata.moveVelocity(0);

  }
  else{
    cata.moveVelocity(100);
    pros::delay(1000);
    cata.setBrakeMode(AbstractMotor::brakeMode::coast);
    cata.moveVelocity(0);
  }
}


void opcontrol() {

	while (true) {
    drive.arcade(controller.getAnalog(ControllerAnalog::leftY), controller.getAnalog(ControllerAnalog::rightX));
    if(cataFire.changedToPressed()){
      cataToggle = !cataToggle;
      pros::Task my_Task (cataTask, (void*)cataToggle,TASK_PRIORITY_DEFAULT,TASK_STACK_DEPTH_DEFAULT,"My Task");
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
          descorerM.setTarget(200);
          }
else if(intakeRev.isPressed() && !intakeFwd.isPressed() && descorer.isPressed()){
          descorerM.setTarget(-200);
          }
    else{
        intake.setTarget(0);
      }

    }

		pros::delay(20);
	}
