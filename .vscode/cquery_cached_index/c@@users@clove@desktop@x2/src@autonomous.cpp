#include "main.h"

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
 auto intakeA = AsyncControllerFactory::velIntegrated(-20);
auto descorerMA = AsyncControllerFactory::velIntegrated(-19);

 okapi::MotorGroup cataA({7,-8});
 pros::ADILineSensor sensorA(1);
 
 auto driveA = ChassisControllerFactory::create(
   {11,9},
   {-12,-10},
   AbstractMotor::gearset::green,
   {4.125_in,11.5_in}
 );
auto drivePID = ChassisControllerFactory::create(
  {11, 9}, // Left motors
  {-12, -10},   // Right motors
  {kp, ki, kd}, // distance controller
  {kp, ki, kd}, // angle controller (helps you drive straight)
  {kp, ki, kd}, // turn controller
  AbstractMotor::gearset::green, // Torque gearset
  {4.125_in, 11.5_in} // 4 inch wheels, 12.5 inch wheelbase width
);


 void cataTask(void*param){
if((bool*)param){

  while(sensorA.get_value() > 1700){
    cataA.moveVelocity(100);

  }
  cataA.setBrakeMode(AbstractMotor::brakeMode::hold);
  cataA.moveVelocity(0);

  }
  else{
    cataA.moveVelocity(100);
    pros::delay(1000);
    cataA.setBrakeMode(AbstractMotor::brakeMode::coast);
    cataA.moveVelocity(0);
  }
}
void loadCata(){


}

void fireCata(){


}
QLength sideCapDistance = 3.5_ft;
QLength frontFlagDistance = 4.125_ft;
QLength frontCapDistance = 4.0_ft;
QLength midFlagDistance = 1.0_ft;
bool blue = false; //If red -> false, If blue -> true
void autonomous() {

  driveA.moveDistanceAsync(sideCapDistance);
  loadCata();
  driveA.waitUntilSettled();
  intakeA.setTarget(200);
  pros::delay(1000);
  intakeA.setTarget(0);
  driveA.moveDistance(-sideCapDistance);
  driveA.waitUntilSettled();
  if(blue){driveA.turnAngle(90_deg);}
  else{driveA.turnAngle(-90_deg);}
  fireCata();
  driveA.moveDistance(frontFlagDistance);
  driveA.waitUntilSettled();
  driveA.moveDistance(-frontFlagDistance);
  driveA.waitUntilSettled();
  if(blue){driveA.turnAngle(-30_deg);}
  else{driveA.turnAngle(30_deg);}
  intakeA.setTarget(-120);
  driveA.moveDistance(frontCapDistance);
  driveA.waitUntilSettled();
  if(blue){driveA.turnAngle(-10_deg);}
  else{driveA.turnAngle(10_deg);}
  driveA.moveDistance(-frontCapDistance);
  driveA.waitUntilSettled();







}
