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
 pros::ADIPotentiometer pot(2);
 auto driveA = ChassisControllerFactory::create(
   {11,9},
   {-12,-10},
   AbstractMotor::gearset::green,
   {4.125_in,12.125_in}
 );
 AsyncMotionProfileController profileController = AsyncControllerFactory::motionProfile(
   1,  // Maximum linear velocity of the Chassis in m/s
   2.0,  // Maximum linear acceleration of the Chassis in m/s/s
   2.5, // Maximum linear jerk of the Chassis in m/s/s/s
  driveA // Chassis Controller
 );



 void cataTaskA(void*param){
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
  while(sensorA.get_value() > 1700){
    cataA.moveVelocity(200);

  }
  cataA.setBrakeMode(AbstractMotor::brakeMode::hold);
  cataA.moveVelocity(0);

}

void fireCata(){
  cataA.moveVelocity(200);
  pros::delay(1000);
  cataA.setBrakeMode(AbstractMotor::brakeMode::coast);
  cataA.moveVelocity(0);

}
QLength sideCapDistance = 4_ft;
QLength frontFlagDistance = 4_ft;
QLength frontCapDistance = 4.0_ft;
QLength midFlagDistance = 1.0_ft;
QLength platformAlignDistance = -4.0_ft;
QLength alliancePlatformDistance = 4.0_ft;
QLength centerPlatformDistance = 6.0_ft;

bool blue = false; //If red -> false, If blue -> true
bool front = true;
bool skills = false;
void autonomous() {
pros::delay(100);

if( pot.get_value() > 10 && pot.get_value() < 827){
  blue = false;
  front = true;
  skills = false;

}
if( pot.get_value() > 828 && pot.get_value() < 1644){
  blue = true;
  front = true;
  skills = false;
}
if( pot.get_value() > 1645 && pot.get_value() < 2461){
  blue = false;
  front = false;
  skills = false;

}
if( pot.get_value() > 2462 && pot.get_value() < 3278){
  blue = true;
  front = false;
  skills = false;

}
if( pot.get_value() > 3279 && pot.get_value() < 4096){
  blue = false;
  front = false;
  skills = true;
}

  pros::lcd::print(2, "Blue?:%d",blue);
    pros::lcd::print(3, "Front?:%d ",front);
      pros::lcd::print(4, "Skills?:%d", skills);

  if(front && !skills){

  profileController.generatePath({
  Point{0_ft, 0_ft, 0_deg},  // Profile starting position, this will normally be (0, 0, 0)
  Point{sideCapDistance, 0_ft, 0_deg}}, // The next point in the profile, 3 feet forward
  "Ball" // Profile name
);
profileController.setTarget("Ball");
intakeA.setTarget(200);
loadCata();


profileController.waitUntilSettled();
profileController.setTarget("Ball", true);

profileController.waitUntilSettled();
profileController.removePath("Ball");
intakeA.setTarget(0);
driveA.setMaxVelocity(50);
if(blue){driveA.turnAngle(90_deg);}
else{driveA.turnAngle(-90_deg);}
driveA.waitUntilSettled();
fireCata();
if(blue){driveA.turnAngle(-90_deg);}
else{driveA.turnAngle(90_deg);}
driveA.waitUntilSettled();
driveA.setMaxVelocity(50);
driveA.moveDistance(-0.75_ft);
driveA.waitUntilSettled();
pros::delay(500);
driveA.moveDistance(0.5_ft);
driveA.waitUntilSettled();
driveA.setMaxVelocity(50);


if(blue){driveA.turnAngle(90_deg);}
else{driveA.turnAngle(-90_deg);}
driveA.waitUntilSettled();
driveA.setMaxVelocity(200);
//TODO: ADJUST FOR COLORS USING IF LOOP//TODO:TEST THIS.
if(blue){profileController.generatePath({
Point{0_ft, 0_ft, -90_deg},  // Profile starting position, this will normally be (0, 0, 0)
Point{0_ft, frontFlagDistance, 90_deg}}, // The next point in the profile, 3 feet forward
"BottomFlag" // Profile name
);}
else{
profileController.generatePath({
Point{0_ft, 0_ft, -90_deg},  // Profile starting position, this will normally be (0, 0, 0)
Point{0_ft, frontFlagDistance, -90_deg}}, // The next point in the profile, 3 feet forward
"BottomFlag" // Profile name
);
}
profileController.setTarget("BottomFlag");
profileController.waitUntilSettled();
profileController.setTarget("BottomFlag", true);
intakeA.setTarget(0);
profileController.waitUntilSettled();
profileController.removePath("BottomFlag");
if(blue){driveA.turnAngle(-45_deg);}
else{driveA.turnAngle(45_deg);}
intakeA.setTarget(-200);
driveA.waitUntilSettled();
driveA.moveDistance(8_ft); //revert to 3_ft if it doesnt work.
driveA.waitUntilSettled();
intakeA.setTarget(0);

/*
if(blue){driveA.turnAngle(35_deg);}
else{driveA.turnAngle(-35_deg);}
driveA.moveDistance(3_ft);
driveA.waitUntilSettled();
*/


/*
//Get Ball Under Cap
  intakeA.setTarget(200);
  driveA.moveDistance(sideCapDistance);
  driveA.waitUntilSettled();
  loadCata();

  pros::delay(1000);
  intakeA.setTarget(0);
  driveA.moveDistance(-sideCapDistance);
  driveA.waitUntilSettled();
//Turn to shoot
  if(blue){driveA.turnAngle(90_deg);}
  else{driveA.turnAngle(-90_deg);}
  driveA.waitUntilSettled();
  fireCata();
//Hit Mid Flag
  driveA.moveDistance(frontFlagDistance);
  driveA.waitUntilSettled();
  driveA.moveDistance(-frontFlagDistance);
  driveA.waitUntilSettled();
//Flip Cap Over
  if(blue){driveA.turnAngle(-30_deg);}
  else{driveA.turnAngle(30_deg);}
  intakeA.setTarget(-120);
  driveA.moveDistance(frontCapDistance);
  driveA.waitUntilSettled();
  if(blue){driveA.turnAngle(-10_deg);}
  else{driveA.turnAngle(10_deg);}

  driveA.moveDistance(-frontCapDistance);
  driveA.waitUntilSettled();
  if(blue){driveA.turnAngle(30_deg);}
  else{driveA.turnAngle(-30_deg);}

  //Return to Park
  driveA.moveDistance(platformAlignDistance);
  driveA.waitUntilSettled();
  if(blue){driveA.turnAngle(-90_deg);}
  else{driveA.turnAngle(90_deg);}
  driveA.moveDistance(alliancePlatformDistance);
  driveA.waitUntilSettled();
*/
}
if(!front && !skills){
  profileController.generatePath({
  Point{0_ft, 0_ft, 0_deg},  // Profile starting position, this will normally be (0, 0, 0)
  Point{sideCapDistance, 0_ft, 0_deg}}, // The next point in the profile, 3 feet forward
  "Ball" // Profile name
  );
  profileController.setTarget("Ball");
  intakeA.setTarget(200);
  //loadCata();


  profileController.waitUntilSettled();
  profileController.setTarget("Ball", true);

  profileController.waitUntilSettled();
  profileController.removePath("Ball");
  driveA.setMaxVelocity(150);
  if(blue){driveA.turnAngle(90_deg);}
  else{driveA.turnAngle(-90_deg);}
  driveA.waitUntilSettled();
  driveA.moveDistance(2_ft);
  driveA.waitUntilSettled();
  if(blue){driveA.turnAngle(-90_deg);}
  else{driveA.turnAngle(90_deg);}
  driveA.moveDistance(4_ft);
  driveA.waitUntilSettled();

}
if (skills){
  sideCapDistance = 6.25_ft;



  profileController.generatePath({
  Point{0_ft, 0_ft, 0_deg},  // Profile starting position, this will normally be (0, 0, 0)
  Point{sideCapDistance, 0_ft, 0_deg}}, // The next point in the profile, 3 feet forward
  "Ball" // Profile name
);
profileController.setTarget("Ball");
intakeA.setTarget(200);
loadCata();


profileController.waitUntilSettled();
profileController.setTarget("Ball", true);

profileController.waitUntilSettled();
profileController.removePath("Ball");
intakeA.setTarget(0);
driveA.setMaxVelocity(150);
if(blue){driveA.turnAngle(90_deg);}
else{driveA.turnAngle(-90_deg);}
driveA.waitUntilSettled();
fireCata();
if(blue){driveA.turnAngle(-90_deg);}
else{driveA.turnAngle(90_deg);}
driveA.waitUntilSettled();
driveA.moveDistanceAsync(-0.25_ft);
driveA.waitUntilSettled();
driveA.moveDistanceAsync(0.25_ft);
driveA.waitUntilSettled();

if(blue){driveA.turnAngle(90_deg);}
else{driveA.turnAngle(-90_deg);}
driveA.waitUntilSettled();
driveA.setMaxVelocity(200);
//TODO: ADJUST FOR COLORS USING IF LOOP//TODO:TEST THIS.
if(blue){profileController.generatePath({
Point{0_ft, 0_ft, -90_deg},  // Profile starting position, this will normally be (0, 0, 0)
Point{0_ft, frontFlagDistance, 90_deg}}, // The next point in the profile, 3 feet forward
"BottomFlag" // Profile name
);}
else{
profileController.generatePath({
Point{0_ft, 0_ft, -90_deg},  // Profile starting position, this will normally be (0, 0, 0)
Point{0_ft, frontFlagDistance, -90_deg}}, // The next point in the profile, 3 feet forward
"BottomFlag" // Profile name
);
}
profileController.setTarget("BottomFlag");
profileController.waitUntilSettled();
profileController.setTarget("BottomFlag", true);
intakeA.setTarget(0);
profileController.waitUntilSettled();
profileController.removePath("BottomFlag");
if(blue){driveA.turnAngle(-45_deg);}
else{driveA.turnAngle(45_deg);}
intakeA.setTarget(-200);
driveA.waitUntilSettled();
driveA.moveDistance(6_ft); //revert to 3_ft if it doesnt work.
driveA.waitUntilSettled();
intakeA.setTarget(0);


//Skills Changes

driveA.moveDistance(-6_ft); //revert to 3_ft if it doesnt work.
driveA.waitUntilSettled();
if(blue){driveA.turnAngle(45_deg);}
else{driveA.turnAngle(-45_deg);}
driveA.moveDistance(-2_ft);
driveA.waitUntilSettled();
if(blue){driveA.turnAngle(-90_deg);}
else{driveA.turnAngle(90_deg);}
driveA.waitUntilSettled();
driveA.moveDistance(8_ft);
driveA.waitUntilSettled();

}






}
