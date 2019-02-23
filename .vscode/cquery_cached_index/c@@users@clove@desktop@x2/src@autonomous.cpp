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
QLength sideCapDistance = 3.5_ft;
QLength frontFlagDistance = 4.5_ft;
QLength frontCapDistance = 4.0_ft;
QLength midFlagDistance = 1.0_ft;
QLength platformAlignDistance = -4.0_ft;
QLength alliancePlatformDistance = 4.0_ft;
QLength centerPlatformDistance = 6.0_ft;

bool blue = false; //If red -> false, If blue -> true
bool front = false;
bool skills = false;
void autonomous() {
pros::delay(100);
/*
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
}*/

  pros::lcd::print(2, "Blue?:%d",blue);
    pros::lcd::print(3, "Front?:%d ",front);
      pros::lcd::print(4, "Skills?:%d", skills);

  if(front && !skills){

  profileController.generatePath({
  Point{0_ft, 0_ft, 0_deg},  //   (0, 0, 0)
  Point{sideCapDistance, 0_ft, 0_deg}}, //
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

driveA.setMaxVelocity(200);
//TODO: ADJUST FOR COLORS USING IF LOOP//TODO:TEST THIS.
if(blue){profileController.generatePath({
Point{0_ft, 0_ft, -90_deg},  //   (0, 0, 0)
Point{-0.1_ft, frontFlagDistance, 90_deg}}, //
"BottomFlag" // Profile name
);}
else{
profileController.generatePath({
Point{0_ft, 0_ft, -90_deg},  //   (0, 0, 0)
Point{-0.1_ft, frontFlagDistance, -90_deg}}, //
"BottomFlag" // Profile name
);
}
profileController.setTarget("BottomFlag");
profileController.waitUntilSettled();

profileController.setTarget("BottomFlag", true);
intakeA.setTarget(0);
profileController.waitUntilSettled();
profileController.removePath("BottomFlag");
driveA.setMaxVelocity(50);
driveA.moveDistance(-1.6_ft);
driveA.waitUntilSettled();
driveA.setMaxVelocity(50);
if(blue){driveA.turnAngle(-95_deg);}
else{driveA.turnAngle(95_deg);}
driveA.waitUntilSettled();
driveA.setMaxVelocity(150);
driveA.moveDistance(10_ft);
driveA.waitUntilSettled();
}

//B
//A
//C
//K
//Auto




if(!front && !skills){
  profileController.generatePath({
  Point{0_ft, 0_ft, 0_deg},  //   (0, 0, 0)
  Point{sideCapDistance, 0_ft, 0_deg}}, //
  "Ball" // Profile name
  );
  profileController.setTarget("Ball");
  intakeA.setTarget(100);
  pros::delay(500);
  intakeA.setTarget(0);
  pros::delay(500);
  intakeA.setTarget(200);
  profileController.waitUntilSettled();
  profileController.setTarget("Ball", true);

  profileController.waitUntilSettled();
  profileController.removePath("Ball");
  driveA.setMaxVelocity(50);
  if(blue){driveA.turnAngle(90_deg);}
  else{driveA.turnAngle(-90_deg);}
  driveA.waitUntilSettled();
  driveA.moveDistance(2_ft);
  driveA.waitUntilSettled();
  driveA.setMaxVelocity(50);
  if(blue){driveA.turnAngle(-90_deg);}
  else{driveA.turnAngle(90_deg);}
  driveA.setMaxVelocity(150);
  driveA.moveDistance(5_ft);
  driveA.waitUntilSettled();
}
//Skills - Highly Experimental Hail Mary autonomous
//Starts from the back tile
//Shoots preload into the flags
//Goes to mid and get's the flag





if (skills){

    profileController.generatePath({
    Point{0_ft, 0_ft, 0_deg},  //   (0, 0, 0)
    Point{sideCapDistance, 0_ft, 0_deg}}, //
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
  driveA.setMaxVelocity(200);
  if(blue){profileController.generatePath({
  Point{0_ft, 0_ft, -90_deg},  //   (0, 0, 0)
  Point{0_ft, 4_ft, 90_deg}}, //
  "MoveStart" // Profile name
  );}
  else{
  profileController.generatePath({
  Point{0_ft, 0_ft, -90_deg},  //   (0, 0, 0)
  Point{0_ft, 4_ft, -90_deg}}, //
  "MoveStart" // Profile name
  );
  }
  profileController.setTarget("MoveStart");
  profileController.waitUntilSettled();
  profileController.removePath("MoveStart");
  fireCata();


  //TODO: ADJUST FOR COLORS USING IF LOOP//TODO:TEST THIS.
  if(blue){profileController.generatePath({
  Point{0_ft, 0_ft, -90_deg},  //   (0, 0, 0)
  Point{-0.1_ft, frontFlagDistance, 90_deg}}, //
  "BottomFlag" // Profile name
  );}
  else{
  profileController.generatePath({
  Point{0_ft, 0_ft, -90_deg},  //   (0, 0, 0)
  Point{-0.1_ft, frontFlagDistance, -90_deg}}, //
  "BottomFlag" // Profile name
  );
  }
  profileController.setTarget("BottomFlag");
  profileController.waitUntilSettled();

  profileController.setTarget("BottomFlag", true);
  intakeA.setTarget(0);
  profileController.waitUntilSettled();
  profileController.removePath("BottomFlag");
  driveA.setMaxVelocity(50);
  if(blue){driveA.turnAngle(-90_deg);}
  else{driveA.turnAngle(90_deg);}
  driveA.waitUntilSettled();
  driveA.setMaxVelocity(200);
  profileController.generatePath({
  Point{0_ft, 0_ft, 0_deg},  //   (0, 0, 0)
  Point{sideCapDistance, 0_ft, 0_deg}}, //
  "Ball" // Profile name
);
profileController.setTarget("Ball");

loadCata();


profileController.waitUntilSettled();
intakeA.setTarget(200);
pros::delay(1000);
intakeA.setTarget(0);
driveA.setMaxVelocity(50);
if(blue){driveA.turnAngle(90_deg);}
else{driveA.turnAngle(-90_deg);}

if(blue){profileController.generatePath({
Point{0_ft, 0_ft, -90_deg},  //   (0, 0, 0)
Point{0_ft, 0.5_ft, 90_deg}}, //
"ShootMidFlag" // Profile name
);}
else{
profileController.generatePath({
Point{0_ft, 0_ft, -90_deg},  //   (0, 0, 0)
Point{0_ft, 0.5_ft, -90_deg}}, //
"ShootMidFlag" // Profile name
);
}
profileController.setTarget("ShootMidFlag");
profileController.waitUntilSettled();
fireCata();
profileController.setTarget("ShootMidFlag", true);
intakeA.setTarget(0);
profileController.waitUntilSettled();
profileController.removePath("ShootMidFlag");
if(blue){profileController.generatePath({
Point{0_ft, 0_ft, -90_deg},  //   (0, 0, 0)
Point{-0.1_ft, frontFlagDistance, 90_deg}}, //
"BottomFlag" // Profile name
);}
else{
profileController.generatePath({
Point{0_ft, 0_ft, -90_deg},  //   (0, 0, 0)
Point{-0.1_ft, frontFlagDistance, -90_deg}}, //
"BottomFlag" // Profile name
);
}
profileController.setTarget("BottomFlag");
profileController.waitUntilSettled();

profileController.setTarget("BottomFlag", true);
intakeA.setTarget(0);
profileController.waitUntilSettled();
profileController.removePath("BottomFlag");
driveA.setMaxVelocity(50);
if(blue){driveA.turnAngle(-90_deg);}
else{driveA.turnAngle(90_deg);}
driveA.waitUntilSettled();
driveA.setMaxVelocity(200);
profileController.setTarget("Ball", true);
driveA.setMaxVelocity(50);
if(blue){driveA.turnAngle(90_deg);}
else{driveA.turnAngle(-90_deg);}
driveA.waitUntilSettled();
  driveA.setMaxVelocity(50);
  driveA.moveDistance(-1.6_ft);
  driveA.waitUntilSettled();
  driveA.setMaxVelocity(50);
  if(blue){driveA.turnAngle(-95_deg);}
  else{driveA.turnAngle(95_deg);}
  driveA.waitUntilSettled();
  driveA.setMaxVelocity(150);
  driveA.moveDistance(10_ft);
  driveA.waitUntilSettled();

}






}
