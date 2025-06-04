#include "vex.h"
#include <cstdlib>
#include <cmath>
#include <stdbool.h>

#define FMT_HEADER_ONLY
using namespace vex;

brain Brain;
competition Competition;
controller Controller1 = controller(primary);
digital_out Pneumatic1(Brain.ThreeWirePort.A);

motor driveMotorRightOne = motor(PORT16, ratio18_1, false);   
motor driveMotorLeftOne = motor(PORT1, ratio18_1, true);   
motor driveMotorRightTwo = motor(PORT15, ratio18_1, false);
motor driveMotorLeftTwo = motor(PORT3, ratio18_1, true); 
motor driveMotorRightThree = motor(PORT14, ratio18_1, false);    
motor driveMotorLeftThree = motor(PORT2, ratio18_1, true);

motor_group RightDrivetrain = motor_group(driveMotorRightOne, driveMotorRightTwo, driveMotorRightThree);
motor_group LeftDrivetrain  = motor_group(driveMotorLeftOne, driveMotorLeftTwo, driveMotorLeftThree);
motor_group FullDrivetrain = motor_group(driveMotorRightOne, driveMotorRightTwo, driveMotorRightThree, driveMotorLeftOne, driveMotorLeftTwo, driveMotorLeftThree);
bool DrivetrainNeedsToBeStopped_Controller1 = true;
int rc_auto_loop_function_Controller1()
{
  
  while (true)
  {
    
    int drivetrainLeftSideSpeed = Controller1.Axis3.position() + Controller1.Axis4.position()/2;
    int drivetrainRightSideSpeed = Controller1.Axis3.position() - Controller1.Axis4.position()/2;

    if (abs(drivetrainLeftSideSpeed) < 5 && abs(drivetrainRightSideSpeed) < 5 && DrivetrainNeedsToBeStopped_Controller1) DrivetrainNeedsToBeStopped_Controller1 = false;
    else DrivetrainNeedsToBeStopped_Controller1 = true;

    double y = Controller1.Axis3.position();
    double x = Controller1.Axis4.position();
    double a = Controller1.Axis1.position();

    if (y >= -5 && y <= 5) y = 0;
    if (x >= -5 && x <= 5) x = 0;
    if (a >= -5 && a <= 5) a = 0;

    RightDrivetrain.spin(forward, y - a, percent);
    LeftDrivetrain.spin(forward, y + a, percent);
  }
  return 0;
}
#pragma endregion VEXcode Generated Robot Configuration

void drivercontrol(void)
{
  
  task rc_auto_loop_task_Controller1(rc_auto_loop_function_Controller1);
}