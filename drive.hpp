#include "vex.h"
#include <cstdlib>
#include <cmath>
#include <stdbool.h>
#include <math.h>
#include <buttons.hpp>
#include <thread>
#define FMT_HEADER_ONLY
using namespace vex;

brain Brain;
competition Competition;
controller Controller1 = controller(primary);
gps GPS17 = gps(PORT17,-100.00, +30.00, mm, +90);
motor driveMotorRightOne = motor(PORT17, ratio18_1, true); 
inertial INS  = inertial(PORT12);
motor driveMotorLeftOne = motor(PORT1, ratio18_1, false);  
motor in1 = motor(PORT11, ratio18_1, false);  
motor in2 = motor(PORT12, ratio18_1, false); 
motor in3 = motor(PORT13, ratio18_1, false); 
motor in4 = motor(PORT13, ratio18_1, false); 
motor driveMotorRightTwo = motor(PORT13, ratio18_1, true);
motor driveMotorLeftTwo = motor(PORT9, ratio18_1, false); 
motor driveMotorRightThree = motor(PORT14, ratio18_1, true);    
motor driveMotorLeftThree = motor(PORT5, ratio18_1, false);

motor_group RightDrivetrain = motor_group(driveMotorRightOne, driveMotorRightTwo, driveMotorRightThree);
motor_group LeftDrivetrain  = motor_group(driveMotorLeftOne, driveMotorLeftTwo, driveMotorLeftThree);
motor_group FullDrivetrain = motor_group(driveMotorRightOne, driveMotorRightTwo, driveMotorRightThree, driveMotorLeftOne, driveMotorLeftTwo, driveMotorLeftThree);
bool DrivetrainNeedsToBeStopped_Controller1 = true;


#pragma endregion VEXcode Generated Robot Configuration

 
vex::task rc_auto_loop_task;

bool DrivetrainNeedsToBeStopped_Controller1 = true;
int rc_auto_loop_function_Controller1()
{
  while (true)
  {
    double y = Controller1.Axis3.position();
    double x = Controller1.Axis4.position();
    double a = Controller1.Axis1.position();

    // Apply deadzone
    if (abs(y) < 3) y = 0;
    if (abs(x) < 3) x = 0;
    if (abs(a) < 3) a = 0;

    // Normalize inputs to range [-1,1]
    double normY = y / 100.0;
    double normA = a / 100.0;

    // Exponential scaling (cubing keeps the sign and smoothens the curve)
    double expY = (0.25 * normY) + (0.75 * pow(normY, 3));
    double expA = (0.25 * normA) + (0.75 * pow(normA, 3));

    expY *= 100;
    expA *= 100;

    RightDrivetrain.spin(forward, expY - expA / 2, percent);
    LeftDrivetrain.spin(forward, expY + expA / 2, percent);
  } 

  return 0;
}
#pragma endregion VEXcode Generated Robot Configuration

void drivercontrol(void)
{
  rc_auto_loop_task = vex::task(rc_auto_loop_function_Controller1);
  task rc_auto_loop_task_Controller1(rc_auto_loop_function_Controller1);
}
    
    
    
   
  
  
  

 
  

