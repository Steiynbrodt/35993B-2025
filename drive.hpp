#include "vex.h"
#include <cstdlib>
#include <cmath>
#include <stdbool.h>
#include <math.h>

#include <thread>
#define FMT_HEADER_ONLY
using namespace vex;

brain Brain;
competition Competition;
controller Controller1 = controller(primary);
gps GPS17 = gps(PORT15,-100.00, +30.00, mm, +90);

inertial INS  = inertial(PORT14);
motor driveMotorLeftOne = motor(PORT21, ratio18_1, false);  
motor driveMotorLeftTwo = motor(PORT12, ratio18_1, false); 
motor driveMotorLeftThree = motor(PORT13, ratio18_1, false);


motor driveMotorRightOne = motor(PORT18, ratio18_1, true); 
motor driveMotorRightTwo = motor(PORT9, ratio18_1, true);
motor driveMotorRightThree = motor(PORT11, ratio18_1, true);  


motor_group RightDrivetrain = motor_group(driveMotorRightOne, driveMotorRightTwo, driveMotorRightThree);
motor_group LeftDrivetrain  = motor_group(driveMotorLeftOne, driveMotorLeftTwo, driveMotorLeftThree);
motor_group FullDrivetrain = motor_group(driveMotorRightOne, driveMotorRightTwo, driveMotorRightThree, driveMotorLeftOne, driveMotorLeftTwo, driveMotorLeftThree);
bool DrivetrainNeedsToBeStopped_Controller1 = false;

motor in1 = motor(PORT1, ratio18_1, false);  
motor in2 = motor(PORT4 ,ratio18_1, false); 
motor in3 = motor(PORT3, ratio18_1, false); 
motor in4 = motor(PORT5, ratio18_1, false); 
#pragma endregion VEXcode Generated Robot Configuration



 
vex::task rc_auto_loop_task;

   
void inntake(int intake ){
  //Intake.spin(forward,100,percent);
  vex::task::sleep(intake);
  //Intake.stop();
}


  // --- Button handlers (no blocking, no while-loops) ---
void R1Pressed()  { in1.setVelocity(100, percent); in1.spin(forward); }
void R1Released() { in1.stop(coast); }

void R2Pressed()  { in1.setVelocity(100, percent); in1.spin(reverse); }
void R2Released() { in1.stop(coast); }

void L1Pressed()  { in3.setVelocity(100, percent); in3.spin(reverse); } // was -100% in your code
void L1Released() { in3.stop(coast); }

void L2Pressed()  { in3.setVelocity(100, percent); in3.spin(forward); }
void L2Released() { in3.stop(coast); }



void RrPressed()  { in2.setVelocity(100, percent); in2.spin(forward); }
void RrReleased() { in2.stop(coast); }

void yPressed()  { in2.setVelocity(100, percent); in2.spin(reverse); }
void yReleased() { in2.stop(coast); }


// --- Driver control using real callbacks ---
void drivercontrol(void) {
  // Bind callbacks ONCE (outside the loop)
  
  Controller1.ButtonR1.pressed(R1Pressed);
  
  Controller1.ButtonR2.pressed(R2Pressed);
  

  Controller1.ButtonL1.pressed(L1Pressed);
  
  Controller1.ButtonL2.pressed(L2Pressed);
  

  Controller1.ButtonRight.pressed(RrPressed);
  

  Controller1.ButtonY.pressed(yPressed);
  

while (true) {
  
    // Arcade drive (same math you had)
    double left  = Controller1.Axis3.position() - Controller1.Axis1.position() / 2.0;
    double right = Controller1.Axis3.position() + Controller1.Axis1.position() / 2.0;

    LeftDrivetrain.spin(reverse, left, percent);
    RightDrivetrain.spin(reverse, right, percent);

    wait(20, msec); // let the scheduler run the callbacks
    
  
}
}
    
    
 
  
  
  

 
  





