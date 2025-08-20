#include "vex.h"
#include <cstdlib>
#include <cmath>
#include <stdbool.h>
#include <math.h>
#include "buttons.hpp"

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

 
void controlling(){

    void onButtonR1Press(void){
 
     while(Controller1.ButtonR1.pressing())
  {
    in4.setVelocity(100, percent);
    in4.spin(forward);
    vex::task::sleep(50);
  }
  in.stop();
  vex::task::sleep(50);
 }

// turning the Intake backwards
void onButtonR2Press(void){
  while(Controller1.ButtonR2.pressing())
  {
    in2.setVelocity(-100, percent);
    in2.spin(forward);
    vex::task::sleep(50);
  }
  in2.stop();
  vex::task::sleep(50);
}

// Motor to use the Holer
void onButtonL1Press()
{
  while(Controller1.ButtonL1.pressing()){
    
  }
 
}

void stakeropen(void){

  
//holer.spin(forward,-100,percent);
}

void inntake(int intake ){
  Intake.spin(forward,100,percent);
  vex::task::sleep(intake);
  Intake.stop();
}
    

  
while (true)
  {
    double drivetrainLeftSideSpeed = Controller1.Axis3.position() - Controller1.Axis1.position()/2;
    double drivetrainRightSideSpeed = Controller1.Axis3.position() +  Controller1.Axis1.position()/2;

    //if (abs(drivetrainLeftSideSpeed) < 5 && abs(drivetrainRightSideSpeed) < 5 && DrivetrainNeedsToBeStopped_Controller1) DrivetrainNeedsToBeStopped_Controller1 = false;
    //else DrivetrainNeedsToBeStopped_Controller1 = true;

    //double y = Controller1.Axis3.position();
    //double x = Controller1.Axis4.position();
    //double a = Controller1.Axis1.position();

    //if (y >= -5 && y <= 5) y = 0;
    //if (x >= -5 && x <= 5) x = 0;
    //if (a >= -5 && a <= 5) a = 0;

    //RightDrivetrain.spin(forward, y - a, percent);
    //LeftDrivetrain.spin(forward, y + a, percent);
    RightDrivetrain.spin(reverse,drivetrainRightSideSpeed, percent);
    LeftDrivetrain.spin(reverse,drivetrainLeftSideSpeed, percent);
    while(true){
      controlling();
    }
  }}
  
  

 
  

