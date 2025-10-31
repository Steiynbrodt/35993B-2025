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
gps GPS17 = gps(PORT8,-140.00, 55.00, mm, -90);

inertial INS  = inertial(PORT11);
bool piston1Extended = false;
vex::digital_out piston1 = vex::digital_out(Brain.ThreeWirePort.A);
bool piston2Extended = false;
vex::digital_out piston2 = vex::digital_out(Brain.ThreeWirePort.B);
bool piston3Extended = false;
vex::digital_out piston3 = vex::digital_out(Brain.ThreeWirePort.D);

motor driveMotorLeftOne = motor(PORT19, ratio18_1, false);  
motor driveMotorLeftTwo = motor(PORT15, ratio18_1, false); 
motor driveMotorLeftThree = motor(PORT6, ratio18_1, false);


motor driveMotorRightOne = motor(PORT3, ratio18_1, true); 
motor driveMotorRightTwo = motor(PORT9, ratio18_1, true);
motor driveMotorRightThree = motor(PORT10, ratio18_1, true);  

motor in1 = motor(PORT1, ratio18_1, false);  //intake oben 
motor in2 = motor(PORT17 ,ratio18_1, false); //mitte
motor in3 = motor(PORT14, ratio18_1, false); //unten 
//motor in4 = motor(PORT5, ratio18_1, false); 

motor_group RightDrivetrain = motor_group(driveMotorRightOne, driveMotorRightTwo, driveMotorRightThree);
motor_group LeftDrivetrain  = motor_group(driveMotorLeftOne, driveMotorLeftTwo, driveMotorLeftThree);
motor_group FullDrivetrain = motor_group(driveMotorRightOne, driveMotorRightTwo, driveMotorRightThree, driveMotorLeftOne, driveMotorLeftTwo, driveMotorLeftThree);
bool DrivetrainNeedsToBeStopped_Controller1 = false;



#pragma endregion VEXcode Generated Robot Configuration



 
vex::task rc_auto_loop_task;

   
void inntake(int intake ){
  //Intake.spin(forward,100,percent);
  vex::task::sleep(intake);
  //Intake.stop();
}


 //buttons 
void R2Pressed()  {  in3.setVelocity(100, percent); in3.spin(reverse);in2.setVelocity(100, percent); in2.spin(forward);in1.setVelocity(100, percent); in1.spin(forward);}
void R2Released() {  in3.stop(coast);in2.stop(coast); in1.stop(coast); }

void L1Pressed()  {in3.setVelocity(-100, percent); in3.spin(reverse); in1.setVelocity(100, percent); in1.spin(forward);in2.setVelocity(100, percent); in2.spin(forward);}
void L1Released() {   in3.stop(coast);in2.stop(coast); in1.stop(coast);}

void R1Pressed()  { in3.setVelocity(100, percent); in3.spin(forward); in1.setVelocity(0, percent); in1.spin(reverse);in2.setVelocity(100, percent); in2.spin(reverse);} // was -100% in your code
void R1Released() {  in3.stop(coast);in2.stop(coast); in1.stop(coast); }

void L2Pressed()  { in3.setVelocity(100, percent); in3.spin(forward); in1.setVelocity(100, percent); in1.spin(reverse);in2.setVelocity(100, percent); in2.spin(forward);}
void L2Released() { in3.stop(coast);in2.stop(coast); in1.stop(coast); }



void RrPressed()  { while (true) {
if (Controller1.ButtonRight.pressing()) {
            wait(20, msec); // Simple debounce
            piston1Extended = !piston1Extended; // Toggle state
            piston1.set(piston1Extended);
            wait(300, msec); // Wait to prevent multiple toggles
        }
        wait(20, msec); // Prevent excessive loop cycling
    } }


void yPressed()  { if (Controller1.ButtonY.pressing()) {
            wait(20, msec); // Simple debounce
            piston2Extended = !piston2Extended; // Toggle state
            piston2.set(piston2Extended);
            wait(300, msec); // Wait to prevent multiple toggles
        }
        wait(20, msec); }
void yReleased() { in2.stop(coast); }

void leftpressed()  { while (true) {
if (Controller1.ButtonLeft.pressing()) {
            wait(20, msec); // Simple debounce
            piston3Extended = !piston3Extended; // Toggle state
            piston3.set(piston3Extended);
            wait(300, msec); // Wait to prevent multiple toggles
        }
        wait(20, msec); // Prevent excessive loop cycling
    } }

void drivercontrol(void) {
  
  // --- Button bindings (pressed + released) ---
  Controller1.ButtonR1.pressed(R1Pressed);
  Controller1.ButtonR1.released(R1Released);

  Controller1.ButtonR2.pressed(R2Pressed);
  Controller1.ButtonR2.released(R2Released);

  Controller1.ButtonL1.pressed(L1Pressed);
  Controller1.ButtonL1.released(L1Released);

  Controller1.ButtonL2.pressed(L2Pressed);
  Controller1.ButtonL2.released(L2Released);

  // If you want Right + Y to control in2 (as in your code)
  Controller1.ButtonRight.pressed(RrPressed);
  Controller1.ButtonLeft.pressed(leftpressed);
  

  Controller1.ButtonY.pressed(yPressed);
  
  // If you actually wanted Up (instead of Right), use this instead:
  // Controller1.ButtonUp.pressed(RrPressed);
  // Controller1.ButtonUp.released(RrReleased);

while (true) {
  
    double leftm  = Controller1.Axis3.position() + Controller1.Axis1.position();
    double rightm = Controller1.Axis3.position() - Controller1.Axis1.position();
   
    LeftDrivetrain.spin(reverse, leftm, percent);
    RightDrivetrain.spin(reverse, rightm, percent);

    wait(20, msec); 
    
  
}
}
    
    
 
  
  
  

 
  





