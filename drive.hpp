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
digital_out piston1(Brain.ThreeWirePort.A);
digital_out piston2(Brain.ThreeWirePort.B);
digital_out piston3(Brain.ThreeWirePort.D);


// State tracking (so toggles work everywhere)
bool piston1Extended = true;
bool piston2Extended = false;
bool piston3Extended = false;
bool piston4Extended = false;

// ---------- Deterministic setters (great for autonomous) ----------
void setPiston1(bool extended) { piston1Extended = extended; piston1.set(extended); }
void setPiston2(bool extended) { piston2Extended = extended; piston2.set(extended); }
void setPiston3(bool extended) { piston3Extended = extended; piston3.set(extended); }

// Optional convenience:
void extendPiston1() { setPiston1(true); }
void retractPiston1() { setPiston1(false); }
void extendPiston2() { setPiston2(true); }
void retractPiston2() { setPiston2(false); }
void extendPiston3() { setPiston3(true); }
void retractPiston3() { setPiston3(false); }


// ---------- Toggles (callable from anywhere, incl. autonomous) ----------
void togglePiston1() { setPiston1(!piston1Extended); }
void togglePiston2() { setPiston2(!piston2Extended); }
void togglePiston3() { setPiston3(!piston3Extended); }
void toggleboth(){
  setPiston2(!piston2Extended); 
  setPiston1(!piston1Extended); 

}

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

   Controller1.ButtonRight.pressed(togglePiston1); // A
  Controller1.ButtonY.pressed(togglePiston2);     // B
  Controller1.ButtonLeft.pressed(togglePiston3);  // D
   
   

while (true) {
  
    double leftm  = Controller1.Axis3.position() + Controller1.Axis1.position();
    double rightm = Controller1.Axis3.position() - Controller1.Axis1.position();
   
    LeftDrivetrain.spin(reverse, leftm, percent);
    RightDrivetrain.spin(reverse, rightm, percent);

    wait(20, msec); 
    
  
}
}
    
    
 
  
  
  

 
  





