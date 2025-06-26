#include "vex.h"
#include <math.h>
#include <algorithm>
#include <vector>
#include <queue>
#include <fstream>
#include <limits>
#include <NAVI.hpp>

using namespace vex;

void spinfor(int TT){
    LeftDrivetrain.spin(forward, 30, percent);
    RightDrivetrain.spin(reverse, 30, percent);
    task::sleep(TT);
    FullDrivetrain.stop(brake);

}
void spin45(){
  LeftDrivetrain.spin(forward, 30, percent);
    RightDrivetrain.spin(reverse, 30, percent);
    task::sleep(250);
    FullDrivetrain.stop(brake);
}

void spin90(){
  LeftDrivetrain.spin(forward, 30, percent);
  RightDrivetrain.spin(reverse, 30, percent);
  task::sleep(500);
  FullDrivetrain.stop(brake);
}
void spinforCC(int OO){
  LeftDrivetrain.spin(reverse, 30, percent);
  RightDrivetrain.spin(forward, 30, percent);
  task::sleep(OO);
  FullDrivetrain.stop(brake);

}
void spin45CC(){
LeftDrivetrain.spin(reverse, 30, percent);
  RightDrivetrain.spin(forward, 30, percent);
  task::sleep(250);
  FullDrivetrain.stop(brake);
}

void spin90CC(){
LeftDrivetrain.spin(reverse, 30, percent);
RightDrivetrain.spin(forward, 30, percent);
task::sleep(500);
FullDrivetrain.stop(brake);
}

void driveforwardfor(int II){
  FullDrivetrain.spin(forward,40,percent);
  task::sleep(II);
  FullDrivetrain.stop();
}

void drivebackwardfor(int IO){
  FullDrivetrain.spin(reverse,40,percent);
  task::sleep(IO);
  FullDrivetrain.stop();
}
//------------------- AUTONOMOUS ROUTINE -------------------//
void vrcauton(void){

}
void hardauton(void){

  driveforwardfor(440);

drivebackwardfor(800);

vex::task::sleep(500);
spinfor(1000);
driveforwardfor(1000);
spinforCC(700);
driveforwardfor(1000);
spinforCC(545);
driveforwardfor(2250);
drivebackwardfor(300);
spinfor(1090);
drivebackwardfor(700);

vex::task::sleep(1000);
driveforwardfor(300); 




 



}
void AIMODE(void){

}
void autonomous() {
    
    NAVI(500,500);

  
}