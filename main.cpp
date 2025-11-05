#include "vex.h"
#include "../vex.h"
#include "drive.hpp"
   
#include "autonomus.hpp"

using namespace vex;

// A global instance of competition


// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

  // 0 = Left, 1 = Right, 2 = Skills


// Globals (place near top of file)






/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/



//
// Main will set up the competition functions and callbacks.
//
int main() {
 
  // Set up callbacks for autonomous and driver control periods.
  //test();
 
  
  Competition.drivercontrol(drivercontrol);
  Competition.autonomous(autonomous);
  pre_auton();
 
  
  
}