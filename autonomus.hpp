#include "vex.h"
#include <math.h>
#include <algorithm>
#include <vector>
#include <queue>
#include <fstream>
#include <limits>
#include "NAVI_fixed.hpp"

using namespace vex;



 void hardcoded(void){

 }



void AIMODE(void){

}
void autonomous() {
 INS.calibrate();
 wait(5,sec);
  INS.setHeading(90,degrees);

  turnToYaw(90);
  

  
}