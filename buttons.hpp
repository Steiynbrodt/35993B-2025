// turning the Intake forwards
void onButtonR1Press()
{
  //double donutDistance = DonutSensor.objectDistance(distanceUnits::mm);
  while(Controller1.ButtonR1.pressing())
  {
    Intake.setVelocity(100, percent);
    Intake.spin(forward);
    vex::task::sleep(50);
  }
  Intake.stop();
  vex::task::sleep(50);
}

// turning the Intake backwards
void onButtonR2Press()
{
  while(Controller1.ButtonR2.pressing())
  {
    Intake.setVelocity(-100, percent);
    Intake.spin(forward);
    vex::task::sleep(50);
  }
  Intake.stop();
  vex::task::sleep(50);
}

// Motor to use the Holer
void onButtonL1Press()
{
  while(Controller1.ButtonL1.pressing()){
    Pneumatic1.set(!Pneumatic1.value());
    vex::task::sleep(500);
  }
 
}

void stakeropen(void){

  Pneumatic1.set(!Pneumatic1.value());
  vex::task::sleep(500);
//holer.spin(forward,-100,percent);
}

void inntake(int intake ){
  Intake.spin(forward,100,percent);
  vex::task::sleep(intake);
  Intake.stop();
}


  