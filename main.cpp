#include "vex.h"
#include "drive.hpp"
#include "auton.hpp"
using namespace vex;

brain       Brain;
controller  Controller1(controllerType::primary);
competition Competition;

// Drive-Motoren
motor L1(PORT19, gearSetting::ratio18_1, false);
motor L2(PORT15, gearSetting::ratio18_1, false);
motor L3(PORT18, gearSetting::ratio18_1, false);
motor R1(PORT2,  gearSetting::ratio18_1, true);
motor R2(PORT9,  gearSetting::ratio18_1, true);
motor R3(PORT10, gearSetting::ratio18_1, true);

// Intake
motor Motorblau      (PORT14, gearSetting::ratio18_1, false);
motor Intakespeicher (PORT17, gearSetting::ratio18_1, false);
motor Intakeoben     (PORT1,  gearSetting::ratio18_1, false);

#if HAVE_PNEUMATICS
digital_out DigitalOutA(Brain.ThreeWirePort.A);
#endif

// ------- Auton Helper -------
inline double myAbs(double x){ return x<0 ? -x : x; }
constexpr double PI = 3.14159265358979323846;
constexpr double WHEEL_DIAM_MM  = 100.0;
constexpr double TRACK_WIDTH_MM = 320.0;

double cmToDeg(double cm) {
  const double wheel_circum_mm = PI * WHEEL_DIAM_MM;
  const double dist_mm         = cm * 10.0;
  const double revs            = dist_mm / wheel_circum_mm;
  return revs * 360.0;
}
void leftSpinFor(directionType d, double rot, rotationUnits u, bool waitAll) {
  L1.spinFor(d, rot, u, false);
  L2.spinFor(d, rot, u, false);
  L3.spinFor(d, rot, u, waitAll);
}
void rightSpinFor(directionType d, double rot, rotationUnits u, bool waitAll) {
  R1.spinFor(d, rot, u, false);
  R2.spinFor(d, rot, u, false);
  R3.spinFor(d, rot, u, waitAll);
}
void setDriveVel(int pct) {
  L1.setVelocity(pct, percentUnits::pct);
  L2.setVelocity(pct, percentUnits::pct);
  L3.setVelocity(pct, percentUnits::pct);
  R1.setVelocity(pct, percentUnits::pct);
  R2.setVelocity(pct, percentUnits::pct);
  R3.setVelocity(pct, percentUnits::pct);
}
void driveStraight(double cm, int vel_pct = AUTON_DRIVE_SPEED) {
  const double deg = cmToDeg(myAbs(cm));
  const directionType dir = (cm >= 0) ? directionType::fwd : directionType::rev;
  setDriveVel(vel_pct);
  leftSpinFor(dir,  deg, rotationUnits::deg, false);
  rightSpinFor(dir, deg, rotationUnits::deg, true);
}
void turnLeft(double deg, int vel_pct = AUTON_TURN_SPEED) {
  const double arc_mm   = PI * TRACK_WIDTH_MM * (myAbs(deg) / 360.0);
  const double arc_cm   = arc_mm / 10.0;
  const double wheelDeg = cmToDeg(arc_cm);
  setDriveVel(vel_pct);
  leftSpinFor(directionType::fwd,  wheelDeg, rotationUnits::deg, false);
  rightSpinFor(directionType::rev, wheelDeg, rotationUnits::deg, true);
}

// -------- Autonomous --------
void auton::runAuto() {
  driveStraight(-AUTON_FWD1);
  turnLeft(+AUTON_TURN1);
  driveStraight(-AUTON_FWD2);

  Motorblau.spin     (directionType::fwd, 100, velocityUnits::pct);
  Intakespeicher.spin(directionType::fwd, 100, velocityUnits::pct);
  Intakeoben.spin    (directionType::fwd, 100, velocityUnits::pct);
  wait(AUTON_INTAKE_IN, timeUnits::msec);
  Motorblau.stop(); Intakespeicher.stop(); Intakeoben.stop();

  turnLeft(+AUTON_TURN2);

  Motorblau.spin     (directionType::rev, 100, velocityUnits::pct);
  Intakespeicher.spin(directionType::rev, 100, velocityUnits::pct);
  Intakeoben.spin    (directionType::rev, 100, velocityUnits::pct);
  wait(AUTON_INTAKE_OUT, timeUnits::msec);
  Motorblau.stop(); Intakespeicher.stop(); Intakeoben.stop();
}

void autonomous()    { auton::runAuto(); }
void drivercontrol() { drive::run(); }

int main() {
  Competition.autonomous(autonomous);
  Competition.drivercontrol(drivercontrol);
  while (true) wait(100, timeUnits::msec);
}
