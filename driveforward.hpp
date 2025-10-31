#include "vex.h"

using namespace vex;
inline double wrap180(double a){
    while (a >  180.0) a -= 360.0;
    while (a < -180.0) a += 360.0;
    return a;
}
// ----- tune these -----
static constexpr double kP_dist = 0.007;
static constexpr double kI_dist = 0.0;
static constexpr double kD_dist = 0.0008;
static constexpr double kP_head = 0.8;
static constexpr double kD_head = 0.15;

static constexpr double maxPct         = 55;
static constexpr double minPct         = 8;
static constexpr double slewPctPer10ms = 4;
static constexpr double settleTolMm    = 5;
static constexpr double settleTolDeg   = 1.0;
static constexpr int    settleTimeMs   = 200;
static constexpr int    timeoutMs      = 4000;

// ----- robot specifics -----
static constexpr double wheelDiamMm = 80;  // set yours
static constexpr double gearRatio   = 1.0;    // motor->wheel
static constexpr double mmPerRot    = M_PI * wheelDiamMm * gearRatio;

// Provided by your project:
extern motor_group LeftDrivetrain;
extern motor_group RightDrivetrain;
extern inertial INS;


// Clamp helper
static inline double clamp(double v, double lo, double hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

void driveStraightMm(double distanceMm) {
  // Zero yaw & encoders
  INS.setRotation(0, degrees);
  wait(50, msec);

  LeftDrivetrain.resetPosition();
  RightDrivetrain.resetPosition();

  // Controller state
  double lastDistErr   = distanceMm;
  double distIntegral  = 0.0;
  double lastFwdCmdPct = 0.0;

  int tMs = 0;
  int withinTolMs = 0;

  while (tMs < timeoutMs) {
    // motor_group::position(...) exists; returns average for the group
    double leftRev  = LeftDrivetrain.position(turns); // turns == revolutions
    double rightRev = RightDrivetrain.position(turns);
    double avgRev   = 0.5 * (leftRev + rightRev);
    double traveledMm = avgRev * mmPerRot;

    double distErr = distanceMm - traveledMm;
    double dErr    = (distErr - lastDistErr) / 0.01; // mm/s (10 ms loop)

    // modest anti-windup
    if (fabs(distErr) < 50) distIntegral += distErr * 0.01;
    else distIntegral = 0;

    // Forward PID (I is optional)
    double fwdCmdPct = kP_dist * distErr + kI_dist * distIntegral + kD_dist * dErr;

    // Heading PD with IMU (error + gyro damping)
    double curHeading = INS.rotation(degrees);
    double headErr    = wrap180(0.0 - curHeading);
    double headRate   = INS.gyroRate(zaxis, dps);
    double turnCmdPct = kP_head * headErr - kD_head * headRate;

    // Limit forward power & ensure minimum to keep moving
    fwdCmdPct = clamp(fwdCmdPct, -maxPct, maxPct);
    if (fabs(fwdCmdPct) > 0 && fabs(fwdCmdPct) < minPct)
      fwdCmdPct = (fwdCmdPct > 0 ? minPct : -minPct);

    // Slew rate (forward only)
    double delta   = fwdCmdPct - lastFwdCmdPct;
    double maxStep = (delta >= 0 ? +slewPctPer10ms : -slewPctPer10ms);
    if (fabs(delta) > fabs(maxStep)) fwdCmdPct = lastFwdCmdPct + maxStep;

    // Mix to sides
    double leftPct  = clamp(fwdCmdPct + turnCmdPct, -100, 100);
    double rightPct = clamp(fwdCmdPct - turnCmdPct, -100, 100);

    LeftDrivetrain.spin(directionType::fwd, leftPct, velocityUnits::pct);
    RightDrivetrain.spin(directionType::fwd, rightPct, velocityUnits::pct);

    // Settle detection
    if (fabs(distErr) <= settleTolMm && fabs(headErr) <= settleTolDeg) {
      withinTolMs += 10;
      if (withinTolMs >= settleTimeMs) break;
    } else {
      withinTolMs = 0;
    }

    lastDistErr   = distErr;
    lastFwdCmdPct = fwdCmdPct;
    wait(10, msec);
    tMs += 10;
  }

  LeftDrivetrain.stop(brakeType::brake);
  RightDrivetrain.stop(brakeType::brake);
}

