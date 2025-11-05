#include "vex.h"
#include <cmath>  // fabs
using namespace vex;

// =================== helpers ===================
inline double wrap180(double a){
  while (a >  180.0) a -= 360.0;
  while (a < -180.0) a += 360.0;
  return a;
}
static inline double clampd(double v, double lo, double hi){
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

// =================== tuning ====================
// Distance PID (forward) + Heading PD (IMU)
// Distance PID
static constexpr double kP_dist = 0.050;   // was 0.010
static constexpr double kI_dist = 0.000;   // keep 0 to avoid windup
static constexpr double kD_dist = 0.0015;  // was 0.0008

static constexpr double kP_head = 0.60;  // was 0.80
static constexpr double kD_head = 0.10;  // was 0.15

// Output shaping
static constexpr double maxPct         = 100; // cap forward power
static constexpr double minPct         = 17;  // overcome stiction
static constexpr double slewPctPer10ms = 40;   // accel limit per 10ms tick

// Stop conditions
static constexpr double settleTolMm  = 8.0;  // was 5.0
static constexpr double settleTolDeg = 1.5;  // was 1.0
static constexpr int    settleTimeMs = 120;  // was 200
 // distance tolerance
 // heading tolerance
 // must stay within tol

// ================= drivetrain ==================
// GEOMETRY: set these to YOUR robot
static constexpr double wheelDiamMm = 61.11105;   // measured tire OD
// gearRatio = WHEEL revs per MOTOR rev (often <1 for a reduction)
static constexpr double gearRatio   = 1.6666;     // example
static constexpr double PI          = 3.14159265358979323846;
static constexpr double mmPerRot    = PI * wheelDiamMm * gearRatio;

// Your project provides these:
extern motor_group LeftDrivetrain;
extern motor_group RightDrivetrain;
extern inertial     INS;

// ============== timeout simulation =============
// Cartridge RPM (100/200/600) and a fudge factor (load, not full command)
static constexpr double motorRpm   = 200.0; // set to your cartridge
static constexpr double speedFudge = 0.7;  // 0.5–0.7 typical

// Predict a safe timeout by simulating your own control loop at 10ms.
static int estimateTimeoutMs(double distanceMm){
  // Your robot drives "forward" when motors spin REVERSE:
  const double fwdSign = -1.0;
  const double dt = 0.01; // 10 ms

  // Approx max linear speed from %command mapping
  const double vMax = (motorRpm * gearRatio * PI * wheelDiamMm / 60.0) * speedFudge; // mm/s

  // Simulated loop state
  double sMm = 0.0;              // simulated traveled (encoder sense)
  double lastErr = distanceMm;   // same init as real loop
  double integ   = 0.0;
  double lastFwd = 0.0;
  int tMs = 0;

  const int hardCapMs = 20000; // 20 s absolute cap

  while (tMs < hardCapMs){
    const double traveledMm = sMm * fwdSign;
    const double err  = distanceMm - traveledMm;      // mm
    const double derr = (err - lastErr) / dt;         // mm/s

    if (err < 50 && err > -50) integ += err * dt; else integ = 0.0;

    double fwd = kP_dist * err + kI_dist * integ + kD_dist * derr;

    // clamps & minimum & slew (same as real loop)
    if (fwd >  maxPct) fwd =  maxPct;
    if (fwd < -maxPct) fwd = -maxPct;
    if (fwd != 0 && (fwd > -minPct && fwd < minPct))
      fwd = (fwd > 0 ? minPct : -minPct);

    double step = fwd - lastFwd;
    if (step >  slewPctPer10ms) step =  slewPctPer10ms;
    if (step < -slewPctPer10ms) step = -slewPctPer10ms;
    fwd = lastFwd + step;

    // %command -> linear speed (ignore heading mixing for timeout)
    const double v = (fabs(fwd) / 100.0) * vMax; // mm/s

    // integrate distance in commanded forward sense
    sMm += v * dt * (fwd >= 0 ? +1.0 : -1.0);

    // done when within distance tolerance (add settle margin when returning)
    const double remaining = distanceMm - sMm * fwdSign;
    if (fabs(remaining) <= settleTolMm)
      return tMs + settleTimeMs + 200; // +200ms slack

    lastErr = err;
    lastFwd = fwd;
    tMs += 10;
  }
  return hardCapMs;
}

// ================= drive function ==============
void driveStraightMm(double distanceMm){
  // Dynamic timeout derived from the simulator above
  const int timeoutMs_dyn = estimateTimeoutMs(distanceMm);

  // Your build: forward motion requires spinning motors in REVERSE.
  const double fwdSign = -1.0;

  INS.setRotation(0, rotationUnits::deg);
  wait(50, timeUnits::msec);

  LeftDrivetrain.resetPosition();
  RightDrivetrain.resetPosition();

  double lastErr = distanceMm;
  double integ   = 0.0;
  double lastFwd = 0.0;
  int    within  = 0;
  int    tMs     = 0;

  while (tMs < timeoutMs_dyn){
    // Read motor shaft revolutions and average
    const double L = LeftDrivetrain.position(turns);
    const double R = RightDrivetrain.position(turns);
    const double avgRev = 0.5 * (L + R);

    // Flip sign so reverse spin counts as positive forward distance
    const double traveledMm = avgRev * mmPerRot * fwdSign;

    // Distance loop (PID on distance)
    const double err  = distanceMm - traveledMm;        // mm
    const double derr = (err - lastErr) / 0.01;         // mm/s (10ms)

    if (err < 50 && err > -50) integ += err * 0.01; else integ = 0.0;

    double fwd = kP_dist * err + kI_dist * integ + kD_dist * derr;

    // Heading hold (PD using IMU rate)
    const double head = wrap180(0.0 - INS.rotation(rotationUnits::deg));
    const double rate = INS.gyroRate(axisType::zaxis, velocityUnits::dps);
    const double turn = kP_head * head - kD_head * rate;

    // Clamp & minimum forward
    fwd = clampd(fwd, -maxPct, maxPct);
    if (fwd != 0 && (fwd > -minPct && fwd < minPct))
      fwd = (fwd > 0 ? minPct : -minPct);

    // Slew on forward only
    double step = fwd - lastFwd;
    if (step >  slewPctPer10ms) step =  slewPctPer10ms;
    if (step < -slewPctPer10ms) step = -slewPctPer10ms;
    fwd = lastFwd + step;

    // Mix to sides
    const double leftPct  = clampd(fwd + turn, -100, 100);
    const double rightPct = clampd(fwd - turn, -100, 100);

    // Spin REVERSE to go physically forward on your robot
    LeftDrivetrain.spin(directionType::rev, leftPct,  velocityUnits::pct);
    RightDrivetrain.spin(directionType::rev, rightPct, velocityUnits::pct);

    // Settle-only exit
    if (fabs(err) <= settleTolMm && fabs(head) <= settleTolDeg){
      within += 10;
      if (within >= settleTimeMs) break;
    } else {
      within = 0;
    }

    lastErr = err;
    lastFwd = fwd;
    wait(10, timeUnits::msec);
    tMs += 10;
  }

  LeftDrivetrain.stop(brakeType::brake);
  RightDrivetrain.stop(brakeType::brake);
   wait(150, timeUnits::msec);
    LeftDrivetrain.stop(coast);
  RightDrivetrain.stop(coast);

}
