#include "vex.h"
#include <cmath>  // fabs
using namespace vex;

// ======================================================================
// Helpers
// ======================================================================

// Normalize any angle to (-180, +180] degrees.
// Useful for heading error so the robot always turns the "short way".
inline double wrap180(double a){
  while (a >  180.0) a -= 360.0;
  while (a < -180.0) a += 360.0;
  return a;
}

// Clamp double into [lo, hi]. Used for power caps and mix limits.
static inline double clampd(double v, double lo, double hi){
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

// ======================================================================
// Tuning constants
// ======================================================================
// Controller structure:
// - Distance: PID on linear error (mm)
// - Heading:  PD on IMU (degrees + deg/s)
// Output shaping:
// - Caps, minimum output to break static friction, and slew-rate limiting

// ---------------- DISTANCE PID GAINS ----------------
// Units: error = mm, derr = mm/s, output = % power (0..±100)
// P pushes toward the target; I corrects steady bias; D damps approach.
static constexpr double kP_dist = 0.135;   // Proportional gain (mm -> %)
                                           // ↑ faster approach, but too high = oscillations.

static constexpr double kI_dist = 0.000;   // Integral gain (mm*s -> %)
                                           // Kept 0 to avoid windup when robot is close/paused.

static constexpr double kD_dist = 0.0040;  // Derivative gain ((mm/s) -> %)
                                           // Adds braking near the target; too high = twitch/noise.

// ---------------- HEADING PD GAINS ------------------
// Heading control keeps the chassis straight while driving forward.
// Using PD (no I) to avoid slow drift accumulation and overshoot.
// head = desired(0) - measured(INS.rotation) in degrees
// rate = INS.gyroRate() in deg/s
static constexpr double kP_head = 0.60;    // How hard to correct heading error (deg -> %)
static constexpr double kD_head = 0.10;    // Damps rotation using gyro rate (deg/s -> %)

// ---------------- OUTPUT SHAPING --------------------
// These shape the *distance* output before mixing with turn.
// maxPct: cap to protect motors; minPct: overcome stiction; slew: acceleration limit.
static constexpr double maxPct         = 100; // Max forward command magnitude (±%)
static constexpr double minPct         = 20;  // Minimum magnitude when non-zero (break static friction)
static constexpr double slewPctPer10ms = 80;  // Max change per 10ms tick (prevents wheelies/brownouts)

// ---------------- STOP CONDITIONS -------------------
// Drive stops only after staying inside both distance and heading tolerances
// continuously for settleTimeMs (debounce against brief crossings).
static constexpr double settleTolMm  = 10.0;  // Distance tolerance (mm)
static constexpr double settleTolDeg = 1.5;   // Heading tolerance (deg)
static constexpr int    settleTimeMs = 120;   // Must remain in tolerance for this long

// ======================================================================
// Drivetrain geometry and conversion
// ======================================================================
// mmPerRot converts motor shaft turns -> linear mm traveled at the wheel.
// gearRatio = (wheel revs) per (motor rev). For reductions: < 1.0
static constexpr double wheelDiamMm = 61.11105;        // Measured tire OD (mm)
static constexpr double gearRatio   = 1.6666;          // Example ratio (wheel/motor)
static constexpr double PI          = 3.14159265358979323846;
static constexpr double mmPerRot    = PI * wheelDiamMm * gearRatio;

// Provided by your project elsewhere:
extern motor_group LeftDrivetrain;
extern motor_group RightDrivetrain;
extern inertial     INS;

// ======================================================================
// Timeout estimation via simple simulation (defensive programming)
// ======================================================================
// We simulate what the loop *could* do to pick a reasonable time-out.
// Prevents soft-locks if the robot is stalled or blocked.
static constexpr double motorRpm   = 200.0; // Motor cartridge nominal RPM
static constexpr double speedFudge = 0.7;   // Real-world loss factor (load/voltage/slip)

// Predict a safe timeout by simulating the forward-only loop at 10ms.
static int estimateTimeoutMs(double distanceMm){
  // NOTE: Due to your build, "forward" requires motors to spin in REVERSE.
  // fwdSign flips encoder sense so positive distance = physically forward.
  const double fwdSign = -1.0;
  const double dt = 0.01; // 10 ms

  // Approximate max linear speed (mm/s) from %command at 100%
  const double vMax =
    (motorRpm * gearRatio * PI * wheelDiamMm / 60.0) * speedFudge;

  // State for simulation (not the real robot)
  double sMm = 0.0;            // synthetic traveled distance in "forward" sense
  double lastErr = distanceMm; // start with full error
  double integ   = 0.0;
  double lastFwd = 0.0;
  int tMs = 0;

  const int hardCapMs = 15000; // Absolute cap (15 s) to avoid runaway

  while (tMs < hardCapMs){
    // Convert sim state into the same error math as the real loop
    const double traveledMm = sMm * fwdSign;
    const double err  = distanceMm - traveledMm;  // mm remaining
    const double derr = (err - lastErr) / dt;     // mm/s

    // Simple anti-windup: integrate only when near target (|err| < 50 mm)
    if (err < 50 && err > -50) integ += err * dt; else integ = 0.0;

    // Distance PID -> "forward %" command
    double fwd = kP_dist * err + kI_dist * integ + kD_dist * derr;

    // Cap and minimum output to fight stiction
    if (fwd >  maxPct) fwd =  maxPct;
    if (fwd < -maxPct) fwd = -maxPct;
    if (fwd != 0 && (fwd > -minPct && fwd < minPct))
      fwd = (fwd > 0 ? minPct : -minPct);

    // Slew rate limit (accelerate/brake smoothly)
    double step = fwd - lastFwd;
    if (step >  slewPctPer10ms) step =  slewPctPer10ms;
    if (step < -slewPctPer10ms) step = -slewPctPer10ms;
    fwd = lastFwd + step;

    // Map %command → linear speed estimate (ignore heading in sim)
    const double v = (fabs(fwd) / 100.0) * vMax; // mm/s

    // Integrate simulated distance in the sign of fwd
    sMm += v * dt * (fwd >= 0 ? +1.0 : -1.0);

    // If within tolerance, return with settle margin and small slack
    const double remaining = distanceMm - sMm * fwdSign;
    if (fabs(remaining) <= settleTolMm)
      return tMs + settleTimeMs + 200; // extra 200ms slack

    lastErr = err;
    lastFwd = fwd;
    tMs += 10;
  }
  return hardCapMs; // worst-case fallback
}

// ======================================================================
// Drive straight for a given distance (mm), holding heading with IMU PD
// ======================================================================
void driveStraightMm(double distanceMm){
  // Dynamic timeout from the simulator (adapts to long vs. short moves)
  const int timeoutMs_dyn = estimateTimeoutMs(distanceMm);

  // Mechanical sign: your build drives forward when motors spin REVERSE.
  // We keep all distance math in "forward = +mm", then flip at the output.
  const double fwdSign = -1.0;

  // Zero heading reference; we want to hold current heading as "desired = 0"
  INS.setRotation(0, rotationUnits::deg);
  wait(50, timeUnits::msec); // small settle so gyro/IMU is stable

  // Reset both sides so encoder averaging starts from 0
  LeftDrivetrain.resetPosition();
  RightDrivetrain.resetPosition();

  // Distance loop state
  double lastErr = distanceMm;  // previous distance error (mm)
  double integ   = 0.0;         // integral accumulator (mm*s)
  double lastFwd = 0.0;         // last forward % after slew
  int    within  = 0;           // ms accumulated inside settle tolerances
  int    tMs     = 0;           // watchdog/timeout clock

  while (tMs < timeoutMs_dyn){
    // ---------------- feedback (distance) ----------------
    const double L = LeftDrivetrain.position(turns);
    const double R = RightDrivetrain.position(turns);
    const double avgRev = 0.5 * (L + R);             // average motor revs
    const double traveledMm = avgRev * mmPerRot * fwdSign; // convert to +mm forward

    // Distance PID terms
    const double err  = distanceMm - traveledMm;     // remaining distance (mm)
    const double derr = (err - lastErr) / 0.01;      // derivative (mm/s) at 10ms

    // Integral with simple windowed anti-windup (only near target)
    if (err < 50 && err > -50) integ += err * 0.01; else integ = 0.0;

    // Raw forward command (%)
    double fwd = kP_dist * err + kI_dist * integ + kD_dist * derr;

    // ---------------- heading hold (PD) ------------------
    // Desire: keep heading at 0 relative to when we zeroed INS.
    // head: signed angle error in degrees, wrapped to (-180, 180]
    // rate: current rotation speed (deg/s), D term to damp spin.
    const double head = wrap180(0.0 - INS.rotation(rotationUnits::deg));
    const double rate = INS.gyroRate(axisType::zaxis, velocityUnits::dps);
    const double turn = kP_head * head - kD_head * rate; // % added to left, subtracted from right

    // ---------------- output shaping --------------------
    // 1) Clamp forward to safe magnitude
    fwd = clampd(fwd, -maxPct, maxPct);

    // 2) Minimum output to overcome static friction (if non-zero)
    if (fwd != 0 && (fwd > -minPct && fwd < minPct))
      fwd = (fwd > 0 ? minPct : -minPct);

    // 3) Slew-limit forward changes to avoid current spikes/skids
    double step = fwd - lastFwd;
    if (step >  slewPctPer10ms) step =  slewPctPer10ms;
    if (step < -slewPctPer10ms) step = -slewPctPer10ms;
    fwd = lastFwd + step;

    // ---------------- mix and apply ---------------------
    // Mix heading into sides. Clamp to ±100% for the API.
    const double leftPct  = clampd(fwd + turn, -100, 100);
    const double rightPct = clampd(fwd - turn, -100, 100);

    // Mechanical sign flip: spin in REVERSE to move physically forward.
    LeftDrivetrain.spin(directionType::rev,  leftPct, velocityUnits::pct);
    RightDrivetrain.spin(directionType::rev, rightPct, velocityUnits::pct);

    // ---------------- settle detection ------------------
    // Must remain inside both distance and heading tolerances for
    // settleTimeMs continuously to declare "done".
    if (fabs(err) <= settleTolMm && fabs(head) <= settleTolDeg){
      within += 10;
      if (within >= settleTimeMs) break;
    } else {
      within = 0;
    }

    // ---------------- bookkeeping -----------------------
    lastErr = err;
    lastFwd = fwd;
    wait(10, timeUnits::msec); // loop period = 10 ms
    tMs += 10;
  }

  // Final stop: brake briefly to kill residual motion, then coast to relax motors.
  LeftDrivetrain.stop(brakeType::brake);
  RightDrivetrain.stop(brakeType::brake);
  wait(150, timeUnits::msec);
  LeftDrivetrain.stop(coast);
  RightDrivetrain.stop(coast);
}
