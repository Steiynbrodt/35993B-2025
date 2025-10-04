Call calibrateINSFromGPS() once after sensors are ready (e.g., at the start of autonomous).

Then call NAVI(targetX_mm, targetY_mm) to plan and go.

If you want stronger path smoothing, keep pathSmoothingEnabled = true (default).

If you see jitter near goals, raise waypointTolerance a bit (e.g., 40–50 mm).

If the bot replans too often, increase deviationThreshold (e.g., to 70–90 mm).

If you hit any build issues (SDK names differ, etc.), paste the error and I’ll adjust the code to your exact VEX project setup.


