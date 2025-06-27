void turnTo(int targetAngleDeg) {
  targetAngleDeg = normalize360(targetAngleDeg);
  double currentAngle = normalize360(GPS17.heading());
  double error = shortestAngleDiff(targetAngleDeg, currentAngle);
  double direction = (error > 0) ? 1 : -1;

  while (fabs(error) > 10) {
    LeftDrivetrain.spin(forward,10.00,percent);
    RightDrivetrain.spin(reverse,10.00,percent);
    

    wait(50, msec);

    currentAngle = normalize360(GPS17.heading());
    error = shortestAngleDiff(targetAngleDeg, currentAngle);
    direction = (error > 0) ? 1 : -1;
  }

  
  FullDrivetrain.stop();
  
  
  
}