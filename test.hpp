// Testfile: tests.cpp
#include "vex.h"
#include <cassert>
#include <iostream>
 using namespace vex;
  void drivetest(){
    LeftDrivetrain.spin(forward, 20, percent);
    RightDrivetrain.spin(forward, 20, percent);
    task::sleep(2000);
    FullDrivetrain.stop(brake);

 } 
 void turnlefttest(void){
  LeftDrivetrain.spin(forward);
  task::sleep(2000);
  LeftDrivetrain.stop(brake);
 }
 void turnrighttest(void){
  RightDrivetrain.spin(forward);
  task::sleep(2000);
  RightDrivetrain.stop(brake);
 }

void test_normalize360() {
  assert(normalize360(-30) == 330);
  assert(normalize360(0) == 0);
  assert(normalize360(360) == 360);
  std::cout << "normalize360 passed\n";
}

void test_clamp() {
  assert(clamp(5, 0, 10) == 5);
  assert(clamp(-5, 0, 10) == 0);
  assert(clamp(15, 0, 10) == 10);
  std::cout << "clamp passed\n";
}

void test_shortestAngleDiff() {
  assert(shortestAngleDiff(10, 350) == 20);
  assert(shortestAngleDiff(350, 10) == -20);
  assert(shortestAngleDiff(180, 0) == 180);
  std::cout << "shortestAngleDiff passed\n";
}

void test_gridToMM_and_toGridCoord() {
  int grid = 5;
  double mm = gridToMM(grid);
  assert(toGridCoord(mm) == grid);
  std::cout << "gridToMM/toGridCoord passed\n";
}

void test_add_and_clear_obstacles() {
  clearObstacles();
  addObstacle(0, 0);
  assert(!walkable[OFFSET][OFFSET]);
  clearObstacles();
  assert(walkable[OFFSET][OFFSET]);
  std::cout << "add/clear obstacle passed\n";
}

void run_all_tests() {
  test_normalize360();
  test_clamp();
  test_shortestAngleDiff();
  test_gridToMM_and_toGridCoord();
  test_add_and_clear_obstacles();
 // drivetest();
 //turnlefttest();
 //task::sleep(1000);
 //turnrighttest();

  std::cout << "All tests passed!\n";
}

int test () {
  run_all_tests();
  return 0;
}
