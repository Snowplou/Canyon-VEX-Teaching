#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "v5.h"
#include "v5_vcs.h"

class TankDrive;

vex::motor leftFrontMotor = vex::motor(vex::PORT12, vex::gearSetting::ratio18_1, true);
vex::motor leftMiddleMotor = vex::motor(vex::PORT1, vex::gearSetting::ratio18_1, true);
vex::motor leftBackMotor = vex::motor(vex::PORT11, vex::gearSetting::ratio18_1, true);
vex::motor_group leftDrive = vex::motor_group(leftFrontMotor, leftMiddleMotor, leftBackMotor);

vex::motor rightFrontMotor = vex::motor(vex::PORT13, vex::gearSetting::ratio18_1, false);
vex::motor rightMiddleMotor = vex::motor(vex::PORT9, vex::gearSetting::ratio18_1, false);
vex::motor rightBackMotor = vex::motor(vex::PORT19, vex::gearSetting::ratio18_1, false);
vex::motor_group rightDrive = vex::motor_group(rightFrontMotor, rightMiddleMotor, rightBackMotor);

vex::inertial inertialSensor = vex::inertial(vex::PORT20);
vex::rotation trackingWheel = vex::rotation(vex::PORT18);

#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))

#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)