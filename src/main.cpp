/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       jacob                                                     */
/*    Created:      7/30/2024, 6:16:07 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include <iostream>

using namespace vex;

// A global instance of competition
competition Competition;
brain Brain;

// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void)
{

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

const float kP = 5.0;
const float kI = 0;
const float kD = 0;

const float WHEEL_DIAMETER = 3.25;
const float TRACKING_WHEEL_DIAMETER = 2.75;
const float GEAR_RATIO = 48.0 / 36.0;

// Drive set distance
void drivePID(float target_distance_in)
{
  leftDrive.resetPosition();
  rightDrive.resetPosition();

  float I = 0;

  float initial_current_distance_deg = (leftDrive.position(deg) + rightDrive.position(deg)) / 2.0;
  float initial_current_distance_in = initial_current_distance_deg / 360.0 * M_PI * WHEEL_DIAMETER * GEAR_RATIO;

  float last_error = target_distance_in - initial_current_distance_in;

  while (true)
  {
    float current_distance_deg = (leftDrive.position(deg) + rightDrive.position(deg)) / 2.0;
    float current_distance_in = current_distance_deg / 360.0 * M_PI * WHEEL_DIAMETER * GEAR_RATIO;

    float error = target_distance_in - current_distance_in;

    float P = error * kP;
    I += (error * kI);
    float D = (last_error - error) * kD;

    float motor_speed = P + I + D;

    leftDrive.spin(forward, motor_speed, pct);
    rightDrive.spin(forward, motor_speed, pct);

    wait(20, msec);
    last_error = error;

    std::cout << Brain.timer(timeUnits::msec) << ", " << error << ", " << P << ", " << I << ", " << D << ", " << motor_speed << std::endl;
    // std::cout << Brain.timer(timeUnits::msec) << ", " << leftDrive.position(deg) << ", " << rightDrive.position(deg) << std::endl;
  }
}

void drive(float target_distance_in, float target_angle_deg)
{
  while (true)
  {
    float linear_speed = drive_pid(); // Returns driving speed
    float angular_speed = turn_pid(); // Returns turning speed

    leftDrive.spin(forward, linear_speed + angular_speed, pct);
    rightDrive.spin(forward, linear_speed - angular_speed, pct);
  }
}

// Runs odometry to calculate robot position
void odometryTask()
{
  float x = 0;
  float y = 0;
  float lastTrackingWheel = trackingWheel.position(deg);

  while (true)
  {
    float rotation = inertialSensor.rotation();

    float trackingWheelRotation = trackingWheel.position(deg);
    float trackingWheelDelta = trackingWheelRotation - lastTrackingWheel;
    float trackingWheelDelta_in = trackingWheelDelta / 360.0 * M_PI * TRACKING_WHEEL_DIAMETER;

    float dx = cos(rotation) * trackingWheelDelta_in;
    float dy = sin(rotation) * trackingWheelDelta_in;

    x += dx;
    y += dy;

    wait(20, msec);
    lastTrackingWheel = trackingWheelRotation;
  }
}

// Runs when autonomous is enabled
void autonomous()
{
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

// Runs when driver control is enabled
void usercontrol()
{
  // User control code here, inside the loop
  while (1)
  {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main()
{
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();
}
