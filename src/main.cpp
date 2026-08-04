/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       mlgodoy                                                   */
/*    Created:      6/25/2026, 5:14:40 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

controller Controller1;

// define your global instances of motors and other devices here

brain Brain;

inertial Gyro(PORT6);

aivision AISensor(PORT5);

motor LeftFront(PORT3, ratio18_1, true);
motor LeftBack(PORT4, ratio18_1, true);
motor RightFront(PORT1, ratio18_1, false);
motor RightBack(PORT2, ratio18_1, false);

// FUNCTIONS

double pi = 3.14;
double dia = 3.25;
double gR = 0.5;
double x = 0;

void tankDrive(double lspeed, double rspeed, int wt)
{
  lspeed = lspeed * 120;
  rspeed = rspeed * 120;
  LeftFront.spin(forward, lspeed, voltageUnits::mV);
  LeftBack.spin(forward, lspeed, voltageUnits::mV);
  RightFront.spin(forward, rspeed, voltageUnits::mV);
  RightBack.spin(forward, rspeed, voltageUnits::mV);
  task::sleep(wt);
}

void arcadeDrive()

{
  int throttle = Controller1.Axis3.position(percent);   // Forward/Backward
  int turn = Controller1.Axis1.position(percent) * 0.5; // Left/Right
  int leftSpeed = throttle + turn;
  int rightSpeed = throttle - turn;

  int wt;

  // 2. Apply a 5% deadzone to prevent joystick drift
  if (abs(throttle) < 5)
    throttle = 0;
  if (abs(turn) < 5)
    turn = 0;

  // 3. Combine inputs and spin the motors
  LeftFront.spin(forward, throttle + turn, percent);
  LeftBack.spin(forward, throttle + turn, percent);
  RightFront.spin(forward, (throttle - turn), percent);
  RightBack.spin(forward, (throttle - turn), percent);

  // limit motor speeds to -100 to 100%
  if (leftSpeed > 100)
    leftSpeed = 100;
  if (leftSpeed < -100)
    leftSpeed = -100;

  if (rightSpeed > 100)
    rightSpeed = 100;
  if (rightSpeed < -100)
    rightSpeed = -100;
}

void driveBrake()

{
  LeftFront.stop();
  LeftBack.stop();
  RightFront.stop();
  LeftBack.stop();
}
// Health of the robot functions
int YOFFSET = 20;   // First Line position
int NewLineY = 20;  // Buffer Between Lines
int LineNumber = 0; // What row the line is on(increments every new motor)

void MotorDisplay(int y, double curr, double temp)
{
  Brain.Screen.setFillColor(transparent);
  Brain.Screen.printAt(5, YOFFSET + LineNumber * NewLineY, "Current: %.1fA", curr);
  if (curr < 1)
    Brain.Screen.setFillColor(green);
  else if (curr >= 1 && curr <= 2.5)
    Brain.Screen.setFillColor(yellow);
  else
    Brain.Screen.setFillColor(red);
  Brain.Screen.drawRectangle(140, YOFFSET + LineNumber * NewLineY - 15, 15, 15);

  Brain.Screen.setFillColor(transparent);
  Brain.Screen.printAt(160, YOFFSET + LineNumber * NewLineY, "Temp: %.1fC", temp);
  if (temp < 45)
    Brain.Screen.setFillColor(green);
  else if (temp <= 50 && temp >= 45)
    // TRUE and TRUE --> True
    // TRUE and FALSE --> False
    // FALSE and FALSE --> False
    Brain.Screen.setFillColor(yellow);
  else
    Brain.Screen.setFillColor(red);
  Brain.Screen.drawRectangle(275, YOFFSET + LineNumber * NewLineY - 15, 15, 15);
  Brain.Screen.setFillColor(transparent);
}

void Display()
{
  double leftFrontCurr = LeftFront.current(amp);
  double leftFrontTemp = LeftFront.temperature(celsius);

  double leftBackCurr = LeftBack.current(amp);
  double leftBackTemp = LeftBack.temperature(celsius);

  double rightFrontCurr = RightFront.current(amp);
  double rightFrontTemp = RightFront.temperature(celsius);

  double rightBackCurr = RightBack.current(amp);
  double rightBackTemp = RightBack.temperature(celsius);

  LineNumber++;
  if (LeftFront.installed())
  {
    MotorDisplay(0, leftFrontCurr, leftFrontTemp);
    Brain.Screen.printAt(300, YOFFSET + LineNumber * NewLineY, "LeftFront");
  }
  else
  {
    Brain.Screen.setPenColor(red);
    Brain.Screen.printAt(5, YOFFSET + LineNumber * NewLineY, "LeftFront Problem");
    Brain.Screen.setPenColor(white);
  }
  LineNumber++;

  if (LeftBack.installed())
  {
    MotorDisplay(0, leftBackCurr, leftBackTemp);
    Brain.Screen.printAt(300, YOFFSET + LineNumber * NewLineY, "LeftBack");
  }
  else
  {
    Brain.Screen.setPenColor(red);
    Brain.Screen.printAt(5, YOFFSET + LineNumber * NewLineY, "LeftBack Problem");
    Brain.Screen.setPenColor(white);
  }
  LineNumber++;

  if (RightFront.installed())
  {
    MotorDisplay(60, rightFrontCurr, rightFrontTemp);
    Brain.Screen.printAt(300, YOFFSET + LineNumber * NewLineY, "RightFront");
  }
  else
  {
    Brain.Screen.setPenColor(red);
    Brain.Screen.printAt(5, YOFFSET + LineNumber * NewLineY, "RightFront Problem");
    Brain.Screen.setPenColor(white);
  }
  LineNumber++;

  if (RightBack.installed())
  {
    MotorDisplay(60, rightBackCurr, rightBackTemp);
    Brain.Screen.printAt(300, YOFFSET + LineNumber * NewLineY, "RightBack");
  }
  else
  {
    Brain.Screen.setPenColor(red);
    Brain.Screen.printAt(5, YOFFSET + LineNumber * NewLineY, "RightBack Problem");
    Brain.Screen.setPenColor(white);
  }
  LineNumber = 0;
}

void inchDrive(float target, int speed)
{

  // TODO: SLOW DOWN BEFORE TARGET

  LeftFront.setPosition(0, rev);
  x = LeftFront.position(rev) * dia * pi * gR;

  if (target >= 0)
  {
    while (x <= target) // While we still need to go
    {
      tankDrive(speed, speed, 10);                 // drive
      x = LeftFront.position(rev) * dia * pi * gR; // update x
    }
  }
  else if (target < 0) // same but reverse
  {
    while (x <= fabs(target))
    {
      tankDrive(-speed, -speed, 10);
      x = -LeftFront.position(rev) * dia * pi * gR;
    }
  }
  tankDrive(0, 0, 0);
}

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

  Gyro.calibrate();
  while (Gyro.isCalibrating())
  {
    wait(20, msec);
  }

  LeftFront.resetPosition();
  RightFront.resetPosition();
  LeftBack.resetPosition();
  RightBack.resetPosition();

  Brain.Screen.clearScreen();
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

void autonomous(void)
{
  inchDrive(36, 30);
  wait(500, msec);
  inchDrive(-36, 30);
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

bool arcadeMode = true;
bool lastButtonState = false;

void usercontrol(void)
{
  // User control code here, inside the loop
  int count = 0;
  while (1)
  {

    bool buttonState = Controller1.ButtonY.pressing();

    if (buttonState && !lastButtonState)
    {
      arcadeMode = !arcadeMode; // Switch modes
    }
    lastButtonState = buttonState;

    if (arcadeMode)
    {
      arcadeDrive();
    }
    else
    {
      tankDrive(Controller1.Axis3.position(), Controller1.Axis2.position(), 10);
    }

    count++;
    if (count == 25)
    {
      Display();
      count = 0;
    }

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

  // Prevent main from exiting with an infinite loop.
  while (true)
  {
    wait(100, msec);
  }
}
