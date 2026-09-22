
/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       xxxx                                                     */
/*    Created:      xxxx                                                     */
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

inertial Gyro(PORT19);

aivision AISensor(PORT6, aivision::ALL_TAGS);

motor LeftFront(PORT20, ratio6_1, true);
motor LeftBack(PORT10, ratio6_1, true);
motor RightFront(PORT11, ratio6_1, false);
motor RightBack(PORT3, ratio6_1, false);

motor claw(PORT5, ratio18_1, false);

// FUNCTIONS

double pi = M_PI;
double dia = 3.25; // wheel diameter
double gR = 0.6;   // 36/60 gear ratio
double x = 0;

double DRIVE_KP = 0.8;
double MAX_CORRECTION = 5; // Maximum gyro correction in percent

void tankDrive(double lspeed, double rspeed, int wt)
{
  lspeed = lspeed * 1;
  rspeed = rspeed * 1;
  LeftFront.spin(forward, lspeed, pct);
  LeftBack.spin(forward, lspeed, pct);
  RightFront.spin(forward, rspeed, pct);
  RightBack.spin(forward, rspeed, pct);
  wait(10, msec);
}

void arcadeDrive()
{
  double throttle = Controller1.Axis3.position(percent);
  double turn = Controller1.Axis1.position(percent) * 0.9;

  // Deadzone
  if (abs(throttle) < 5)
    throttle = 0;

  if (abs(turn) < 5)
    turn = 0;

  int leftSpeed = throttle + turn;
  int rightSpeed = throttle - turn;

  if (throttle != 0)
  {
    turn = Controller1.Axis1.position(percent) * 0.93; // turn while driving
    int correction = 1;
    leftSpeed += correction;
  }
  else if (throttle == 0)
  {
    turn = Controller1.Axis1.position(percent) * 0.33; // turn when there's no driving
  }

  // Clamp actual motor speeds
  leftSpeed = fmax(-100, fmin(100, leftSpeed));
  rightSpeed = fmax(-100, fmin(100, rightSpeed));

  LeftFront.spin(forward, leftSpeed, percent);
  LeftBack.spin(forward, leftSpeed, percent);
  RightFront.spin(forward, rightSpeed, percent);
  RightBack.spin(forward, rightSpeed, percent);
}

double clawCurrentLimit = 0.6;
bool currentMax = false;

int clawIntake()
{
  while (true)
  {
    // 1. If the motor spikes past the limit, trip the latch permanently
    if (!currentMax && claw.current(amp) > clawCurrentLimit)
    {
      currentMax = true;
    }
    // 2. Override the motor if tripped
    if (currentMax)
    {
      claw.stop(brakeType::coast);
    }
    // Check every 20 milliseconds
    wait(20, msec);
  }
  return 0;
}

void driveBrake()
{
  LeftFront.stop(brakeType::brake);
  LeftBack.stop(brakeType::brake);
  RightFront.stop(brakeType::brake);
  RightBack.stop(brakeType::brake);
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

int autonSelect = 2;
int autonMax = 5;
int autonMin = 0;
bool selectingAuton = true;

void drawGUI()
{
  Brain.Screen.clearScreen();
  Brain.Screen.printAt(1, 40, "Select Auton and Press Go");
  Brain.Screen.printAt(1, 200, "Auton Selected =&d", autonSelect);
  Brain.Screen.setFillColor(red);
  Brain.Screen.drawRectangle(20, 50, 100, 100);
  Brain.Screen.drawCircle(300, 75, 25);
  Brain.Screen.printAt(25, 75, "Select");
  Brain.Screen.setFillColor(green);
  Brain.Screen.drawRectangle(170, 50, 100, 100);
  Brain.Screen.printAt(175, 75, "Go");
  Brain.Screen.setFillColor(black);
}

void selectAuton()
{

  int x = Brain.Screen.xPosition();
  int y = Brain.Screen.yPosition();

  if (x >= 20 && x <= 120 && y >= 50 && y <= 150)
  {
    autonSelect++;
    if (autonSelect > autonMax)
    {
      autonSelect = autonMin;
    }
    Brain.Screen.printAt(1, 20, "AutonSelected = %d", autonSelect);
  }

  if (x >= 170 && x <= 270 && y >= 50 && y <= 150)

  {
    selectingAuton = false;
    Brain.Screen.printAt(1, 200, "Auton = %d GO", autonSelect);
  }

  if (!selectingAuton)
  {
    Brain.Screen.setFillColor(green);
    Brain.Screen.drawCircle(300, 75, 25);
  }
  else
  {
    Brain.Screen.setFillColor(red);
    Brain.Screen.drawCircle(300, 75, 25);
  }

  wait(10, msec);
  Brain.Screen.setFillColor(black);
}

void inchDrive(double target, int speed)

{
  // Reset the left and right motor encoders.
  LeftFront.setPosition(0, rev);
  RightFront.setPosition(0, rev);

  // Reset the gyro to 0 degrees.
  Gyro.setRotation(0, degrees);

  timer printTimer;

  double distance = 0;

  // Keep driving until we reach the requested distance.
  // fabs() makes the target positive so this works for both
  // forward and backward driving.
  while (distance < fabs(target))
  {
    // Motor revolutions × wheel circumference × gear ratio = inches.
    double leftDistance =
        LeftFront.position(rev) * dia * pi * gR;

    double rightDistance =
        RightFront.position(rev) * dia * pi * gR;

    // Average the distance from both sides.
    distance = (fabs(leftDistance) + fabs(rightDistance)) / 2.0;

    if (printTimer.time(msec) >= 100)
    {
      Brain.Screen.clearScreen();

      Brain.Screen.printAt(10, 30,
                           "LF Rev: %.3f",
                           LeftFront.position(rev));

      Brain.Screen.printAt(10, 60,
                           "RF Rev: %.3f",
                           RightFront.position(rev));

      Brain.Screen.printAt(10, 90,
                           "Distance: %.2f",
                           distance);

      printTimer.clear();
    }

    // Read the robot's current heading from the gyro.
    double currentHeading = Gyro.rotation(degrees);
    double error = currentHeading;

    // Convert the heading error into a motor-speed correction.
    // A larger heading error produces a larger correction.
    double correction = error * DRIVE_KP;

    // Prevent the gyro correction from becoming too large.
    // This keeps one side from being slowed down or sped up excessively.
    if (correction > MAX_CORRECTION)
      correction = MAX_CORRECTION;

    if (correction < -MAX_CORRECTION)
      correction = -MAX_CORRECTION;

    // Variables for the final left and right motor speeds.
    double leftSpeed;
    double rightSpeed;

    // Check whether we are driving forward or backward.
    if (target > 0)
    {
      // Forward driving.
      //
      // The correction changes the two sides in opposite directions
      // to steer the robot back toward its original heading.
      leftSpeed = speed - correction;
      rightSpeed = speed + correction;
    }
    else
    {
      // Backward driving.
      //
      // The correction is reversed appropriately so the gyro
      // still keeps the robot traveling along the original heading.
      leftSpeed = -speed - correction;
      rightSpeed = -speed + correction;
    }

    // Make sure the left speed stays between -100% and +100%.
    if (leftSpeed > 100)
      leftSpeed = 100;

    if (leftSpeed < -100)
      leftSpeed = -100;

    // Make sure the right speed stays between -100% and +100%.
    if (rightSpeed > 100)
      rightSpeed = 100;

    if (rightSpeed < -100)
      rightSpeed = -100;

    // Send the corrected speeds to the left motors.
    LeftFront.spin(forward, leftSpeed, pct);
    LeftBack.spin(forward, leftSpeed, pct);

    // Send the corrected speeds to the right motors.
    RightFront.spin(forward, rightSpeed, pct);
    RightBack.spin(forward, rightSpeed, pct);

    // Wait 10 milliseconds before checking the sensors again.
    // This creates a feedback loop that constantly corrects drifting.
    wait(10, msec);
  }

  // The target distance has been reached.
  // Stop all four motors using the brake setting.
  driveBrake();
}

void gyroTurn(double target)

{
  // Compensation for drivetrain overshoot
  if (target > 0)
    target -= 5;
  else
    target += 5;

  Gyro.setRotation(0, degrees);

  double accuracy = 1.0;

  double maxSpeed = 35;
  double minSpeed = 8;

  double kpFar = 0.7;
  double kpNear = 1.2;

  double error = target;

  while (fabs(error) > accuracy)
  {
    double heading = Gyro.rotation(degrees);
    error = target - heading;

    double speed;

    if (fabs(error) < 25)
      speed = error * kpNear;
    else
      speed = error * kpFar;

    if (speed > maxSpeed)
      speed = maxSpeed;

    if (speed < -maxSpeed)
      speed = -maxSpeed;

    if (fabs(speed) < minSpeed)
    {
      if (speed > 0)
        speed = minSpeed;
      else
        speed = -minSpeed;
    }

    LeftFront.spin(forward, speed, pct);
    LeftBack.spin(forward, speed, pct);

    RightFront.spin(forward, -speed, pct);
    RightBack.spin(forward, -speed, pct);

    wait(10, msec);
  }

  LeftFront.stop(brake);
  LeftBack.stop(brake);
  RightFront.stop(brake);
  RightBack.stop(brake);

  wait(100, msec);
}

void testAprilTags()

{
  while (true)
  {
    Brain.Screen.clearScreen();

    // Check whether the AI Vision Sensor is connected
    if (!AISensor.installed())
    {
      Brain.Screen.setPenColor(red);
      Brain.Screen.printAt(10, 40, "AI VISION NOT CONNECTED");
      Brain.Screen.printAt(10, 70, "Check Smart Port 6");
      wait(500, msec);
      continue;
    }

    Brain.Screen.setPenColor(green);
    Brain.Screen.printAt(10, 40, "AI VISION CONNECTED");

    // Take an AprilTag snapshot
    int count = AISensor.takeSnapshot(aivision::ALL_TAGS);

    Brain.Screen.setPenColor(white);

    Brain.Screen.printAt(10, 80, "Return: %d", count);
    Brain.Screen.printAt(10, 110, "Count: %d", AISensor.objectCount);

    if (AISensor.objectCount > 0 &&
        AISensor.objects[0].exists)
    {
      Brain.Screen.printAt(10, 140,
                           "TAG ID: %d",
                           AISensor.objects[0].id);

      Brain.Screen.printAt(10, 170,
                           "X: %.0f",
                           AISensor.objects[0].centerX);

      Brain.Screen.printAt(10, 200,
                           "Y: %.0f",
                           AISensor.objects[0].centerY);

      Brain.Screen.printAt(10, 230,
                           "W: %.0f H: %.0f",
                           AISensor.objects[0].width,
                           AISensor.objects[0].height);
    }
    else
    {
      Brain.Screen.printAt(10, 140, "NO TAG");
    }

    wait(100, msec);
  }
}

void driveToAprilTag(int targetWidth = 120)
{
  while (true)
  {
    int count = AISensor.takeSnapshot(aivision::ALL_TAGS);

    if (count > 0 && AISensor.objects[0].exists)
    {
      // Use the AprilTag position as the steering input.
      double turn =
          (AISensor.objects[0].centerX - 158) * 0.35;

      // Drive forward toward the tag.
      double throttle = 40;

      // Stop when the tag is close enough.
      if (AISensor.objects[0].width >= targetWidth)
      {
        driveBrake();
        break;
      }

      // Same basic math as arcadeDrive()
      int leftSpeed = throttle + turn;
      int rightSpeed = throttle - turn;

      // Clamp motor speeds
      leftSpeed = fmax(-100, fmin(100, leftSpeed));
      rightSpeed = fmax(-100, fmin(100, rightSpeed));

      LeftFront.spin(forward, leftSpeed, percent);
      LeftBack.spin(forward, leftSpeed, percent);
      RightFront.spin(forward, rightSpeed, percent);
      RightBack.spin(forward, rightSpeed, percent);

      // Display useful information
      Brain.Screen.clearScreen();

      Brain.Screen.printAt(
          10, 30,
          "Tag ID: %d",
          AISensor.objects[0].id);

      Brain.Screen.printAt(
          10, 60,
          "X: %.0f",
          AISensor.objects[0].centerX);

      Brain.Screen.printAt(
          10, 90,
          "Width: %.0f",
          AISensor.objects[0].width);

      Brain.Screen.printAt(
          10, 120,
          "Turn: %.1f",
          turn);

      Brain.Screen.printAt(
          10, 150,
          "L: %d R: %d",
          leftSpeed,
          rightSpeed);
    }
    else
    {
      // No tag — stop instead of driving blindly.
      driveBrake();

      Brain.Screen.clearScreen();
      Brain.Screen.printAt(10, 30, "NO APRILTAG");
    }

    wait(50, msec);
  }
}

void doubleTurn()
{
  inchDrive(5, 80);
  wait(150, msec);
  inchDrive(-7, 100);
  inchDrive(8, 100);
  wait(1000, msec);
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

  // Let the sensor settle after calibration
  wait(500, msec);

  Gyro.setRotation(0, degrees);

  LeftFront.resetPosition();
  RightFront.resetPosition();
  LeftBack.resetPosition();
  RightBack.resetPosition();

  Brain.Screen.clearScreen();

  drawGUI();
  Brain.Screen.pressed(selectAuton);
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
  switch (autonSelect)
  {
  case 0:
    // inchdrive test
    inchDrive(28, 50);
    driveBrake();
    gyroTurn(90);
    wait(2000, msec);
    gyroTurn(-90);
    wait(1000, msec);
    inchDrive(-28, 50);

    break;

  case 1:
    inchDrive(40, 50);
    driveBrake();
    gyroTurn(-45);
    wait(1000, msec);
    inchDrive(25, 50);
    driveBrake();
    gyroTurn(-45);
    wait(1000, msec);
    inchDrive(40, 50);

    break;

    Brain.Screen.printAt(10, 30, "LF rev: %.3f", LeftFront.position(rev));
  case 2:
    driveToAprilTag();
    break;
  }
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

    if (!currentMax)
    {
      if (Controller1.ButtonUp.pressing())
      {
        claw.spin(forward, 75, percent);
      }
      else if (Controller1.ButtonDown.pressing())
      {
        claw.spin(forward, -75, percent);
      }
      else
      {
        claw.stop(brakeType::coast);
      }
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
