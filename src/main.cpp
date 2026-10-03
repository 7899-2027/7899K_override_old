
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

double clawCurrentLimit = 0.6;
bool currentMax = false;

// drive functions
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
    else
    {
      currentMax = false;
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

// display functions
//  Health of the robot functions
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

// basic auton functions

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

// april tag functions
void alignToAprilTag(int targetID, int targetWidth = 80)
{
  // ---------------------------------------------------------
  // SETTINGS
  // ---------------------------------------------------------

  const double cameraCenterX = 158.0;

  // Normal driving
  const double driveSpeed = 35;

  // Rotation speeds
  const double turnKp = 0.45;
  const double maxTurnSpeed = 30;
  const double minTurnSpeed = 6;

  // How accurately the tag must be centered.
  const double finalTolerance = 3.0;

  // Number of consecutive readings that must be good.
  const int requiredStableFrames = 10;

  // Once the tag reaches this width, enter final alignment.
  const double finalAlignmentWidth = targetWidth * 0.75;

  int lostFrames = 0;
  int stableFrames = 0;

  bool finalAlignment = false;

  while (true)
  {
    // ---------------------------------------------------------
    // TAKE VISION SNAPSHOT
    // ---------------------------------------------------------

    int count = AISensor.takeSnapshot(aivision::ALL_TAGS);

    int targetIndex = -1;

    // Find requested AprilTag
    for (int i = 0; i < count; i++)
    {
      if (AISensor.objects[i].exists &&
          AISensor.objects[i].id == targetID)
      {
        targetIndex = i;
        break;
      }
    }

    // ---------------------------------------------------------
    // TAG LOST
    // ---------------------------------------------------------

    if (targetIndex == -1)
    {
      lostFrames++;

      driveBrake();

      stableFrames = 0;

      Brain.Screen.clearScreen();

      Brain.Screen.printAt(10, 30,
                           "TAG %d LOST",
                           targetID);

      Brain.Screen.printAt(10, 60,
                           "Lost Frames: %d",
                           lostFrames);

      if (lostFrames >= 10)
      {
        Brain.Screen.printAt(10, 100,
                             "ALIGNMENT FAILED");

        driveBrake();

        wait(300, msec);

        return;
      }

      wait(50, msec);

      continue;
    }

    lostFrames = 0;

    // ---------------------------------------------------------
    // READ TAG
    // ---------------------------------------------------------

    double tagX =
        AISensor.objects[targetIndex].centerX;

    double tagWidth =
        AISensor.objects[targetIndex].width;

    double error =
        tagX - cameraCenterX;

    // ---------------------------------------------------------
    // ENTER FINAL ALIGNMENT
    // ---------------------------------------------------------

    if (tagWidth >= finalAlignmentWidth)
    {
      finalAlignment = true;
    }

    // =========================================================
    // FINAL ALIGNMENT MODE
    // =========================================================

    if (finalAlignment)
    {
      // We are close enough.
      // Stop driving forward/backward.
      driveBrake();

      // -------------------------------------------------------
      // CHECK ALIGNMENT
      // -------------------------------------------------------

      if (fabs(error) <= finalTolerance)
      {
        stableFrames++;
      }
      else
      {
        stableFrames = 0;
      }

      // -------------------------------------------------------
      // FINISHED
      // -------------------------------------------------------

      if (stableFrames >= requiredStableFrames)
      {
        driveBrake();

        Brain.Screen.clearScreen();

        Brain.Screen.printAt(10, 30,
                             "APRILTAG LOCKED");

        Brain.Screen.printAt(10, 60,
                             "ID: %d",
                             targetID);

        Brain.Screen.printAt(10, 90,
                             "X: %.1f",
                             tagX);

        Brain.Screen.printAt(10, 120,
                             "ERROR: %.1f",
                             error);

        Brain.Screen.printAt(10, 150,
                             "WIDTH: %.1f",
                             tagWidth);

        Brain.Screen.printAt(10, 180,
                             "STABLE: %d",
                             stableFrames);

        wait(300, msec);

        return;
      }

      // -------------------------------------------------------
      // ROTATE IN PLACE
      // -------------------------------------------------------

      double turnSpeed =
          error * turnKp;

      // Clamp
      if (turnSpeed > maxTurnSpeed)
        turnSpeed = maxTurnSpeed;

      if (turnSpeed < -maxTurnSpeed)
        turnSpeed = -maxTurnSpeed;

      // Minimum turning speed so the robot doesn't get stuck.
      if (fabs(turnSpeed) < minTurnSpeed)
      {
        if (turnSpeed > 0)
          turnSpeed = minTurnSpeed;
        else if (turnSpeed < 0)
          turnSpeed = -minTurnSpeed;
        else
          turnSpeed = 0;
      }

      // -------------------------------------------------------
      // ROTATE IN PLACE
      // -------------------------------------------------------

      LeftFront.spin(forward, turnSpeed, pct);
      LeftBack.spin(forward, turnSpeed, pct);

      RightFront.spin(forward, -turnSpeed, pct);
      RightBack.spin(forward, -turnSpeed, pct);

      // -------------------------------------------------------
      // DEBUG
      // -------------------------------------------------------

      Brain.Screen.clearScreen();

      Brain.Screen.printAt(10, 30,
                           "FINAL ALIGNMENT");

      Brain.Screen.printAt(10, 60,
                           "ID: %d",
                           targetID);

      Brain.Screen.printAt(10, 90,
                           "X: %.1f",
                           tagX);

      Brain.Screen.printAt(10, 120,
                           "ERROR: %.1f",
                           error);

      Brain.Screen.printAt(10, 150,
                           "WIDTH: %.1f",
                           tagWidth);

      Brain.Screen.printAt(10, 180,
                           "STABLE: %d/%d",
                           stableFrames,
                           requiredStableFrames);

      wait(30, msec);

      continue;
    }

    // =========================================================
    // NORMAL APPROACH
    // =========================================================

    double correction =
        error * turnKp;

    // Limit steering
    if (correction > 20)
      correction = 20;

    if (correction < -20)
      correction = -20;

    double speed = driveSpeed;

    // Slow down as we approach.
    if (tagWidth > targetWidth * 0.55)
      speed = 25;

    if (tagWidth > targetWidth * 0.65)
      speed = 18;

    // If we're already nearly centered, slow down further.
    if (fabs(error) < 10)
      speed = 15;

    // ---------------------------------------------------------
    // BACKWARD APPROACH
    // ---------------------------------------------------------

    double leftSpeed =
        -speed + correction;

    double rightSpeed =
        -speed - correction;

    leftSpeed =
        fmax(-100, fmin(100, leftSpeed));

    rightSpeed =
        fmax(-100, fmin(100, rightSpeed));

    LeftFront.spin(forward, leftSpeed, pct);
    LeftBack.spin(forward, leftSpeed, pct);

    RightFront.spin(forward, rightSpeed, pct);
    RightBack.spin(forward, rightSpeed, pct);

    // ---------------------------------------------------------
    // DEBUG
    // ---------------------------------------------------------

    Brain.Screen.clearScreen();

    Brain.Screen.printAt(10, 30,
                         "APRILTAG APPROACH");

    Brain.Screen.printAt(10, 60,
                         "ID: %d",
                         targetID);

    Brain.Screen.printAt(10, 90,
                         "X: %.1f",
                         tagX);

    Brain.Screen.printAt(10, 120,
                         "ERROR: %.1f",
                         error);

    Brain.Screen.printAt(10, 150,
                         "WIDTH: %.1f",
                         tagWidth);

    Brain.Screen.printAt(10, 180,
                         "L: %.1f",
                         leftSpeed);

    Brain.Screen.printAt(10, 210,
                         "R: %.1f",
                         rightSpeed);

    wait(30, msec);
  }
}

void debugAprilTag(int targetID)
{
  while (true)
  {
    int count = AISensor.takeSnapshot(aivision::ALL_TAGS);

    if (count > 0)
    {
      int id = AISensor.objects[0].id;
      int x = AISensor.objects[0].centerX;
      int y = AISensor.objects[0].centerY;
      int w = AISensor.objects[0].width;
      int h = AISensor.objects[0].height;

      Brain.Screen.clearScreen();

      Brain.Screen.printAt(10, 30, "ID: %d", id);
      Brain.Screen.printAt(10, 60, "X: %d", x);
      Brain.Screen.printAt(10, 90, "Y: %d", y);
      Brain.Screen.printAt(10, 120, "W: %d", w);
      Brain.Screen.printAt(10, 150, "H: %d", h);
    }

    wait(100, msec);
  }
}

void turnToAprilTag(int targetID, int direction, double speed)
{
  // =========================================================
  // SETTINGS
  // =========================================================

  // Your camera is facing backward, but its image center is
  // still approximately 158 pixels.
  const double cameraCenterX = 158.0;

  // How close the tag must be to the center before we consider
  // the robot aligned.
  const double alignmentTolerance = 6.0;

  // Proportional turning constant.
  const double turnKp = 0.25;

  // Prevent excessively fast turning.
  double maxTurnSpeed = speed;

  // Minimum speed needed to overcome drivetrain friction.
  const double minTurnSpeed = 5.0;

  // Number of consecutive centered readings required.
  const int requiredStableFrames = 5;

  // If the tag disappears this many consecutive snapshots
  // during alignment, return to searching.
  const int maxLostFrames = 5;

  int stableFrames = 0;
  int lostFrames = 0;

  // Make sure direction is valid.
  if (direction != 1 && direction != -1)
  {
    direction = 1;
  }

  // Limit speed.
  if (speed < 5)
  {
    speed = 5;
  }

  if (speed > 50)
  {
    speed = 50;
  }

  maxTurnSpeed = speed;

  // =========================================================
  // MAIN SEARCH / ALIGNMENT LOOP
  // =========================================================

  while (true)
  {
    // -------------------------------------------------------
    // TAKE SNAPSHOT
    // -------------------------------------------------------

    int count = AISensor.takeSnapshot(aivision::ALL_TAGS);

    int targetIndex = -1;

    // Find the requested AprilTag.
    for (int i = 0; i < count; i++)
    {
      if (AISensor.objects[i].exists &&
          AISensor.objects[i].id == targetID)
      {
        targetIndex = i;
        break;
      }
    }

    // =======================================================
    // TAG NOT FOUND
    // =======================================================

    if (targetIndex == -1)
    {
      stableFrames = 0;
      lostFrames++;

      // If we were already close to alignment, don't immediately
      // start spinning wildly because of one bad frame.
      driveBrake();

      Brain.Screen.clearScreen();

      Brain.Screen.printAt(10, 30,
                           "SEARCHING FOR %d",
                           targetID);

      Brain.Screen.printAt(10, 60,
                           "Objects: %d",
                           AISensor.objectCount);

      Brain.Screen.printAt(10, 90,
                           "Lost: %d/%d",
                           lostFrames,
                           maxLostFrames);

      // If we've lost it for several frames, actively search.
      if (lostFrames >= maxLostFrames)
      {
        double searchSpeed = maxTurnSpeed * direction;

        LeftFront.spin(forward, searchSpeed, pct);
        LeftBack.spin(forward, searchSpeed, pct);

        RightFront.spin(forward, -searchSpeed, pct);
        RightBack.spin(forward, -searchSpeed, pct);
      }

      wait(40, msec);
      continue;
    }

    // We found it again.
    lostFrames = 0;

    // =======================================================
    // READ TAG POSITION
    // =======================================================

    double tagX =
        AISensor.objects[targetIndex].centerX;

    // IMPORTANT:
    // The camera is facing backward, so its X direction is
    // reversed relative to the robot.
    //
    // X = 158 means centered.
    //
    // Physical robot-left  -> X gets larger
    // Physical robot-right -> X gets smaller
    double error =
        cameraCenterX - tagX;

    // =======================================================
    // CHECK WHETHER WE ARE CENTERED
    // =======================================================

    if (fabs(error) <= alignmentTolerance)
    {
      // Stop turning while we verify that we're really centered.
      driveBrake();

      stableFrames++;

      Brain.Screen.clearScreen();

      Brain.Screen.printAt(10, 30,
                           "CENTERING TAG %d",
                           targetID);

      Brain.Screen.printAt(10, 60,
                           "X: %.1f",
                           tagX);

      Brain.Screen.printAt(10, 90,
                           "ERROR: %.1f",
                           error);

      Brain.Screen.printAt(10, 120,
                           "STABLE: %d/%d",
                           stableFrames,
                           requiredStableFrames);

      // -----------------------------------------------------
      // FULLY ALIGNED
      // -----------------------------------------------------

      if (stableFrames >= requiredStableFrames)
      {
        driveBrake();

        Brain.Screen.clearScreen();

        Brain.Screen.printAt(10, 30,
                             "APRILTAG ALIGNED!");

        Brain.Screen.printAt(10, 60,
                             "ID: %d",
                             targetID);

        Brain.Screen.printAt(10, 90,
                             "X: %.1f",
                             tagX);

        Brain.Screen.printAt(10, 120,
                             "ERROR: %.1f",
                             error);

        Brain.Screen.printAt(10, 150,
                             "STABLE: %d",
                             stableFrames);

        wait(250, msec);

        // Make absolutely sure the motors remain stopped
        // before returning to autonomous.
        driveBrake();

        return;
      }

      wait(40, msec);
      continue;
    }

    // =======================================================
    // NOT CENTERED
    // =======================================================

    // The tag moved outside our tolerance, so the consecutive
    // centered-frame counter must be reset.
    stableFrames = 0;

    // -------------------------------------------------------
    // PROPORTIONAL TURNING
    // -------------------------------------------------------

    double turnSpeed =
        fabs(error) * turnKp;

    // Maximum speed.
    if (turnSpeed > maxTurnSpeed)
    {
      turnSpeed = maxTurnSpeed;
    }

    // Minimum speed to overcome friction.
    if (turnSpeed < minTurnSpeed)
    {
      turnSpeed = minTurnSpeed;
    }

    // -------------------------------------------------------
    // DETERMINE PHYSICAL TURN DIRECTION
    // -------------------------------------------------------

    // Because the camera is facing backward:
    //
    // error > 0:
    //     Tag is physically to robot RIGHT
    //     Turn robot RIGHT
    //
    // error < 0:
    //     Tag is physically to robot LEFT
    //     Turn robot LEFT

    double signedTurnSpeed;

    if (error > 0)
    {
      signedTurnSpeed = turnSpeed;
    }
    else
    {
      signedTurnSpeed = -turnSpeed;
    }

    // -------------------------------------------------------
    // TURN
    // -------------------------------------------------------

    LeftFront.spin(forward, signedTurnSpeed, pct);
    LeftBack.spin(forward, signedTurnSpeed, pct);

    RightFront.spin(forward, -signedTurnSpeed, pct);
    RightBack.spin(forward, -signedTurnSpeed, pct);

    // -------------------------------------------------------
    // DEBUG
    // -------------------------------------------------------

    Brain.Screen.clearScreen();

    Brain.Screen.printAt(10, 30,
                         "ALIGNING TAG %d",
                         targetID);

    Brain.Screen.printAt(10, 60,
                         "X: %.1f",
                         tagX);

    Brain.Screen.printAt(10, 90,
                         "ERROR: %.1f",
                         error);

    Brain.Screen.printAt(10, 120,
                         "TURN: %.1f",
                         signedTurnSpeed);

    Brain.Screen.printAt(10, 150,
                         "LOST: %d",
                         lostFrames);

    wait(30, msec);
  }
}

void centerAprilTag(int targetID)
{
  // =========================================================
  // CAMERA
  // =========================================================

  const double cameraCenterX = 158.0;

  // Final acceptable alignment.
  const double centerTolerance = 8.0;

  // Larger zone where we slow down BEFORE reaching center.
  const double slowZone = 25.0;

  // Number of consecutive centered readings required.
  const int requiredStableFrames = 6;

  // =========================================================
  // TURN CONTROL
  // =========================================================

  // Far from center = moderate correction.
  const double farKp = 0.10;

  // Close to center = VERY gentle correction.
  const double nearKp = 0.045;

  // Maximum turning speed when far away.
  const double maxFarSpeed = 10.0;

  // Maximum turning speed near the center.
  const double maxNearSpeed = 4.0;

  // Minimum speed needed to actually move the drivetrain.
  const double minTurnSpeed = 2.0;

  // =========================================================
  // LOST TAG
  // =========================================================

  const int maxLostFrames = 5;

  int stableFrames = 0;
  int lostFrames = 0;

  // =========================================================
  // START STOPPED
  // =========================================================

  driveBrake();

  wait(100, msec);

  // =========================================================
  // MAIN LOOP
  // =========================================================

  while (true)
  {
    // -------------------------------------------------------
    // SNAPSHOT
    // -------------------------------------------------------

    int count =
        AISensor.takeSnapshot(aivision::ALL_TAGS);

    int targetIndex = -1;

    for (int i = 0; i < count; i++)
    {
      if (AISensor.objects[i].exists &&
          AISensor.objects[i].id == targetID)
      {
        targetIndex = i;
        break;
      }
    }

    // =======================================================
    // TAG LOST
    // =======================================================

    if (targetIndex == -1)
    {
      lostFrames++;
      stableFrames = 0;

      // NEVER keep rotating when the tag disappears.
      driveBrake();

      Brain.Screen.clearScreen();

      Brain.Screen.printAt(10, 30,
                           "TAG %d LOST",
                           targetID);

      Brain.Screen.printAt(10, 60,
                           "Lost: %d/%d",
                           lostFrames,
                           maxLostFrames);

      if (lostFrames >= maxLostFrames)
      {
        driveBrake();

        Brain.Screen.printAt(10, 100,
                             "ALIGNMENT STOPPED");

        wait(300, msec);

        return;
      }

      wait(50, msec);
      continue;
    }

    lostFrames = 0;

    // =======================================================
    // READ TAG
    // =======================================================

    double tagX =
        AISensor.objects[targetIndex].centerX;

    // Camera faces backward.
    // X increasing = robot LEFT.
    // X decreasing = robot RIGHT.
    //
    // X = 158 = centered.
    double error = tagX - cameraCenterX;

    double absoluteError =
        fabs(error);

    // =======================================================
    // CENTER ZONE
    // =======================================================

    if (absoluteError <= centerTolerance)
    {
      // Completely stop turning.
      driveBrake();

      stableFrames++;

      Brain.Screen.clearScreen();

      Brain.Screen.printAt(10, 30,
                           "CENTER ZONE");

      Brain.Screen.printAt(10, 60,
                           "X: %.1f",
                           tagX);

      Brain.Screen.printAt(10, 90,
                           "Error: %.1f",
                           error);

      Brain.Screen.printAt(10, 120,
                           "Stable: %d/%d",
                           stableFrames,
                           requiredStableFrames);

      // -----------------------------------------------------
      // SUCCESS
      // -----------------------------------------------------

      if (stableFrames >= requiredStableFrames)
      {
        driveBrake();

        Brain.Screen.clearScreen();

        Brain.Screen.printAt(10, 40,
                             "APRILTAG LOCKED");

        Brain.Screen.printAt(10, 80,
                             "ID: %d",
                             targetID);

        Brain.Screen.printAt(10, 120,
                             "X: %.1f",
                             tagX);

        Brain.Screen.printAt(10, 160,
                             "Error: %.1f",
                             error);

        // Final hard stop.
        driveBrake();

        wait(500, msec);

        driveBrake();

        return;
      }

      wait(50, msec);
      continue;
    }

    // Outside the center zone, so we're not stable.
    stableFrames = 0;

    // =======================================================
    // CALCULATE TURN SPEED
    // =======================================================

    double turnSpeed;

    // -------------------------------------------------------
    // FAR FROM CENTER
    // -------------------------------------------------------

    if (absoluteError > slowZone)
    {
      turnSpeed =
          absoluteError * farKp;

      if (turnSpeed > maxFarSpeed)
      {
        turnSpeed = maxFarSpeed;
      }

      if (turnSpeed < minTurnSpeed)
      {
        turnSpeed = minTurnSpeed;
      }
    }

    // -------------------------------------------------------
    // NEAR CENTER
    // -------------------------------------------------------

    else
    {
      turnSpeed =
          absoluteError * nearKp;

      if (turnSpeed > maxNearSpeed)
      {
        turnSpeed = maxNearSpeed;
      }

      // Don't force a minimum speed if we're extremely close.
      //
      // This is important because forcing 2-5% at tiny errors
      // is what causes hunting/oscillation.
      if (turnSpeed < minTurnSpeed)
      {
        turnSpeed = minTurnSpeed;
      }
    }

    // =======================================================
    // FINAL DIRECTION
    // =======================================================

    double signedTurnSpeed;

    if (error > 0)
    {
      // Tag is physically RIGHT.
      signedTurnSpeed = turnSpeed;
    }
    else
    {
      // Tag is physically LEFT.
      signedTurnSpeed = -turnSpeed;
    }

    // =======================================================
    // TURN
    // =======================================================

    LeftFront.spin(forward,
                   signedTurnSpeed,
                   pct);

    LeftBack.spin(forward,
                  signedTurnSpeed,
                  pct);

    RightFront.spin(forward,
                    -signedTurnSpeed,
                    pct);

    RightBack.spin(forward,
                   -signedTurnSpeed,
                   pct);

    // =======================================================
    // DEBUG
    // =======================================================

    Brain.Screen.clearScreen();

    Brain.Screen.printAt(10, 30,
                         "VISION ALIGN");

    Brain.Screen.printAt(10, 60,
                         "ID: %d",
                         targetID);

    Brain.Screen.printAt(10, 90,
                         "X: %.1f",
                         tagX);

    Brain.Screen.printAt(10, 120,
                         "Error: %.1f",
                         error);

    Brain.Screen.printAt(10, 150,
                         "Turn: %.1f",
                         signedTurnSpeed);

    if (absoluteError > slowZone)
    {
      Brain.Screen.printAt(10, 180,
                           "ZONE: FAR");
    }
    else
    {
      Brain.Screen.printAt(10, 180,
                           "ZONE: SLOW");
    }

    wait(50, msec);
  }
}

void gyroThenVision(double angle, int targetID)
{
  gyroTurn(angle);
  wait(100, msec);
  centerAprilTag(targetID);
}

void doubleTurn()
{
  inchDrive(5, 20);
  wait(150, msec);
  inchDrive(-3, 100);
  driveBrake();
  wait(100, msec);
  inchDrive(8, 20);
  wait(500, msec);
}

void brakeNwait(int waitTime = 500)
{
  driveBrake();
  wait(waitTime, msec);
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
    // jayden test
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
  case 2:
    gyroThenVision(-90, 3);
    break;
  case 3:
    // first auton using 1st full asian strat
    doubleTurn();
    inchDrive(-14, 50);
    brakeNwait(500);
    gyroTurn(-91);
    brakeNwait(500);
    inchDrive(-13, 40);
    alignToAprilTag(3, 80);
    // score
    brakeNwait(500);
    inchDrive(14, 40);
    brakeNwait(500);
    gyroTurn(-40);
    inchDrive(-14, 40);
    // at wall
    brakeNwait(500);
    gyroTurn(40);
    brakeNwait(500);
    inchDrive(-6, 40);
    brakeNwait();
    gyroTurn(-90);
    inchDrive(-4, 40);
    brakeNwait(500);
    inchDrive(2, 30);
    brakeNwait(500);
    // turn around
    gyroTurn(180);
    brakeNwait(500);
    inchDrive(-7, 30);
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

  // Set up competition callbacks
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run pre-autonomous setup
  pre_auton();

  // Keep main alive
  while (true)
  {
    wait(100, msec);
  }
}
