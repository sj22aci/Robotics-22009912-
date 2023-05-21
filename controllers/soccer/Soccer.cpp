#include "Soccer.hpp"
#include <RobotisOp2GaitManager.hpp>
#include <RobotisOp2MotionManager.hpp>
#include <RobotisOp2VisionManager.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/Camera.hpp>
#include <webots/Gyro.hpp>
#include <webots/LED.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Speaker.hpp>

#include <cassert>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>

using namespace webots;
using namespace managers;
using namespace std;

static double clamp(double value, double min, double max) {
  if (min > max) {
    assert(0);
    return value;
  }
  return value < min ? min : value > max ? max : value;
}

static double minMotorPositions[NMOTORS];
static double maxMotorPositions[NMOTORS];

static const char *motorNames[NMOTORS] = {
  "ShoulderR" /*ID1 */, "ShoulderL" /*ID2 */, "ArmUpperR" /*ID3 */, "ArmUpperL" /*ID4 */, "ArmLowerR" /*ID5 */,
  "ArmLowerL" /*ID6 */, "PelvYR" /*ID7 */,    "PelvYL" /*ID8 */,    "PelvR" /*ID9 */,     "PelvL" /*ID10*/,
  "LegUpperR" /*ID11*/, "LegUpperL" /*ID12*/, "LegLowerR" /*ID13*/, "LegLowerL" /*ID14*/, "AnkleR" /*ID15*/,
  "AnkleL" /*ID16*/,    "FootR" /*ID17*/,     "FootL" /*ID18*/,     "Neck" /*ID19*/,      "Head" /*ID20*/
};

Soccer::Soccer() : Robot() {
  mTimeStep = getBasicTimeStep();

  mEyeLED = getLED("EyeLed");
  mHeadLED = getLED("HeadLed");
  mHeadLED->set(0x00FF00);
  mBackLedRed = getLED("BackLedRed");
  mBackLedGreen = getLED("BackLedGreen");
  mBackLedBlue = getLED("BackLedBlue");
  mCamera = getCamera("Camera");
  mCamera->enable(2 * mTimeStep);
  mAccelerometer = getAccelerometer("Accelerometer");
  mAccelerometer->enable(mTimeStep);
  mGyro = getGyro("Gyro");
  mGyro->enable(mTimeStep);
  mSpeaker = getSpeaker("Speaker");

  for (int i = 0; i < NMOTORS; i++) {
    mMotors[i] = getMotor(motorNames[i]);
    string sensorName = motorNames[i];
    sensorName.push_back('S');
    mPositionSensors[i] = getPositionSensor(sensorName);
    mPositionSensors[i]->enable(mTimeStep);
    minMotorPositions[i] = mMotors[i]->getMinPosition();
    maxMotorPositions[i] = mMotors[i]->getMaxPosition();
  }

  mMotionManager = new RobotisOp2MotionManager(this);
  mGaitManager = new RobotisOp2GaitManager(this, "config.ini");
  mVisionManager = new RobotisOp2VisionManager(mCamera->getWidth(), mCamera->getHeight(), 240, 15, 60, 15, 0, 30);
}

Soccer::~Soccer() {
}

void Soccer::myStep() {
  int ret = step(mTimeStep);
  if (ret == -1)
    exit(EXIT_SUCCESS);
}

void Soccer::wait(int ms) {
  double startTime = getTime();
  double s = (double)ms / 1000.0;
  while (s + startTime >= getTime())
    myStep();
}

// Detect Ball from ball color using the Vision Manager
bool Soccer::getBallCenter(double &x, double &y) {
  static int width = mCamera->getWidth();
  static int height = mCamera->getHeight();

  const unsigned char *im = mCamera->getImage();
  bool find = mVisionManager->getBallCenter(x, y, im);

  if (!find) {
    x = 0.0;
    y = 0.0;
    return false;
  } else {
    x = 2.0 * x / width - 1.0;
    y = 2.0 * y / height - 1.0;
    return true;
  }
}

// Main feedback loop
void Soccer::run() {

  //update sensors
  myStep();

  //eye color to purple
  mEyeLED->set(0x800080);

  //prepare to walk
  mGaitManager->start();
  mMotionManager->playPage(9);
  mGaitManager->step(mTimeStep);

  // main loop
  double px = 0.0;
  double py = 0.0;

  while (true) {
    double x, y, neckPosition, headPosition;
    bool ballInFieldOfView = getBallCenter(x, y);

    if (ballInFieldOfView) {
      // set eye led to blue
      mEyeLED->set(0x0000FF);

      // compute the direction of the head
      x = 0.015 * x + px;
      y = 0.015 * y + py;
      px = x;
      py = y;
      neckPosition = clamp(-x, minMotorPositions[18], maxMotorPositions[18]);
      headPosition = clamp(-y, minMotorPositions[19], maxMotorPositions[19]);

      // go forwards and turn according to the head rotation
      if (y < 0.1)  // ball far away, go quickly
        mGaitManager->setXAmplitude(1.0);
      else  // ball close, go slowly
        mGaitManager->setXAmplitude(0.5);
      mGaitManager->setAAmplitude(neckPosition);
      mGaitManager->step(mTimeStep);

      // Move head
      mMotors[18]->setPosition(neckPosition);
      mMotors[19]->setPosition(headPosition);

      // if the ball is close enough
      // kick the ball with the right foot
      if (y > 0.35) {
        mGaitManager->stop();
        wait(500);
        // set eye led to green
        mEyeLED->set(0x00FF00);
        if (x < 0.0)
          mMotionManager->playPage(12);  // right kick
        else
          mMotionManager->playPage(13);  // left kick
          
        mMotionManager->playPage(237);  //Waits for goal
        mMotionManager->playPage(27);   // Celebration 1
        mMotionManager->playPage(17);   // Celebration 2
      }
    }

    // the ball is not in the field of view,
    // move vertically the head and stop
    else {
      // set eye led to red
      mEyeLED->set(0xFF0000);
    }

    // step
    myStep();
  }
}
