/*******************************************************************************
  Tutorial.ino - This is a MotionSensor example sketch which shows how to use it
                 and how to use Quaternions for robust and efficient motion
                 processing

  Copyright(C) 2022  Howard James May

  This file is part of the SweetMaker SDK

  The SweetMaker SDK is free software: you can redistribute it and / or
  modify it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  The SweetMaker SDK is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.If not, see <http://www.gnu.org/licenses/>.

  Contact me at sweet.maker@outlook.com

********************************************************************************
Release     Date                        Change Description
--------|-------------|--------------------------------------------------------|
   1      07-Mar-2022   Initial release
*******************************************************************************/

#include <Wire.h>
#include <SweetMaker.h>
#include <MotionSensor.h>

using namespace SweetMaker;

/*
 * Motion Sensor instance for our use.
 */
MotionSensor motionSensor;

/*
 * Function prototypes used in this sketch
 */
void myEventHandlerCallback(uint16_t event, uint8_t source, uint16_t value);
void handleSerialInput();

/*
 * RunModes used in this tutorial. These correspond to different outputs 
 */
enum RunMode { 
  IDLE = 0, 
  QUATERNION = 1,
  YAW_PITCH_ROLL = 2,
  GRAVITY = 3,
  TILT_X_Y = 4,
  TILT_X_Y_NON_TRIG = 5,
  ROTATIONAL_SPEED = 6,
  VECTOR_ANGLE = 7,
} runMode = IDLE;

/*
 * Arduino framework setup function - called once at system start
 */
void setup()
{
  Serial.begin(115200); // Serial connection
  Serial.println("Motion Sensor & Quaternion Tutorial");

  /*
   * Register eventHandler with SweetMaker Framework
   */
  EventMngr::getMngr()->configCallBack(myEventHandlerCallback);

  /*
   * Initialise motionSensor - this establishes comms over I2C with
   * the MPU6050 and downloads code to the MPU for motion processing
   * allowing generation of rotation quaternion. 
   */
  if(motionSensor.init() != 0) {
    // try again
  }

  /*
   * While this sketch doesn't use TimerTick events we give it a kick
   * to start the generation of events.
   */
  TimerTickMngt::getTimerMngt()->update(0);

  Serial.println("Setup complete");
}

/*
 * Arduino framework main loop
 *
 * This function does little more than:
 *     1) Calculate loop elapsed time,
 *     2) Watch for input on the serial port 
 *     3) Call SweetMaker update function
 *
 * The Motion Sensor is part of the SweetMaker framework and inherits from
 * AutoUpdate - this integrates it with the Auto Update process which 
 * triggers the checking for new sensor readings.
 *
 */
void loop()
{
  static unsigned long lastUpdateTime_ms = millis();
  unsigned long thisTime_ms = millis();
  unsigned long elapsedTime_ms = thisTime_ms - lastUpdateTime_ms;

  handleSerialInput();

  AutoUpdateMngr::getUpdater()->update(elapsedTime_ms);

  lastUpdateTime_ms = thisTime_ms;
}

/*
 * myEventHandlerCallback: This captures events from the SweetMaker Framework
 *                         we are only interested in:
 *                           - MOTION_SENSOR_NEW_SMPL_RDY
 *                           - MOTION_SENSOR_INIT_ERROR
 *
 * This function switches on runmode state and demonstrates different motion processing
 * accordingly. The output is sent to the Serial port for the Arduino Serial Plotter.
 */
void myEventHandlerCallback(uint16_t event, uint8_t source, uint16_t value)
{
  switch (event)
  {
  case MotionSensor::MOTION_SENSOR_NEW_SMPL_RDY:
    switch (runMode) {

      /*
       * The MPU6050 returns a Rotational Quaternion which represents how the sensor has been rotated.
       * See the following link for a full explanation.
       * https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation
       * As the sensor moves the Rotational Quaternion updates accordingly.
       *
       * Note: the rotational quaternion doesn't immediately say how a particular vector
       * has changed - this must be calculated using the Rotational Quaternion
       *
       * Also: the MPU6050 doesn't have a magnetometer and thus no concept of true North,
       * consequently it's hortizontal orientation is arbitrary and will drift
       *
       * The Rotational Quaternion indicates the amount of rotation (r) about an axis (x,y,z)
       *
       * The MPU6050 provides this as 14 bit signed integer values +/-16384 
       *
       * These values have little use on their own without further processing.
       */
    case QUATERNION:
    {
      Serial.print("r:"); Serial.print(motionSensor.rotQuat.r); Serial.print("\t");
      Serial.print("x:"); Serial.print(motionSensor.rotQuat.x); Serial.print("\t");
      Serial.print("y:"); Serial.print(motionSensor.rotQuat.y); Serial.print("\t");
      Serial.print("z:"); Serial.print(motionSensor.rotQuat.z); Serial.println();
    }
    break;

    /*
     * One common representation of 3D orientation is Yaw Pitch Roll. 
     *     - Yaw is horizontal orientation - rotation about Z axis
     *     - Pitch is forward backward tilt - rotation about Y axis    
     *     - Roll is side to side tilt - rotation about X axis
     *
     * These values can be calculated by MotionSensor from the Rotational Quaternion.
     * Note that defining pitch as rotation about Y is arbitrary etc.
     * Also note that this uses trigonometric functions to calculate and so takes a 
     * little time to compute. This may be fine for many applications.
     *
     * Reminder that Yaw will drift as the MPU6050 has no magnetometer
     */
    case YAW_PITCH_ROLL:
    {
      unsigned long start_us = micros();
      int16_t yaw = motionSensor.rotQuat.getRotZ();
      int16_t pitch = motionSensor.rotQuat.getRotY();
      int16_t roll = motionSensor.rotQuat.getRotX();
      unsigned long stop_us = micros();

      Serial.print(stop_us - start_us); Serial.print("\t");
      Serial.print(yaw); Serial.print("\t");
      Serial.print(pitch); Serial.print("\t");
      Serial.print(roll); Serial.println();
    }
    break;

    /*
     * The MPU6050 is great for calculating Gravity and the lack of
     * magnetometer is no limitation for this. MotionSensor supports
     * returning Gravity and this has small computational expense 
     * using no trigonometric functions.
     *
     */
    case GRAVITY:
    {
      Quaternion_16384 gravity;
      unsigned long start_us = micros();
      motionSensor.rotQuat.getGravity(&gravity);
      unsigned long stop_us = micros();

      Serial.print(stop_us - start_us); Serial.print("\t");
      Serial.print(gravity.x); Serial.print("\t");
      Serial.print(gravity.y); Serial.print("\t");
      Serial.print(gravity.z); Serial.println();
    }
    break;

    /*
     * As previously shown Tilt X and Tilt Y (Pitch and Roll)
     * can be produced.
     */
    case TILT_X_Y:
    {
      unsigned long start_us = micros();
      int16_t tilt_forward = motionSensor.rotQuat.getRotX();
      int16_t tilt_side = motionSensor.rotQuat.getRotY();
      unsigned long stop_us = micros();

      Serial.print(stop_us - start_us); Serial.print("\t");
      Serial.print(tilt_forward); Serial.print("\t");
      Serial.print(tilt_side); Serial.println("\t0");
    }
    break;

    /*
     * In order to avoid the use of trigonometric functions it may be
     * adequate to use the Sin of RotX and Sin RotY. This can be calculated
     * using only multiplcations and right shifts.
     */
    case TILT_X_Y_NON_TRIG:
    {
      unsigned long start_us = micros();
      int16_t tilt_forward = motionSensor.rotQuat.getSinRotX();
      int16_t tilt_side = motionSensor.rotQuat.getSinRotY();
      unsigned long stop_us = micros();

      Serial.print(stop_us - start_us); Serial.print("\t");
      Serial.print(tilt_forward); Serial.print("\t");
      Serial.print(tilt_side); Serial.println("\t0");
    }
    break;

    /*
     * The rotational quaternion produced by the MPU6050 can also
     * provide an indication of rotational speed by looking at the 
     * rotational quaternion delta and the angular part of that.
     *
     * The angular part is equal to cos(theta/2), theta being the 
     * angular rotation between the two last readings. As the sample period
     * is a fraction of a second, theta is likely to be small and cos(theta/2)
     * smaller still. The value should be handled accordingly.
     *
     * Note: the rotation speed here is not about any one particular axis rather
     * it is about any arbitraty axis.
     */
    case ROTATIONAL_SPEED:
    {
      int16_t rot_speed = 16384 - motionSensor.rotQuatDelta.r;
      Serial.print(rot_speed); Serial.println("\t0\t0\t0");
    }
    break;

    /*
     * One of the most powerful uses of the quaternion is finding 
     * how close the sensor is to pointing in a particular direction.
     * The dot product between two positional quaternions tells us the 
     * cosine of the angle between the two vectors in 3D space. This 
     * is calculated without the use of trigonometric functions.
     *
     * As we have seen the MPU6050 provides a robust calculation of where the 
     * Z axis (gravity) is now pointing. Thus we choose to compare gravity 
     * against vectors we are interested in.
     *
     */
    case VECTOR_ANGLE:
    {
      Quaternion_16384 gq;
      motionSensor.rotQuat.getGravity(&gq);

      Quaternion_16384 z_axis = { 0, 0, 0, 0x4000 };
      Quaternion_16384 forward_right = { 0,9459,9459,9459 };
      Quaternion_16384 forward_left = { 0,9459,-9459,9459 };

      int16_t up = z_axis.dotProduct(&gq);
      int16_t fr = forward_right.dotProduct(&gq);
      int16_t fl = forward_left.dotProduct(&gq);

      Serial.print(up); Serial.print("\t");
      Serial.print(fr); Serial.print("\t");
      Serial.print(fl); Serial.println("\t0");
    }
    break;

    default:
      break;
    }
    break;

  case MotionSensor::MOTION_SENSOR_INIT_ERROR:
    Serial.print("Motion Sensor Init Error: "); Serial.println(value);
    break;

  default:
    break;
  }

}


void handleSerialInput()
{
  if (!Serial.available())
    return;

  char inChar = Serial.read();

  Serial.println("_:0\t_:0\t_:0\t_:0");

  switch (inChar)
  {

  case '0':
    runMode = IDLE;
    break;

  case '1':
    Serial.println("r x y z");
    runMode = QUATERNION;
    break;

  case '2':
    Serial.println("time_us yaw pitch roll");
    runMode = YAW_PITCH_ROLL;
    break;

  case '3':
    Serial.println("time_us x y z");
    runMode = GRAVITY;
    break;

  case '4':
    Serial.println("time_us tilt_forward tilt_side");
    runMode = TILT_X_Y;
    break;

  case '5':
    Serial.println("time_us tilt_forward tilt_side");
    runMode = TILT_X_Y_NON_TRIG;
    break;

  case '6':
    Serial.println("rotational_speed");
    runMode = ROTATIONAL_SPEED;
    break;

  case '7':
    Serial.println("z_axis forward_right forward_left");
    runMode = VECTOR_ANGLE;
    break;

  case 'c':
  {
    /*
     * Each MPU6050 requires it's own set of calibration values
     * These can be calculated by MotionSensor and these values
     * submited in the motionSensor.init() function.
     *
     * You may choose to store calibration values in EEPROM so 
     * so that they can be retrieved at system start without 
     * hardcoding them into the sketch (which only works if you
     * are using a single MPU6050). See the SweetMaker StrawberryString
     * which does just this.
     */
    Serial.println("Calibrating Motion Sensor");
    MotionSensor::CALIBRATION calibration;
    motionSensor.runSelfCalibrate(&calibration);
  }
  break;


  /*
   * The MPU6050 may not be mounted 100% flat or may be mounted in 
   * some arbitrary orientation and an offset applied to make it
   * work as if it were horizontal. This can be done using the 
   * MotionSensor auto level feature. This takes advantage of the 
   * RotationalQuaternions ability to easily combine rotations.
   *
   * This works by finding the rotation beteen gravity and the Z axis
   * and using this as an offset.
   *
   * https://stackoverflow.com/questions/1171849/finding-quaternion-representing-the-rotation-from-one-vector-to-another
   */
  case 'l':
    Serial.println("Self Leveling");
    motionSensor.autoLevel();
    break;

    /*
     * AutoLeveling can compensate for non-horizontal mounting
     * but a rotation about Z requires specific configuration.
     * Here we autolevel and also apply a 45 degree rotation offset.
     */
  case 'z':
  {
    Serial.println("Self Leveling + Z Rotation of 45 degrees");

    float rotation_z = 45;

    Quaternion_16384 gravity;
    RotationQuaternion_16384 offsetQ;

    RotationQuaternion_16384 rotZ(rotation_z, 0, 0, 16384);
    Quaternion_16384 zAxis(0, 0, 0, 16384);

    motionSensor.clearOffsetRotation();
    motionSensor.rotQuat.getGravity(&gravity);

    offsetQ.findOffsetRotation(&zAxis, &gravity);
    offsetQ.crossProduct(&rotZ);

    motionSensor.setOffsetRotation(&offsetQ);
  }
  break;

  default:
    break;
  }
}


