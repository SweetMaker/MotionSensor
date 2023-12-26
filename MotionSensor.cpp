/*******************************************************************************
MotionSensor.cpp takes an mpu6050 class and manages configuration and control
                 while integrating it into the SweetMaker framework. Presents 
				 the output from the sensor as a SM::Quaternion_16384

Copyright(C) 2017-2021  Howard James May

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
1      13-Sep-2017   Initial release
2      03-May-2019   Major updates including:
                     - addition of calibration routine
					           - use of SM::Quaternion_16384 for processing
					           - addition of rotation offset
3      07-Mar-2021   Fixed calibration routine
                     - fixed rotation offset / autoLevel
*******************************************************************************/

#include <Arduino.h>
#include "MotionSensor.h"
#include "EventMngr.h"
#include "SM_MPU6050_6Axis_MotionApps20.h"

using namespace SweetMaker;

MotionSensor::MotionSensor()
{
};

/*
 * init - sets to known values
 */
int MotionSensor::init()
{
	CALIBRATION cal;
	cal.accelXoffset = 0;
	cal.accelYoffset = 0;
	cal.accelZoffset = 4096;
	cal.gyroXoffset = 0;
	cal.gyroYoffset = 0;
	cal.gyroZoffset = 0;

	return (init(&cal));
}

int MotionSensor::init(CALIBRATION * calibration)
{
  Serial.println("MotionSensor::init");

  uint8_t retVal;
	/*
	 * Start the Wire library - used to communicate with MPU6050
	 */
#ifdef ARDUINO_ARCH_AVR
	Wire.begin();
	TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#else 
    #ifdef ARDUINO_ARCH_ESP32
        Serial.println("MotionSensor::Wire begin");

        Wire.begin(21, 22, 400000L);
    #else
        #pragma message ( "MotionSensor supports either AVR or ESP32 architecures" )
    #endif
#endif

  Serial.println("MotionSensor::testConnection");
 
	// verify connection
	if (mpu6050.testConnection() != true) {
		Serial.println("MotionSensor::testConnectionFailure");
		EventMngr::getMngr()->handleEvent(MotionSensor::MOTION_SENSOR_INIT_ERROR, 0, 3);
		return -1;
	}

	Serial.println("Have Connected to Motion Sensor");

/*
 * Initialize MPU6050 Chip - returns void
 */
  mpu6050.initialize();

  // Configure DMP processor
	retVal = mpu6050.dmpInitialize();
	if (retVal != 0) {
		// 1 = initial memory load failed
		// 2 = DMP configuration updates failed
		EventMngr::getMngr()->handleEvent(MotionSensor::MOTION_SENSOR_INIT_ERROR, 0, retVal);
		return -1;
	}

	// Set Offsets for this chip instance
	setCalibration(calibration);

	mpu6050.setDMPEnabled(true);

  /* The Delta Quarternion is initialised thus with a zero real part */
  rotQuatDelta.r = 0;
  rotQuatDelta.x = 0;
  rotQuatDelta.y = 0;
  rotQuatDelta.z = 16384;

  /*
   * Set gravity to sane, though incorrect value!
   */
  gravity.r = 0;
  gravity.x = 0;
  gravity.y = 0;
  gravity.z = 16384;

  /*
   * Send indication that sensor is now ready
   */
	eventHandler = EventMngr::getMngr();
	eventHandler->handleEvent(MotionSensor::MOTION_SENSOR_READY, 0, 0);

	return 0;
}

/*
 * configEventHandler - sets the callback for generated events.
 *                      Consider registering your eventHandler with 
 *                      the EventMngr:: instead
 */
void MotionSensor::configEventHandler(IEventHandler *eh)
{
	eventHandler = eh;
	if(eventHandler == NULL)
		eventHandler = EventMngr::getMngr();
}


/*
 * Called repeatedly by updater - allows motionSensor to process Fifo
 * and generate new sample events
 */
void MotionSensor::update(uint16_t elapsedTime_ms)
{
	/*
	 * Check there is a complete sample of data waiting for us
	 */
	uint16_t fifoCount = mpu6050.getFIFOCount();
	if (fifoCount < MPU6050::dmpPacketSize) {
		return;
	}

	/*
	 * Check the mpu6050's buffer hasn't overflowed because we haven't been reading
	 * it quick enough, or have stalled for some reason.
	 */
	if (fifoCount == 1024) {
		// reset so we can continue cleanly
		mpu6050.resetFIFO();
		eventHandler->handleEvent(MotionSensor::MOTION_SENSOR_RUNTIME_ERROR, 0, 0);
		return;
	}

	/*
	 * read a packet from FIFO and extract readings
	 */
	uint8_t fifoBuffer[MPU6050::dmpPacketSize];
	mpu6050.getFIFOBytes(fifoBuffer, MPU6050::dmpPacketSize);

	int16_t raw_quarternion[4];

	mpu6050.dmpGetQuaternion(raw_quarternion, fifoBuffer);

  /*
   * The MPU6050 returns a rotational quaternion
   */
  RotationQuaternion_16384 newRq;

  newRq.r = raw_quarternion[0];
  newRq.x = raw_quarternion[1];
  newRq.y = raw_quarternion[2];
  newRq.z = raw_quarternion[3];

  /*
   * If there is an offsetRotation configured then this is 
   * applied using a crossProduct
   */
  if (offsetRotation != NULL) {
    newRq.crossProduct(offsetRotation);
  }

  /*
   * Delta is calculated by taking the conjugate of the old rotation 
   * and removing if from the new. This is effectively a subtraction
   */
  rotQuatDelta.r = newRq.r;
  rotQuatDelta.x = newRq.x;
  rotQuatDelta.y = newRq.y;
  rotQuatDelta.z = newRq.z;

  rotQuat.conjugate();
  rotQuatDelta.crossProduct(&rotQuat);
  
  rotQuat.r = newRq.r;
  rotQuat.x = newRq.x;
  rotQuat.y = newRq.y;
  rotQuat.z = newRq.z;

  /*
   * Now calculate gravity
   */
  rotQuat.getGravity(&gravity);

	/*
	 * Notify system a new sample is available
	 */
	if(eventHandler != NULL)
	  eventHandler->handleEvent(MotionSensor::MOTION_SENSOR_NEW_SMPL_RDY, 0, 0);

	return;
}

/*
 * readingAvailable - checks whether a new reading is available from the MPU
 */
bool MotionSensor::readingAvailable()
{
	if (mpu6050.getFIFOCount() >= mpu6050.dmpPacketSize) {
		return (true);
	}
	return (false);
}

/*
 * takeSamples simply gets lots of sample values, sums them and returns the
 * average - this is used as part of calibration
 */
void MotionSensor::takeSamples(SAMPLE_AVGS * sampleAvgs, uint16_t numSamples) {
	long sum_ax = 0, sum_ay = 0, sum_az = 0, sum_gx = 0, sum_gy = 0, sum_gz = 0;
	int16_t ax, ay, az, gx, gy, gz;

	for (uint16_t i = 0; i < numSamples; i++) {
		// read raw accel/gyro measurements from device
		while (!mpu6050.dmpPacketAvailable())
			;
		mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

		sum_ax += ax;
		sum_ay += ay;
		sum_az += az;
		sum_gx += gx;
		sum_gy += gy;
		sum_gz += gz;
	}

	sampleAvgs->accelXAvg = sum_ax / numSamples;
	sampleAvgs->accelYAvg = sum_ay / numSamples;
	sampleAvgs->accelZAvg = sum_az / numSamples;
	sampleAvgs->gyroXAvg = sum_gx / numSamples;
	sampleAvgs->gyroYAvg = sum_gy / numSamples;
	sampleAvgs->gyroZAvg = sum_gz / numSamples;
}

/*
 * runSelfCalibrate - iteratively conveges on a set of calibration 
 *                    values by trying values and slowly modifying them
 */
int MotionSensor::runSelfCalibrate(CALIBRATION * calibration) {
	SAMPLE_AVGS sample_avgs;
	const uint16_t num_samples = 1000;     //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)


	calibration->accelXoffset = 0;
	calibration->accelYoffset = 0;
	calibration->accelZoffset = 0;
	calibration->gyroXoffset = 0;
	calibration->gyroYoffset = 0;
	calibration->gyroZoffset = 0;

	setCalibration(calibration);

	/*
	* Delay 10 seconds by taking 1000 samples.
	* This is to ensure sensor has settled down
	*/
	for (uint16_t i = 0; i < 1000; i++) {
		int16_t ax, ay, az, gx, gy, gz;
		while (!mpu6050.dmpPacketAvailable())
			;
		mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
	}

	Serial.println("Initial Samples");

	takeSamples(&sample_avgs, num_samples);

	calibration->accelXoffset = -sample_avgs.accelXAvg / 8;
	calibration->accelYoffset = -sample_avgs.accelYAvg / 8;
	calibration->accelZoffset = (16384 - sample_avgs.accelZAvg) / 8;

	calibration->gyroXoffset = -sample_avgs.gyroXAvg / 4;
	calibration->gyroYoffset = -sample_avgs.gyroYAvg / 4;
	calibration->gyroZoffset = -sample_avgs.gyroZAvg / 4;

	bool finished = false;
	while (!finished) {
		const int16_t acel_deadzone = 8;     //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
		const int16_t giro_deadzone = 1;     //Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)

		setCalibration(calibration);

		Serial.print(calibration->accelXoffset);
		Serial.print("\t");
		Serial.print(calibration->accelYoffset);
		Serial.print("\t");
		Serial.print(calibration->accelZoffset);
		Serial.print("\t");
		Serial.print(calibration->gyroXoffset);
		Serial.print("\t");
		Serial.print(calibration->gyroYoffset);
		Serial.print("\t");
		Serial.print(calibration->gyroZoffset);
		Serial.print("\t");
		Serial.println("\t");

		Serial.println("Taking More Samples");

		takeSamples(&sample_avgs, num_samples);

		Serial.print(sample_avgs.accelXAvg);
		Serial.print("\t");
		Serial.print(sample_avgs.accelYAvg);
		Serial.print("\t");
		Serial.print(sample_avgs.accelZAvg);
		Serial.print("\t");
		Serial.print(sample_avgs.gyroXAvg);
		Serial.print("\t");
		Serial.print(sample_avgs.gyroYAvg);
		Serial.print("\t");
		Serial.print(sample_avgs.gyroZAvg);
		Serial.print("\t");
		Serial.println("\t");

		finished = true;
		if (abs(sample_avgs.accelXAvg) > acel_deadzone) {
			finished = false;
			calibration->accelXoffset -= sample_avgs.accelXAvg / acel_deadzone;
		}

		if (abs(sample_avgs.accelYAvg) > acel_deadzone) {
			finished = false;
			calibration->accelYoffset -= sample_avgs.accelYAvg / acel_deadzone;
		}

		if (abs(16384 - sample_avgs.accelZAvg) > acel_deadzone) {
			finished = false;
			calibration->accelZoffset += (16384 - sample_avgs.accelZAvg) / acel_deadzone;
		}

		if (abs(sample_avgs.gyroXAvg) > giro_deadzone) {
			finished = false;
			calibration->gyroXoffset -= sample_avgs.gyroXAvg / (giro_deadzone + 1);
		}

		if (abs(sample_avgs.gyroYAvg) > giro_deadzone) {
			finished = false;
			calibration->gyroYoffset -= sample_avgs.gyroYAvg / (giro_deadzone + 1);
		}

		if (abs(sample_avgs.gyroZAvg) > giro_deadzone) {
			finished = false;
			calibration->gyroZoffset -= sample_avgs.gyroZAvg / (giro_deadzone + 1);
		}
	}
	return(0);
}

/*
 * autoLevel - this finds the rotationOffset needed to adjust an MPU6050
 *             which is not level (with Z upwards) so that it behaves as if it is.
 *
 *             This is achieved by finding gravity and calculating the rotation
 *             from this to the Z axis.
 */
void MotionSensor::autoLevel()
{
  if (offsetRotation)
    clearOffsetRotation();

  RotationQuaternion_16384 offsetQ;
  Quaternion_16384 zAxis(0,0,0,16384);
  Quaternion_16384 gq;

  rotQuat.getGravity(&gq);
  offsetQ.findOffsetRotation(&zAxis, &gq);

  setOffsetRotation(&offsetQ);
}

/*
 * setOffsetRotation - configures new offset rotation to given value
 *                     this also updates current rotation
 */
void MotionSensor::setOffsetRotation(RotationQuaternion_16384 * input)
{
	if (offsetRotation)
		clearOffsetRotation();

	offsetRotation = new RotationQuaternion_16384(input);
	rotQuat.crossProduct(offsetRotation);
}

/*
 * clearOffsetRotation - clears offset rotation
 *                       also clears current offset
 */
void MotionSensor::clearOffsetRotation()
{
	if (offsetRotation)	{
		offsetRotation->conjugate();
		rotQuat.crossProduct(offsetRotation);
		delete offsetRotation;
		offsetRotation = NULL;
	}
}

/*
 * setCalibration - sets calibration values in MPU6050
 */
void MotionSensor::setCalibration(CALIBRATION * calibration)
{
	// Set Offsets for this chip instance
	mpu6050.setXGyroOffset(calibration->gyroXoffset);
	mpu6050.setYGyroOffset(calibration->gyroYoffset);
	mpu6050.setZGyroOffset(calibration->gyroZoffset);

	mpu6050.setXAccelOffset(calibration->accelXoffset);
	mpu6050.setYAccelOffset(calibration->accelYoffset);
	mpu6050.setZAccelOffset(calibration->accelZoffset);
}


