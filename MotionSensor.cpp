/*******************************************************************************
MotionSensor.cpp takes an mpu6050 class and manages configuration and control
                 while integrating it into the SweetMaker framework. Presents 
				 the output from the sensor as a SM::Quaternion_16384

Copyright(C) 2017-2024  Howard James May

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
4      17-Apr-2024   - Significant Refactoring
                     - added rotation about Z offset
					 - added linear acceleration
*******************************************************************************/

#include <Arduino.h>
#include "MotionSensor.h"
#include "EventMngr.h"
#include "SM_MPU6050_6Axis_MotionApps20.h"
#include "string.h"

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
	cal.accelXFineGain = GAIN_UNDEFINED;
	cal.accelYFineGain = GAIN_UNDEFINED;
	cal.accelZFineGain = GAIN_UNDEFINED;
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
#ifdef ARDUINO_ESP32C3_DEV
  Serial.println("MotionSensor::Wire begin - dataPin 10, clkPin 9");
  Wire.begin(10, 9, 400000L);

#else
#ifdef ARDUINO_ARCH_ESP32
  Serial.println("MotionSensor::Wire begin - dataPin 21, clkPin 22");
  Wire.begin(21, 22, 400000L);
#else
#pragma message ( "MotionSensor supports either AVR or ESP32 architecures" )
#endif
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


bool MotionSensor::getLatestSensorReadings(MPU6050* mpu6050, MotionProcessor::SENSOR_READINGS* readings) {
	if ((mpu6050 == NULL) || (readings == NULL)) {
		EventMngr::getMngr()->handleEvent(MotionSensor::MOTION_SENSOR_RUNTIME_ERROR, 0, 1);
		return false;
	}

	/*
	 * Check there is a complete sample of data waiting for us
	 */
	uint16_t fifoCount = mpu6050->getFIFOCount();
	if (fifoCount < MPU6050::dmpPacketSize) {
		return false;
	}

	/*
	 * Check the mpu6050's buffer hasn't overflowed because we haven't been reading
	 * it quick enough, or have stalled for some reason.
	 */
	if (fifoCount == 1024) {
		// reset so we can continue cleanly
		mpu6050->resetFIFO();
		EventMngr::getMngr()->handleEvent(MotionSensor::MOTION_SENSOR_RUNTIME_ERROR, 0, 0);
		return false;
	}

	/*
	 * read a packet from FIFO and extract readings
	 */
	uint8_t fifoBuffer[MPU6050::dmpPacketSize];
	mpu6050->getFIFOBytes(fifoBuffer, MPU6050::dmpPacketSize);

	int16_t raw_quarternion[4];
	mpu6050->dmpGetQuaternion(raw_quarternion, fifoBuffer);

	/*
	 * The MPU6050 returns a rotational quaternion which represents the rotation to "real world" from "sensor" frame
	 */
	readings->rotationReading_rs.r = raw_quarternion[0];
	readings->rotationReading_rs.x = raw_quarternion[1];
	readings->rotationReading_rs.y = raw_quarternion[2];
	readings->rotationReading_rs.z = raw_quarternion[3];

	int16_t raw_accel_s[3];
	mpu6050->dmpGetAccel(raw_accel_s, fifoBuffer);
	readings->linearAcceleration_s.r = 0;
	readings->linearAcceleration_s.x = raw_accel_s[0];
	readings->linearAcceleration_s.y = raw_accel_s[1];
	readings->linearAcceleration_s.z = raw_accel_s[2];

	return true;
}


/*
 * Called repeatedly by updater - allows motionSensor to process Fifo
 * and generate new sample events
 */
void MotionSensor::update(uint16_t elapsedTime_ms)
{
	bool isNewReading = getLatestSensorReadings(&mpu6050, &sensorReadings);
	if (!isNewReading)
		return;

	motionProcessor.processSensorReadings(&sensorReadings);

	/*
	 * Notify system a new sample is available
	 */
	if (eventHandler != NULL)
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
	int16_t raw_accel_s[3];
	int16_t raw_gyro_s[3];

	for (uint16_t i = 0; i < numSamples; i++) {
		// read raw accel/gyro measurements from device
		while (mpu6050.dmpPacketAvailable() == false){
		}
			
		uint8_t fifoBuffer[MPU6050::dmpPacketSize];
		mpu6050.getFIFOBytes(fifoBuffer, MPU6050::dmpPacketSize);
//		mpu6050.dmpGetAccel(raw_accel_s, fifoBuffer);
//		mpu6050.dmpGetGyro(raw_gyro_s, fifoBuffer);
		// Get the readings direct from the sensor not from the fifo 
		mpu6050.getMotion6(raw_accel_s, raw_accel_s + 1, raw_accel_s + 2, raw_gyro_s, raw_gyro_s + 1, raw_gyro_s + 2);

		sum_ax += raw_accel_s[0];
		sum_ay += raw_accel_s[1];
		sum_az += raw_accel_s[2];
		sum_gx += raw_gyro_s[0];
		sum_gy += raw_gyro_s[1];
		sum_gz += raw_gyro_s[2];

		if (i % 50 == 49) {
			sampleAvgs->accelXAvg = sum_ax / (i+1);
			sampleAvgs->accelYAvg = sum_ay / (i+1);
			sampleAvgs->accelZAvg = sum_az / (i+1);
			sampleAvgs->gyroXAvg = sum_gx / (i+1);
			sampleAvgs->gyroYAvg = sum_gy / (i+1);
			sampleAvgs->gyroZAvg = sum_gz / (i+1);

			Serial.print("50 ");
			printSamples(sampleAvgs);
		}
	}

	sampleAvgs->accelXAvg = sum_ax / numSamples;
	sampleAvgs->accelYAvg = sum_ay / numSamples;
	sampleAvgs->accelZAvg = sum_az / numSamples;
	sampleAvgs->gyroXAvg = sum_gx / numSamples;
	sampleAvgs->gyroYAvg = sum_gy / numSamples;
	sampleAvgs->gyroZAvg = sum_gz / numSamples;
}


void MotionSensor::printCalibration(CALIBRATION* calibration) {
	Serial.print("Calibration: \tx:");
	Serial.print(calibration->accelXoffset);
	Serial.print("\ty:");
	Serial.print(calibration->accelYoffset);
	Serial.print("\tz:");
	Serial.print(calibration->accelZoffset);
	Serial.print("\ttx:");
	Serial.print(calibration->accelXFineGain);
	Serial.print("\tty:");
	Serial.print(calibration->accelYFineGain);
	Serial.print("\ttz:");
	Serial.print(calibration->accelZFineGain);
	Serial.print("\tgx:");
	Serial.print(calibration->gyroXoffset);
	Serial.print("\tgy:");
	Serial.print(calibration->gyroYoffset);
	Serial.print("\tgz:");
	Serial.println(calibration->gyroZoffset);
}

void MotionSensor::printCalibration() {
	Serial.print("Calibration: \tax:");
	Serial.print(mpu6050.getXAccelOffset());
	Serial.print("\tay:");
	Serial.print(mpu6050.getYAccelOffset());
	Serial.print("\taz:");
	Serial.print(mpu6050.getZAccelOffset());
	Serial.print("\ttx:");
	Serial.print(mpu6050_getXFineGain_accel());
	Serial.print("\tty:");
	Serial.print(mpu6050_getYFineGain_accel());
	Serial.print("\ttz:");
	Serial.print(mpu6050_getZFineGain_accel());
	Serial.print("\tgx:");
	Serial.print(mpu6050.getXGyroOffset());
	Serial.print("\tgy:");
	Serial.print(mpu6050.getYGyroOffset());
	Serial.print("\tgz:");
	Serial.println(mpu6050.getZGyroOffset());
}

void MotionSensor::printSamples(SAMPLE_AVGS* sample_avgs) {
	Serial.print("Samples: \tx:");
	Serial.print(sample_avgs->accelXAvg);
	Serial.print("\ty:");
	Serial.print(sample_avgs->accelYAvg);
	Serial.print("\tz:");
	Serial.print(sample_avgs->accelZAvg);
	Serial.print("\tgx:");
	Serial.print(sample_avgs->gyroXAvg);
	Serial.print("\tgy:");
	Serial.print(sample_avgs->gyroYAvg);
	Serial.print("\tgz:");
	Serial.println(sample_avgs->gyroZAvg);
}

void MotionSensor::findMaximumReadings(int32_t *ax, int32_t *ay, int32_t*az) {
	SAMPLE_AVGS current_samples;
	Serial.println("findMaximumReadings");

	*ax = 0;
	*ay = 0;
	*az = 0;

	for (int i = 0; i < 100; i++) {

		takeSamples(&current_samples, 10);
		printSamples(&current_samples);

		if (abs(current_samples.accelXAvg) > abs(*ax))
			*ax = current_samples.accelXAvg;

		if (abs(current_samples.accelYAvg) > abs(*ay))
			*ay = current_samples.accelYAvg;

		if (abs(current_samples.accelZAvg) > abs(*az))
			*az = current_samples.accelZAvg;
	}
	Serial.print("Max Values:\t");
	Serial.print(*ax); Serial.print("\t");
	Serial.print(*ay); Serial.print("\t");
	Serial.println(*az);
}

/*
 * calibrateOffsetSingleAxis - iteratively conveges on a offset calibration
 *                            value by trying values and slowly modifying them
 */
int MotionSensor::calibrateOffsetSingleAxis(CALIBRATION* calibration, int axis_id, uint16_t maxVal) {
	SAMPLE_AVGS sample_avgs;
	const uint16_t num_samples = 200;     //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
	const int16_t accel_target_accuracy = 8;     //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
	const int16_t giro_target_accuracy = 1;     //Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)

	int16_t expectedAccelX = 0;
	int16_t expectedAccelY = 0;
	int16_t expectedAccelZ = 0;

	Serial.println("MotionSensor::calibrateOffsetSingleAxis");

	if (axis_id == 0) expectedAccelX = maxVal;
	if (axis_id == 1) expectedAccelY = maxVal;
	if (axis_id == 2) expectedAccelZ = maxVal;
	if (axis_id == 3) expectedAccelX = -maxVal;
	if (axis_id == 4) expectedAccelY = -maxVal;
	if (axis_id == 5) expectedAccelZ = -maxVal;

	calibration->accelXoffset = expectedAccelX/ 8;
	calibration->accelYoffset = expectedAccelY/ 8;
	calibration->accelZoffset = expectedAccelZ/ 8;
	calibration->gyroXoffset = 0;
	calibration->gyroYoffset = 0;
	calibration->gyroZoffset = 0;

	bool finished = false;
	while (!finished) {
		setCalibration(calibration);
		printCalibration(calibration);
		takeSamples(&sample_avgs, 300);
		takeSamples(&sample_avgs, num_samples);
		printSamples(&sample_avgs);

		int16_t accelXDelta = sample_avgs.accelXAvg - expectedAccelX;
		int16_t accelYDelta = sample_avgs.accelYAvg - expectedAccelY;
		int16_t accelZDelta = sample_avgs.accelZAvg - expectedAccelZ;

		Serial.print("Deltas:\t\tx:"); Serial.print(accelXDelta); Serial.print("\ty:"); Serial.print(accelYDelta); Serial.print("\tz:"); Serial.println(accelZDelta);

		finished = true;
		if (abs(accelXDelta) > accel_target_accuracy) {
			finished = false;
			calibration->accelXoffset -= accelXDelta / 9;
		}

		if (abs(accelYDelta) > accel_target_accuracy) {
			finished = false;
			calibration->accelYoffset -= accelYDelta / 9;
		}

		if (abs(accelZDelta) > accel_target_accuracy) {
			finished = false;
			calibration->accelZoffset -= accelZDelta / 9;
		}

		if (abs(sample_avgs.gyroXAvg) > giro_target_accuracy) {
			finished = false;
			calibration->gyroXoffset -= 2* sample_avgs.gyroXAvg;
		}

		if (abs(sample_avgs.gyroYAvg) > giro_target_accuracy) {
			finished = false;
			calibration->gyroYoffset -= 2* sample_avgs.gyroYAvg;
		}

		if (abs(sample_avgs.gyroZAvg) > giro_target_accuracy) {
			finished = false;
			calibration->gyroZoffset -= 2*sample_avgs.gyroZAvg;
		}
	}
	return(0);
}

/*
 * runOffsetSelfCalibrate - iteratively converges on a set of offset calibration 
 *                          values by trying values and slowly modifying them
 */
int MotionSensor::runOffsetSelfCalibrate(CALIBRATION * calibration) {
	calibration->accelXoffset = 0;
	calibration->accelYoffset = 0;
	calibration->accelZoffset = 0;
	calibration->gyroXoffset = 0;
	calibration->gyroYoffset = 0;
	calibration->gyroZoffset = 0;
	calibration->accelXFineGain = GAIN_UNDEFINED;
	calibration->accelYFineGain = GAIN_UNDEFINED;
	calibration->accelZFineGain = GAIN_UNDEFINED;

	setOffsetCalibration(calibration);

	/*
	* Delay 10 seconds by taking 1000 samples.
	* This is to ensure sensor has settled down
	*/
	Serial.println("Waiting 10s for sensor to settle");
	SAMPLE_AVGS sample_avgs;
	takeSamples(&sample_avgs, 1000);

	Serial.println("Starting calibration");

	// This actually just performs Z axis calibration
	calibrateOffsetSingleAxis(calibration, 2, 16384);
	return(0);
}

/*
 * setCalibration - sets calibration values in MPU6050
 */
void MotionSensor::setCalibration(CALIBRATION* calibration)
{
	setOffsetCalibration(calibration);
	setGainCalibration(calibration);
}

/*
 * setCalibration - sets calibration values in MPU6050
 */
void MotionSensor::setOffsetCalibration(CALIBRATION* calibration)
{
	// Set Offsets for this chip instance
	mpu6050.setXGyroOffset(calibration->gyroXoffset);
	mpu6050.setYGyroOffset(calibration->gyroYoffset);
	mpu6050.setZGyroOffset(calibration->gyroZoffset);

	mpu6050.setXAccelOffset(calibration->accelXoffset);
	mpu6050.setYAccelOffset(calibration->accelYoffset);
	mpu6050.setZAccelOffset(calibration->accelZoffset);
}

/*
 * setCalibration - sets calibration values in MPU6050
 */
void MotionSensor::setGainCalibration(CALIBRATION* calibration)
{
	if (calibration->accelXFineGain != GAIN_UNDEFINED)
		mpu6050_setXFineGain_accel(calibration->accelXFineGain);
	if (calibration->accelYFineGain != GAIN_UNDEFINED)
		mpu6050_setYFineGain_accel(calibration->accelYFineGain);
	if (calibration->accelZFineGain != GAIN_UNDEFINED)
		mpu6050_setZFineGain_accel(calibration->accelZFineGain);
}

void MotionSensor::resetOffsetCalibration() {
	// Set Offsets for this chip instance
	mpu6050.setXGyroOffset(0);
	mpu6050.setYGyroOffset(0);
	mpu6050.setZGyroOffset(0);

	mpu6050.setXAccelOffset(0);
	mpu6050.setYAccelOffset(0);
	mpu6050.setZAccelOffset(0);
}

void MotionSensor::printSamples() {
	SAMPLE_AVGS sample_avgs;
	const uint16_t num_samples = 1000;     //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
	takeSamples(&sample_avgs, num_samples);

	Serial.println(sample_avgs.accelXAvg);
	Serial.println(sample_avgs.accelYAvg);
	Serial.println(sample_avgs.accelZAvg);
}

/* 
 * The fineGain register holds two 4 bit signed values. The top for accel, the lower for gyro trim.
 * 
 */
int8_t MotionSensor::mpu6050_getXFineGain_accel()
{
	return (mpu6050.getXFineGain() >> 4);
}

int8_t MotionSensor::mpu6050_getYFineGain_accel()
{
	return (mpu6050.getYFineGain() >> 4);
}

int8_t MotionSensor::mpu6050_getZFineGain_accel()
{
	return (mpu6050.getZFineGain() >> 4);
}

void MotionSensor::mpu6050_setXFineGain_accel(int8_t gain) {
	uint8_t oldGain = mpu6050.getXFineGain();
	gain = gain << 4 | oldGain & 0x0f;
	mpu6050.setXFineGain(gain);
}

void MotionSensor::mpu6050_setYFineGain_accel(int8_t gain) {
	uint8_t oldGain = mpu6050.getYFineGain();
	gain = gain << 4 | oldGain & 0x0f;
	mpu6050.setYFineGain(gain);
}

void MotionSensor::mpu6050_setZFineGain_accel(int8_t gain) {
	uint8_t oldGain = mpu6050.getZFineGain();
	gain = gain << 4 | oldGain & 0x0f;
	mpu6050.setZFineGain(gain);
}

