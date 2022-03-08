/*******************************************************************************
  MotionSensor.h Implements IMotionSensor using MPU6050

  Copyright(C) 2017-2022  Howard James May

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
   2      08-Mar-2019   Various updates including self calibration
   3      04-Mar-2021   Various updates including offset rotation calculation
*******************************************************************************/

#ifndef _MOTION_SENSOR_h
#define _MOTION_SENSOR_h

#include "IEventHandler.h"
#include "Updater.h"
#include "ISigInput.h"
#include "ISigOutput.h"

#include "I2Cdev.h"
#define MPU6050_INCLUDE_DMP_MOTIONAPPS20
#include "SM_MPU6050.h"
#include "Wire.h"
#include "Quaternion_16384.h"

//#define DEBUG

namespace SweetMaker {

	class MotionSensor :  private AutoUpdate {
	public:

    /*
     * These events are generated by MotionSensor as part of the SweetMaker framework
     */
		typedef enum
		{
			MOTION_SENSOR_INIT_ERROR = IEventHandler::MOTION_SENSOR_EVENTS + 0,
			MOTION_SENSOR_RUNTIME_ERROR = IEventHandler::MOTION_SENSOR_EVENTS + 1,
			MOTION_SENSOR_READY = IEventHandler::MOTION_SENSOR_EVENTS + 2,
			MOTION_SENSOR_NEW_SMPL_RDY = IEventHandler::MOTION_SENSOR_EVENTS + 3
		} SIG_GEN_EVENT;

    /*
     * Each MPU6050 will require it's own calibration values to work accurately
     */
		typedef struct calibration {
			int16_t accelXoffset;
			int16_t accelYoffset;
			int16_t accelZoffset;
			int16_t gyroXoffset;
			int16_t gyroYoffset;
			int16_t gyroZoffset;
		}CALIBRATION;

		const static uint16_t gravity_mm_ss = 9810;

		MotionSensor();
		int init();
		int init(CALIBRATION * calibration);

    /*
     * Events are handled by an EventHandler - if this is NULL
     * then the SweetMaker default handler is called.
     */
		void configEventHandler(IEventHandler * eventHandler);

    /*
     * The MP6050 may not be mounted flat. These routines allow an offset
     * to compensate for this. Either a RotationQuaternion can be supplied
     * or the MPU6050's calculation of Gravity can be used to auto level.
     *
     * Note autolevel will only work after the MotionSensor has finished starting
     * up and has been properly callibrated.
     */
    void autoLevel();
    void setOffsetRotation(RotationQuaternion_16384* q);
    void clearOffsetRotation(); // including autoLevel;

    /*
     * This generates calibration values for MPU6050.
     * The MPU6050 should be flat, stationary and with the Z axis upmost
     * It should remain in this position until calibration is complete
     *
     * Note: Calibration values for an MPU6050 instance don't change
     */
    int runSelfCalibrate(CALIBRATION * calibration);

		bool readingAvailable();

    /*
     * This results in the MPU6050 being polled to see if a new reading is available
     */
		void update(uint16_t elapsedTime_ms);

    /*
     * Current offsetRotation - if any
     */
    RotationQuaternion_16384* offsetRotation = NULL;

    /*
     * Current rotation (following any offset)
     */
    RotationQuaternion_16384 rotQuat;

    /*
     * Rotation delta -can be helpful
     */
    RotationQuaternion_16384 rotQuatDelta;

    /*
     * raw linear acceleration values;
     */
    int16_t linearAccel[3];

	private:
		IEventHandler * eventHandler;
		MPU6050 mpu6050;

		typedef struct sample_avgs {
			int32_t accelXAvg;
			int32_t accelYAvg;
			int32_t accelZAvg;
			int32_t gyroXAvg;
			int32_t gyroYAvg;
			int32_t gyroZAvg;
		}SAMPLE_AVGS;

		void takeSamples(SAMPLE_AVGS * sample_avgs, uint16_t num_samples);
		void setCalibration(CALIBRATION * calibration);
	};
}
#endif

