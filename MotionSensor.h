/*******************************************************************************
  MotionSensor.h Implements IMotionSensor using MPU6050

  Copyright(C) 2017-2019  Howard James May

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
		typedef enum
		{
			MOTION_SENSOR_INIT_ERROR = IEventHandler::MOTION_SENSOR_EVENTS + 0,
			MOTION_SENSOR_RUNTIME_ERROR = IEventHandler::MOTION_SENSOR_EVENTS + 1,
			MOTION_SENSOR_READY = IEventHandler::MOTION_SENSOR_EVENTS + 2,
			MOTION_SENSOR_NEW_SMPL_RDY = IEventHandler::MOTION_SENSOR_EVENTS + 3
		} SIG_GEN_EVENT;

		typedef enum {
			MSSP_6MS = 2,
			MSSP_8MS = 3,
			MSSP_10MS = 4,
			MSSP_12MS = 5,
			MSSP_14MS = 6,
			MSSP_16MS = 7
		}SAMPLE_PERIOD;

		typedef struct calibration {
			int16_t accelXoffset;
			int16_t accelYoffset;
			int16_t accelZoffset;
			int16_t gyroXoffset;
			int16_t gyroYoffset;
			int16_t gyroZoffset;
		}CALIBRATION;

		const static uint16_t sampleRate_hz = 100;
		const static uint8_t samplePeriod_ms = 10;
		const static uint16_t gravity_mm_ss = 9810;

		MotionSensor();
		int init();
		int init(CALIBRATION * calibration);
		void configEventHandler(IEventHandler * eventHandler);
		void clearOffsetRotation();
		void setOffsetRotation(RotationQuaternion_16384 *q);
		int runSelfCalibrate(CALIBRATION * calibration);

		bool readingAvailable();
		void update(uint16_t elapsedTime_ms);

    RotationQuaternion_16384* offsetRotation = NULL;
    RotationQuaternion_16384 rotQuat;
    RotationQuaternion_16384 rotQuatDelta;
    int16_t linearAccel[3];

#define MOTION_SENSOR_SCALE_FACTOR_U_SS (599)


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

