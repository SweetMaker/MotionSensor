#ifndef _MOTION_PROCESSOR_h
#define _MOTION_PROCESSOR_h
#include "Quaternion_16384.h"

namespace SweetMaker {

	class MotionProcessor {
	public:
		typedef struct sensorReadings {
			RotationQuaternion_16384 rotationReading_rs;
			Quaternion_16384 linearAcceleration_s;
		}SENSOR_READINGS;

		typedef struct processedReadings {
			RotationQuaternion_16384 rotQuat_rm; 		/* Current rotation (to real world from from model - following offset) */
			RotationQuaternion_16384 rotQuatDelta; 		/* Rotation delta -can be helpful (frame independent) */
			Quaternion_16384 gravity_m;     	/* Gravity - relative to "model" frame */
			Quaternion_16384 linearAccel_m; 	/* Raw linear acceleration values - relative to "model" frame */
		}PROCESSED_READINGS;

		/* Configuration */
		RotationQuaternion_16384 offsetRotation_sm_xy; 		/* Current offsetRotation to sensor from model frame for xy tilt */
		RotationQuaternion_16384 offsetRotation_r_z; 		/* Current offsetRotation in real world for z rotation */

        /* Data Types */
		SENSOR_READINGS sensorReadings;
		PROCESSED_READINGS processedReadings;

		MotionProcessor();

		void processSensorReadings(SENSOR_READINGS* latestSensorReadings);
		RotationQuaternion_16384 autoLevel();
		void setLevelOffset(RotationQuaternion_16384 *offset);
		void clearLevelOffset();
		void autoYawOffset();
		void clearYawOffset();

	private:
		static RotationQuaternion_16384 _calculateLevelOffset(RotationQuaternion_16384* rot_rs);
		static RotationQuaternion_16384 _calculateYawOffset(RotationQuaternion_16384* rot_rs, RotationQuaternion_16384 * offset_sm_xy);
		static RotationQuaternion_16384 _calculateRotQuat_rm(RotationQuaternion_16384 *rot_rs, RotationQuaternion_16384* offset_sm_xy, RotationQuaternion_16384 * offset_r_z);
		static Quaternion_16384 _calculateLinearAccel_m(RotationQuaternion_16384 * rot_rs, RotationQuaternion_16384* rot_sm, Quaternion_16384 * accel_s);
		static Quaternion_16384 _compensateAccelForGravity(Quaternion_16384* raw_accel_s, RotationQuaternion_16384* orientation_rs);
	};
};
#endif