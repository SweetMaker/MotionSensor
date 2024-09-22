#include "string.h"
#include "MotionProcessor.h"

using namespace SweetMaker;

MotionProcessor::MotionProcessor()
{
	memset(this, 0, sizeof(MotionProcessor));
	this->offsetRotation_r_z.r = 16384;
	this->offsetRotation_sm_xy.r = 16384;
}

void MotionProcessor::processSensorReadings(SENSOR_READINGS* newReadings) {
	processedReadings.rotQuatDelta = RotationQuaternion_16384::calcDelta(&sensorReadings.rotationReading_rs, &newReadings->rotationReading_rs);
	processedReadings.rotQuat_rm = _calculateRotQuat_rm(&newReadings->rotationReading_rs, &offsetRotation_sm_xy, &offsetRotation_r_z);
	processedReadings.linearAccel_m = _calculateLinearAccel_m(&newReadings->rotationReading_rs, &offsetRotation_sm_xy, &newReadings->linearAcceleration_s);
	processedReadings.gravity_m = processedReadings.rotQuat_rm.getGravity();

	memcpy(&sensorReadings, newReadings, sizeof(SENSOR_READINGS));
}

RotationQuaternion_16384 MotionProcessor::autoLevel() {
	offsetRotation_sm_xy = _calculateLevelOffset(&sensorReadings.rotationReading_rs);
	processedReadings.rotQuat_rm = _calculateRotQuat_rm(&sensorReadings.rotationReading_rs, &offsetRotation_sm_xy, &offsetRotation_r_z);
	processedReadings.linearAccel_m = _calculateLinearAccel_m(&sensorReadings.rotationReading_rs, &offsetRotation_sm_xy, &sensorReadings.linearAcceleration_s);
	processedReadings.gravity_m = processedReadings.rotQuat_rm.getGravity();
	return offsetRotation_sm_xy;
};

void MotionProcessor::setLevelOffset(RotationQuaternion_16384 * _offsetRotation_sm_xy) {
	offsetRotation_sm_xy = *_offsetRotation_sm_xy;
	processedReadings.rotQuat_rm = _calculateRotQuat_rm(&sensorReadings.rotationReading_rs, &offsetRotation_sm_xy, &offsetRotation_r_z);
	processedReadings.linearAccel_m = _calculateLinearAccel_m(&sensorReadings.rotationReading_rs, &offsetRotation_sm_xy, &sensorReadings.linearAcceleration_s);
	processedReadings.gravity_m = processedReadings.rotQuat_rm.getGravity();
};

void MotionProcessor::clearLevelOffset() {
	offsetRotation_sm_xy = { (int16_t)16384, 0, 0, 0 };
	processedReadings.rotQuat_rm = _calculateRotQuat_rm(&sensorReadings.rotationReading_rs, &offsetRotation_sm_xy, &offsetRotation_r_z);
	processedReadings.linearAccel_m = _calculateLinearAccel_m(&sensorReadings.rotationReading_rs, &offsetRotation_sm_xy, &sensorReadings.linearAcceleration_s);
	processedReadings.gravity_m = processedReadings.rotQuat_rm.getGravity();
};

void MotionProcessor::autoYawOffset() {
	offsetRotation_r_z = _calculateYawOffset(&sensorReadings.rotationReading_rs, &offsetRotation_sm_xy);
	processedReadings.rotQuat_rm = _calculateRotQuat_rm(&sensorReadings.rotationReading_rs, &offsetRotation_sm_xy, &offsetRotation_r_z);
};

void MotionProcessor::clearYawOffset() {
	offsetRotation_r_z = { (int16_t)16384, 0, 0, 0 };
	processedReadings.rotQuat_rm = _calculateRotQuat_rm(&sensorReadings.rotationReading_rs, &offsetRotation_sm_xy, &offsetRotation_r_z);
};

RotationQuaternion_16384 MotionProcessor::_calculateLevelOffset(RotationQuaternion_16384* rot_rs)
{
	Quaternion_16384 zAxis_m(0, 0, 0, 16384);
	Quaternion_16384 gravity_s = rot_rs->getGravity();
	RotationQuaternion_16384 offset_sm_xy = RotationQuaternion_16384::findOffsetRotation(&zAxis_m, &gravity_s);
	return offset_sm_xy;
}

/*
 * _calculateYawOffset - this calculates a rotation offset to reorientate the modelFrame with no rotation about real World Z axis.
 *                       
 */
RotationQuaternion_16384 MotionProcessor::_calculateYawOffset(RotationQuaternion_16384* rot_rs, RotationQuaternion_16384* offset_sm_xy)
{
	RotationQuaternion_16384 rot_rm = Quaternion_16384::crossProduct(rot_rs, offset_sm_xy);
	RotationQuaternion_16384 yawOffset = rot_rm.getRotationAboutZ();
	return yawOffset;
}


RotationQuaternion_16384 MotionProcessor::_calculateRotQuat_rm(RotationQuaternion_16384* rot_rs, RotationQuaternion_16384* offset_sm_xy, RotationQuaternion_16384* offset_r_z) {
	RotationQuaternion_16384 result = Quaternion_16384::crossProduct(rot_rs, offset_sm_xy);
	result = Quaternion_16384::crossProduct(offset_r_z, &result);
	return result;
};

Quaternion_16384 MotionProcessor::_calculateLinearAccel_m(RotationQuaternion_16384* rot_rs, RotationQuaternion_16384* rot_sm, Quaternion_16384* accelReading_s) {
	Quaternion_16384 accel_s = _compensateAccelForGravity(accelReading_s, rot_rs);
	RotationQuaternion_16384 rot_ms = Quaternion_16384::conjugate(rot_sm);
	return rot_ms.rotate(&accel_s);
};

Quaternion_16384 MotionProcessor::_compensateAccelForGravity(Quaternion_16384* raw_accel_s, RotationQuaternion_16384* orientation_rs) {
	Quaternion_16384 accel;
	RotationQuaternion_16384 gravity_s = orientation_rs->getGravity();

	accel.r = 0;
	accel.x = 2 * raw_accel_s->x - gravity_s.x;
	accel.y = 2 * raw_accel_s->y - gravity_s.y;
	accel.z = 2 * raw_accel_s->z - gravity_s.z;
	return accel;
}
