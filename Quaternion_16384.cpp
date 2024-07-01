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
   1      26-Apr-2019   Initial release
--------|-------------|--------------------------------------------------------|
						- Added rounding for better accuracy
   2      25-Feb-2024   - Added getRotationAboutZ
*******************************************************************************/

#include "math.h"
#include "Quaternion_16384.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


using namespace SweetMaker;

Quaternion_16384::Quaternion_16384()
{
	r = 0;
	x = 0;
	y = 0;
	z = 0;
}

Quaternion_16384::Quaternion_16384(int16_t _r, int16_t _x, int16_t _y, int16_t _z)
{
	r = _r;
	x = _x;
	y = _y;
	z = _z;
}

Quaternion_16384::Quaternion_16384(Quaternion_16384 * q)
{
	r = q->r;
	x = q->x;
	y = q->y;
	z = q->z;
}

/*
 * crossProduct: Performs the cross product q * s where q is 
 *               this quaternion. 
 *
 * NOTE: quaternion cross product is non commutative and so 
 *       q * s != s * q
 */
[[deprecated]]
void Quaternion_16384::crossProduct(Quaternion_16384 * s) 
{
	int32_t _r = ((int32_t)r * (int32_t)s->r) - ((int32_t)x * (int32_t)s->x) - ((int32_t)y * (int32_t)s->y) - ((int32_t)z * (int32_t)s->z);
	int32_t _x = ((int32_t)x * (int32_t)s->r) + ((int32_t)r * (int32_t)s->x) - ((int32_t)z * (int32_t)s->y) + ((int32_t)y * (int32_t)s->z);
	int32_t _y = ((int32_t)y * (int32_t)s->r) + ((int32_t)z * (int32_t)s->x) + ((int32_t)r * (int32_t)s->y) - ((int32_t)x * (int32_t)s->z);
	int32_t _z = ((int32_t)z * (int32_t)s->r) - ((int32_t)y * (int32_t)s->x) + ((int32_t)x * (int32_t)s->y) + ((int32_t)r * (int32_t)s->z);

  this->r = asrRounded(_r, 14);
  this->x = asrRounded(_x, 14);
  this->y = asrRounded(_y, 14);
  this->z = asrRounded(_z, 14);
}

Quaternion_16384 Quaternion_16384::crossProduct(const Quaternion_16384* a, const Quaternion_16384* b) {
	int32_t _r = ((int32_t)a->r * (int32_t)b->r) - ((int32_t)a->x * (int32_t)b->x) - ((int32_t)a->y * (int32_t)b->y) - ((int32_t)a->z * (int32_t)b->z);
	int32_t _x = ((int32_t)a->x * (int32_t)b->r) + ((int32_t)a->r * (int32_t)b->x) - ((int32_t)a->z * (int32_t)b->y) + ((int32_t)a->y * (int32_t)b->z);
	int32_t _y = ((int32_t)a->y * (int32_t)b->r) + ((int32_t)a->z * (int32_t)b->x) + ((int32_t)a->r * (int32_t)b->y) - ((int32_t)a->x * (int32_t)b->z);
	int32_t _z = ((int32_t)a->z * (int32_t)b->r) - ((int32_t)a->y * (int32_t)b->x) + ((int32_t)a->x * (int32_t)b->y) + ((int32_t)a->r * (int32_t)b->z);

	Quaternion_16384 result;
	result.r = asrRounded(_r, 14);
	result.x = asrRounded(_x, 14);
	result.y = asrRounded(_y, 14);
	result.z = asrRounded(_z, 14);

	return (result);
}

/*
 * dotProduct:: The dot product is cos(t) where t is the angle between 
 *              two 4D quaternions
 */
[[deprecated]]
int16_t Quaternion_16384::dotProduct(Quaternion_16384* q)
{
	int32_t ret_val;
	ret_val = (int32_t)x * (int32_t)q->x;
	ret_val += (int32_t)y * (int32_t)q->y;
	ret_val += (int32_t)z * (int32_t)q->z;
	ret_val += (int32_t)r * (int32_t)q->r;

	return (asrRounded(ret_val, 14));
}
int16_t Quaternion_16384::dotProduct(Quaternion_16384* a, Quaternion_16384* b)
{
	int32_t ret_val;
	ret_val = (int32_t)a->x * (int32_t)b->x;
	ret_val += (int32_t)a->y * (int32_t)b->y;
	ret_val += (int32_t)a->z * (int32_t)b->z;
	ret_val += (int32_t)a->r * (int32_t)b->r;

	return (asrRounded(ret_val, 14));
}

/*
 * conjugate:: the quaternion conjugate is formed by negating the vector part
 */
[[deprecated]]
void Quaternion_16384::conjugate()
{
	x = -x;
	y = -y;
	z = -z;
}

Quaternion_16384 Quaternion_16384::conjugate(Quaternion_16384 * input)
{
	Quaternion_16384 result;
	result.r = input->r;
	result.x = -input->x;
	result.y = -input->y;
	result.z = -input->z;
	return result;
}

/*
 * normalize:: scales the quaternion to make it a unit quaternion
 *
 */
void Quaternion_16384::normalize() 
{
	int32_t mag = getMagnitude();
	if (mag == 0)
		return;

	if (mag< 0x2000)
	{
		uint16_t pre_scale = 0x1000 / mag;
		r = r * pre_scale;
		x = x * pre_scale;
		y = y * pre_scale;
		z = z * pre_scale;
		mag = getMagnitude();
	}

	r = (r * unit_scale_factor) / mag;
	x = (x * unit_scale_factor) / mag;
	y = (y * unit_scale_factor) / mag;
	z = (z * unit_scale_factor) / mag;
}


uint16_t my_sqrt(uint32_t input)
{
	// Declare initial approximation

	// Calculare next approximation
	// Continue until desired accuracy achieved

	// Compensate for error

	return(0);
}


/*
 * getMagnitude:: this calculates the scalar size of the quaternion
 *                returns 16384 for a unit quaternion
 *
 * NOTE: this uses 'sqrt' which is computationally expensive. 
 * NOTE: if the magnitude is small (<1000) then the accurancy is poor.
 */
uint32_t SweetMaker::Quaternion_16384::getMagnitude() {
	uint32_t ret_val;
	ret_val = (int32_t)r * (int32_t)r;
	ret_val += (int32_t)x * (int32_t)x;
	ret_val += (int32_t)y * (int32_t)y;
	ret_val += (int32_t)z * (int32_t)z;
	ret_val = (int32_t)sqrt(ret_val);

	return (ret_val);
}

Quaternion_16384& SweetMaker::Quaternion_16384::operator-(const Quaternion_16384& rhs)
{
	this->x -= rhs.x;
	this->y -= rhs.y;
	this->z -= rhs.z;
	return *this;
}


Quaternion_16384& SweetMaker::Quaternion_16384::operator+(const Quaternion_16384& rhs)
{
	this->x += rhs.x;
	this->y += rhs.y;
	this->z += rhs.z;
	return *this;
}

RotationQuaternion_16384::RotationQuaternion_16384(float angle_degrees, int16_t _x, int16_t _y, int16_t _z)
{
	double angle_rad = angle_degrees * 2 * M_PI / 360;
	double cos_theta = cos(angle_rad/2);
	double sin_theta = sin(angle_rad/2);

	double mag = (uint32_t)_x * (uint32_t)_x + (uint32_t)_y * (uint32_t)_y + (uint32_t)_z * (uint32_t)_z;
	mag = sqrt(mag);

	x = ((int32_t)_x * 16384) / mag;
	y = ((int32_t)_y * 16384) / mag;
	z = ((int32_t)_z * 16384) / mag;

	x = (int16_t)round(sin_theta * x);
	y = (int16_t)round(sin_theta * y);
	z = (int16_t)round(sin_theta * z);
	r = (int16_t)round(16384 * cos_theta);
}

SweetMaker::RotationQuaternion_16384::RotationQuaternion_16384() : Quaternion_16384(0x2000, 0, 0, 0x2000)
{
}

RotationQuaternion_16384::RotationQuaternion_16384(Quaternion_16384* q) : Quaternion_16384(q)
{
}

RotationQuaternion_16384::RotationQuaternion_16384(Quaternion_16384 q) : Quaternion_16384(q)
{
}

RotationQuaternion_16384::RotationQuaternion_16384(int16_t r, int16_t x, int16_t y, int16_t z) : Quaternion_16384(r, x, y, z)
{
}

/*
* Quaternion rotation assumes the following:
* 1) This quaternion is a pure quaternion (r == 0)
* 2) The rotation quaternion is a unit quaternion
* 3) The vector part of the rotation quat is the axis of rotation
* 4) The real part == cos(t/2) where t is the angle of rotation
*
* The rotation is achieved then achieved with the following equation:
*    q_ = (r)*q*(r-1)
* where:
*    q_ is the rotated quaternion
*    r  is the rotation quaternion
*    r-1 is the inverse of the rotation quaternion
*    q is the quaternion to be rotated
*/
Quaternion_16384 RotationQuaternion_16384::rotate(const Quaternion_16384 * input_q)
{
	RotationQuaternion_16384 conj_this;
	conj_this = Quaternion_16384::conjugate(this);

	Quaternion_16384 rotated_q;

	rotated_q = Quaternion_16384::crossProduct(this, input_q);
	rotated_q = Quaternion_16384::crossProduct(&rotated_q, &conj_this);

	return rotated_q;
}

/*
* getSinRotX: returns sin(theta) * 16384 where theta is rotation about x
*
*/
int16_t RotationQuaternion_16384::getSinRotX()
{
  int32_t gy = 2 * (((int32_t)r * (int32_t)x) + ((int32_t)y * (int32_t)z));
  gy = asrRounded(gy, 14);
	return (gy);
}

/*
* getSinRotY: returns sin(theta) * 16384 where theta is rotation about y
*
*/
int16_t RotationQuaternion_16384::getSinRotY()
{
  int32_t gx = 2 * (((int32_t)x * (int32_t)z) - ((int32_t)r * (int32_t)y));
  gx = asrRounded(gx, 14);
  return(gx);
}

/*
* getSinRotZ: returns sin(theta) * 16384 where theta is rotation about z
*
* Note: for sensors without magnometers this will drift because it has
*       no absolute reference (unlike X & Y which have gravity)
*/
int16_t RotationQuaternion_16384::getSinRotZ()
{
  Quaternion_16384 qx(0, 16384, 0, 0);
  qx = rotate(&qx);
  return (qx.y);
}

/*
* getCosRotZ: returns cos(theta) * 16384 where theta is rotation about z
*
* Note: for sensors without magnometers this will drift because it has
*       no absolute reference (unlike X & Y which have gravity)
*/
int16_t RotationQuaternion_16384::getCosRotZ()
{
  Quaternion_16384 qx(0, 16384, 0, 0);
  qx = rotate(&qx);
  return (qx.x);
}

/*
* Returns +/-16384 for +/-(pi/2)
*/
int16_t RotationQuaternion_16384::getRotX()
{
	double rotX = getSinRotX();
	rotX /= 16384;
	rotX = (asin(rotX) * 0x8000 / M_PI);
	return (int16_t)rotX;
}

/*
 * Returns +/-16384 for +/-(pi/2)
 */
int16_t RotationQuaternion_16384::getRotY()
{
	double rotY = getSinRotY();
	rotY /= 16384;
	rotY = (asin(rotY) * 0x8000 / M_PI);
	return (int16_t)rotY;
}

/*
* Returns +/-16384 for +/-(pi/2)
*/
int16_t RotationQuaternion_16384::getRotZ()
{
	double rotZ = getSinRotZ();
	rotZ /= 16384;
	rotZ = (asin(rotZ) * 0x8000 / M_PI);
	return (int16_t)rotZ;
}

/*
 * getGravity - Gravity (+ve z-axis in base frame) transfered into rotated frame. This is useful for isolating x and y tilt from
 *              rotation about the z axis. 
 */
[[deprecated]]
void RotationQuaternion_16384::getGravity(Quaternion_16384* gq)
{
	gq->r = 0;

	int32_t gx = 2 * (((int32_t)x * (int32_t)z) - ((int32_t)r * (int32_t)y));
	gq->x = asrRounded(gx, 14);

	int32_t gy = 2 * (((int32_t)r * (int32_t)x) + ((int32_t)y * (int32_t)z));
	gq->y = asrRounded(gy, 14);

	int32_t gz = ((int32_t)r * (int32_t)r) - ((int32_t)x * (int32_t)x) - ((int32_t)y * (int32_t)y) + ((int32_t)z * (int32_t)z);
	gq->z = asrRounded(gz, 14);

	return;
}

Quaternion_16384 RotationQuaternion_16384::getGravity()
{
	Quaternion_16384 gq;
	gq.r = 0;

	int32_t gx = 2 * (((int32_t)x * (int32_t)z) - ((int32_t)r * (int32_t)y));
	gq.x = asrRounded(gx, 14);

	int32_t gy = 2 * (((int32_t)r * (int32_t)x) + ((int32_t)y * (int32_t)z));
	gq.y = asrRounded(gy, 14);

	int32_t gz = ((int32_t)r * (int32_t)r) - ((int32_t)x * (int32_t)x) - ((int32_t)y * (int32_t)y) + ((int32_t)z * (int32_t)z);
	gq.z = asrRounded(gz, 14);

	return gq;
}

/*
 * findOffsetRotation - given two vectors this calculates a rotation from one to the other.
 */
RotationQuaternion_16384 RotationQuaternion_16384::findOffsetRotation(Quaternion_16384 * first, Quaternion_16384 * second)
{
	RotationQuaternion_16384 result;
    result = Quaternion_16384::crossProduct(first, second);

	int16_t _r = dotProduct(first, second);
	result.r = _r;
	result.r += 16384;

    result.normalize();

	return result;
}

/*
 * getRotationAboutZ - isolate rotation about z-axis from x and y
 *                   - this is done by 
 */
RotationQuaternion_16384 RotationQuaternion_16384::getRotationAboutZ() {
	// Calculate the rotation related to just xy tilt by using 'gravity'
	Quaternion_16384 z_axis(0, 0, 0, 16384);
	Quaternion_16384 gravity = this->getGravity();
	RotationQuaternion_16384 rot_xy = findOffsetRotation(&gravity, &z_axis);

	// On the assumption that rot_xyz == rot_z * rot_xy
	// Then rot_xyz * inv(rot_xy) == rot_z * rot_xy *inv(rot_xy) == rot_z
	
	rot_xy = Quaternion_16384::conjugate(&rot_xy);
	RotationQuaternion_16384 z_rot; 
	z_rot = Quaternion_16384::crossProduct(this, &rot_xy);
	return z_rot;
}


#ifdef ARDUINO
#include "Arduino.h"
void Quaternion_16384::printQ(void)
{
  Serial.print(this->r); Serial.print("\t");
  Serial.print(this->x); Serial.print("\t");
  Serial.print(this->y); Serial.print("\t");
  Serial.println(this->z);
}
#else
#include <stdio.h>
void Quaternion_16384::printQ(void)
{
  printf("%d\t%d\t%d\t\%d\n", this->r, this->x, this->y, this->z);
 }

#endif

/*
 * Arithmetic Shift Right for signed integers
 */
int16_t Quaternion_16384::asr(int32_t value, uint8_t amount)
{
  if (value < 0)
  {
    return (int16_t)(-(-value >> amount));
  }
  return (int16_t)(value >> amount);
}

/*
 * Arithmetic Shift Right with rounding for signed integers
 */
int16_t Quaternion_16384::asrRounded(int32_t value, uint8_t amount)
{
	int32_t round_offset = (1 << (amount - 1)) -1;
	if (value < 0)
	{
		value -= round_offset;
		return (int16_t)(-(-value >> amount));
	}
	value += round_offset;
	return (int16_t)(value >> amount);
}

RotationQuaternion_16384 RotationQuaternion_16384::calcDelta(RotationQuaternion_16384* old_q, RotationQuaternion_16384* new_q) {
	/*
	 * Delta is calculated by taking the conjugate of the old rotation
	 * and removing if from the new. This is effectively a subtraction
	 */
	RotationQuaternion_16384 old_conj;
	old_conj = Quaternion_16384::conjugate(old_q);

	RotationQuaternion_16384 delta;
	delta = Quaternion_16384::crossProduct(&old_conj, new_q);
	return delta;
}
