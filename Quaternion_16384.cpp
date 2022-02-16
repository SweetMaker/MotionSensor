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
void Quaternion_16384::crossProduct(Quaternion_16384 * s) 
{

	int32_t _r = ((int32_t)r * (int32_t)s->r) - ((int32_t)x * (int32_t)s->x) - ((int32_t)y * (int32_t)s->y) - ((int32_t)z * (int32_t)s->z);
	int32_t _x = ((int32_t)x * (int32_t)s->r) + ((int32_t)r * (int32_t)s->x) - ((int32_t)z * (int32_t)s->y) + ((int32_t)y * (int32_t)s->z);
	int32_t _y = ((int32_t)y * (int32_t)s->r) + ((int32_t)z * (int32_t)s->x) + ((int32_t)r * (int32_t)s->y) - ((int32_t)x * (int32_t)s->z);
	int32_t _z = ((int32_t)z * (int32_t)s->r) - ((int32_t)y * (int32_t)s->x) + ((int32_t)x * (int32_t)s->y) + ((int32_t)r * (int32_t)s->z);

	this->r = (int16_t)(_r / unit_scale_factor);
	this->x = (int16_t)(_x / unit_scale_factor);
	this->y = (int16_t)(_y / unit_scale_factor);
	this->z = (int16_t)(_z / unit_scale_factor);
}

/*
 * dotProduct:: The dot product is cos(t) where t is the angle between 
 *              two 4D quaternions
 */
int32_t Quaternion_16384::dotProduct(Quaternion_16384 * q) 
{
	int32_t ret_val;
	ret_val = (int32_t)x*(int32_t)q->x;
	ret_val += (int32_t)y*(int32_t)q->y;
	ret_val += (int32_t)z*(int32_t)q->z;
	ret_val += (int32_t)r*(int32_t)q->r;
	ret_val /= unit_scale_factor;

	return ((uint32_t)ret_val);
}

/*
 * conjugate:: the quaternion conjugate is formed by negating the vector part
 */
void Quaternion_16384::conjugate() 
{
	x = -x;
	y = -y;
	z = -z;
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

	x *= sin_theta;
	y *= sin_theta;
	z *= sin_theta;
	r = 16384 * cos_theta;
}

SweetMaker::RotationQuaternion_16384::RotationQuaternion_16384() : Quaternion_16384(0x2000, 0, 0, 0x2000)
{
}

RotationQuaternion_16384::RotationQuaternion_16384(Quaternion_16384 *q) : Quaternion_16384(q)
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
void RotationQuaternion_16384::rotate(Quaternion_16384 * subject_quat)
{
	RotationQuaternion_16384 rq_conj(this);
	rq_conj.conjugate();

	Quaternion_16384 rotated_q(this);

	rotated_q.crossProduct(subject_quat);
	rotated_q.crossProduct(&rq_conj);

	subject_quat->r = rotated_q.r;
	subject_quat->x = rotated_q.x;
	subject_quat->y = rotated_q.y;
	subject_quat->z = rotated_q.z;
}

/*
* getSinRotX: returns sin(theta) * 16384 where theta is rotation about x
*
*/
int16_t RotationQuaternion_16384::getSinRotX()
{
	Quaternion_16384 qy(0, 0, 16384, 0);
	rotate(&qy);
	return (qy.z);
}

/*
* getSinRotY: returns sin(theta) * 16384 where theta is rotation about y
*
*/
int16_t RotationQuaternion_16384::getSinRotY()
{
	Quaternion_16384 qx(0, 16384, 0, 0);
	rotate(&qx);
	return (qx.z);
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
	rotate(&qx);
	return (qx.y);
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


#ifdef ARDUINO
#include "Arduino.h"
void Quaternion_16384::printQ(void)
{
  Serial.print(this->r); Serial.print("\t");
  Serial.print(this->x); Serial.print("\t");
  Serial.print(this->y); Serial.print("\t");
  Serial.println(this->z);
}
#endif
