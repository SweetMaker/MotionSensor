/*******************************************************************************
Quaternion_16384.h Class for a Unit Quaternion represented by int16_t values

 NOTE: This is designed for use with motion sensors which output unit quaternions
       scaled to have magnitude 0x4000 such as the MPU6050

Copyright(C) 2019  Howard James May

This file is part of the SweetMaker project

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
*******************************************************************************/

#ifndef __QUATERNION_16384_H__
#define __QUATERNION_16384_H__

#include <stdint.h>

namespace SweetMaker {

	class Quaternion_16384 {
	public:
		int16_t r;
		int16_t x;
		int16_t y;
		int16_t z;

		Quaternion_16384();
		Quaternion_16384(int16_t r, int16_t x, int16_t y, int16_t z);
		Quaternion_16384(Quaternion_16384 * q);

		void crossProduct(Quaternion_16384 * q);
		int32_t dotProduct(Quaternion_16384 * q);
		void conjugate();
		void normalize();
    void printQ(void);


		uint32_t getMagnitude();

		const static int32_t unit_scale_factor = 0x4000;
	};

	class RotationQuaternion_16384 : public Quaternion_16384
	{
	public:
		RotationQuaternion_16384();
		RotationQuaternion_16384(Quaternion_16384 *q);
		RotationQuaternion_16384(int16_t r, int16_t x, int16_t y, int16_t z);
		RotationQuaternion_16384(float angle, int16_t x, int16_t y, int16_t z);

		void rotate(Quaternion_16384 * q);

		int16_t getSinRotX();
		int16_t getSinRotY();
		int16_t getSinRotZ();

		int16_t getRotX();
		int16_t getRotY();
		int16_t getRotZ();
	};

}

#endif
