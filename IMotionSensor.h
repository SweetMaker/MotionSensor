/*******************************************************************************
IMotionSensor.h Provides Interface to motion sensors

Copyright(C) 2017  Howard James May

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
   1      07-Sep-2017   Initial release
*******************************************************************************/

#ifndef __I_MOTIONSENSOR_H__
#define __I_MOTIONSENSOR_H__

#include <stdint.h>
#include "IEventHandler.h"

namespace SweetMaker {
	/*
	* IMotionSensor:
	*/
	class IMotionSensor {
	public:


		virtual uint8_t getReading(SAMPLE * reading) = 0;
		virtual bool readingAvailable() = 0;
	};
}

#endif
