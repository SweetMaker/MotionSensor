#include <Wire.h>
#include <SweetMaker.h>
#include "MotionSensor.h"
#include "MotionProcessor.h"

using namespace SweetMaker;


void myEventHandler(uint16_t eventId, uint8_t src, uint16_t eventInfo);

static unsigned long lastUpdateTime_ms;

#ifdef ARDUINO_ARCH_AVR
const static uint8_t ms_scl = 19;    // A5+
const static uint8_t ms_sda = 18;    // A4
const static uint8_t ms_5v_pin = 17; // A3
const static uint8_t ms_0v_pin = 16; // A2

const static uint8_t ledStripSigPin = 11;
#else 
#ifdef ARDUINO_ESP32C3_DEV
const static uint8_t ms_scl = 9;
const static uint8_t ms_sda = 10;
const static uint8_t ms_5v_pin = 6;
const static uint8_t ms_0v_pin = 7;

const static uint8_t ledStripSigPin = 4;
#else
#ifdef ARDUINO_ARCH_ESP32
const static uint8_t ms_scl = 21;    // A5+
const static uint8_t ms_sda = 22;    // A4
const static uint8_t ms_5v_pin = 17; // A3
const static uint8_t ms_0v_pin = 16; // A2

const static uint8_t ledStripSigPin = 27;
#endif
#endif
#endif

//MotionSensor::CALIBRATION cal = { -960, 602, 816, 88, 5, -7 };
MotionSensor::CALIBRATION cal = { -683, 994, 1875, 34, 112, 0, MotionSensor::GAIN_UNDEFINED, MotionSensor::GAIN_UNDEFINED, MotionSensor::GAIN_UNDEFINED };
bool contPrintAccel = false;
bool contPrintGravity = false;

MotionSensor motionSensor;

void setup()
{
	int ret_val;

	pinMode(ms_5v_pin, OUTPUT);
	digitalWrite(ms_5v_pin, HIGH);
	pinMode(ms_0v_pin, OUTPUT);
	digitalWrite(ms_0v_pin, LOW);

	/* Start Serial at a speed (Baud rate) of 112500 Bytes per second */
	Serial.begin(112500);
	Serial.println("Motion Sensor Test Says Hello");

	EventMngr::getMngr()->configCallBack(myEventHandler);

	lastUpdateTime_ms = millis();


	while (motionSensor.init(&cal) != 0)
	{
		Serial.println("MS init fail");
	}

	TimerTickMngt::getTimerMngt()->update(0);
	Serial.println("Setup Complete");
}

void loop()
{
	unsigned long thisTime_ms = millis();
	unsigned long elapsedTime_ms = thisTime_ms - lastUpdateTime_ms;

	PerfMon::getPerfMon()->intervalStop();
	PerfMon::getPerfMon()->intervalStart();

	AutoUpdateMngr::getUpdater()->update(elapsedTime_ms);
		
	// Check for any input on the Serial Port (only relevant when connected to computer)
	handleSerialInput();

	lastUpdateTime_ms = thisTime_ms;
}

void myEventHandler(uint16_t eventId, uint8_t eventRef, uint16_t eventInfo)
{
	switch (eventId)
	{

	case SigGen::SIG_GEN_STARTED: // A Signal Generator has been started
		break;


	case SigGen::SIG_GEN_FINISHED: // A Signal Generator has finished 
	{
	}
	break;

	case TimerTickMngt::TIMER_EXPIRED: // A timer has expired - eventInfo from timerId
	{
	}
	break;

	case TimerTickMngt::TIMER_TICK_S: // Generated every second
		break;

	case TimerTickMngt::TIMER_TICK_10S: // Generated once every ten seconds
	//	PerfMon::getPerfMon()->print();
	//	Serial.println("calcAccel");
	//	motionSensor.calcAccel();
		break;

	case MotionSensor::MOTION_SENSOR_INIT_ERROR:
		Serial.print("Motion Sensor Init Failed: ");
		Serial.println(eventInfo);
		break;

	case MotionSensor::MOTION_SENSOR_READY:
		Serial.print("MOTION_SENSOR_READY: ");
		break;

	case MotionSensor::MOTION_SENSOR_RUNTIME_ERROR:
		Serial.println("Motion Sensor Error");
		break;

	case MotionSensor::MOTION_SENSOR_NEW_SMPL_RDY:
		motionSensor.motionProcessor.processSensorReadings(&motionSensor.sensorReadings);
		
		if(contPrintGravity) motionSensor.motionProcessor.processedReadings.gravity_m.printQ();
		if(contPrintAccel)  motionSensor.motionProcessor.processedReadings.linearAccel_m.printQ();
		break;

	case SigGen::SIG_GEN_STOPPED: // A Signal Generator has been stopped
	case TimerTickMngt::TIMER_TICK_UPDATE: // Generated every time fizzyMint is updated - could be every 500us (micro seconds) e.g. less than a millisecond
	case TimerTickMngt::TIMER_TICK_100MS: // Generated ten times a second
	case TimerTickMngt::TIMER_FREQ_GEN: // Generated a certain number of times a seconds
		break;
	}
}

/* This supports various management functions as shown below */
void handleSerialInput() {
	if (Serial.available()) {
		char c = Serial.read();
		Serial.println(c);

		switch (c) {
		case 'a': {
			contPrintAccel = !contPrintAccel;
		}
		break;

		case 'b': {
		}
		break;

		case 'c': {
			// Calibrates the motionSensor and stores result in EEPROM
			MotionSensor::CALIBRATION calibration;
			Serial.println("MotionSensor must be level and stationary with Z Axis facing up");
			Serial.println("Starting to calibrate");
			motionSensor.runOffsetSelfCalibrate(&calibration);
            Serial.println("Calibration complete");
		}
		break;

		case 'd': {
			// Displays Calibration
			Serial.println("Display Configured Calibration");
			motionSensor.printCalibration();
		}
		break;
		
		case 't': {
			Serial.setTimeout(10);
			Serial.println("Trim Calibration");
			Serial.read(); // blank space
			char axis = Serial.read();
			int16_t offset = Serial.parseInt();
			int newGain = Serial.parseInt();

			Serial.print(axis); Serial.print("\t");
			Serial.print(offset); Serial.print("\t");
			Serial.println(newGain);

			if (newGain >= 16) {
				Serial.println("Valid gain must be less than 16");
				break;
			}

			switch (axis) {
			case 'x': {
				uint8_t oldGain = motionSensor.mpu6050.getXFineGain();
				newGain = newGain << 4 | oldGain & 0x0f;
				motionSensor.mpu6050.setXAccelOffset(offset);
				motionSensor.mpu6050.setXFineGain(newGain);
			}
					break;
			case 'y': {
				uint8_t oldGain = motionSensor.mpu6050.getYFineGain();
				newGain = newGain << 4 | oldGain & 0x0f;
				motionSensor.mpu6050.setYAccelOffset(offset);
				motionSensor.mpu6050.setYFineGain(newGain);
			}
					break;
			case 'z': {
				uint8_t oldGain = motionSensor.mpu6050.getZFineGain();
				newGain = newGain << 4 | oldGain & 0x0f;
				motionSensor.mpu6050.setZAccelOffset(offset);
				motionSensor.mpu6050.setZFineGain(newGain);
			}
					break;

			default:
				Serial.println("Valid axis values are x, y or z");
				break;
			}
			
		}
		break;

		case 'f': {
			Serial.println("GetZ fine gain");
			int newGain = Serial.parseInt();
			int8_t gain = motionSensor.mpu6050.getZFineGain();
			Serial.println(gain);
			if (newGain != 0) {
				Serial.println(newGain);
				motionSensor.mpu6050.setZFineGain(newGain);
			}
		}
				break;
				

		case 'g': {
			contPrintGravity = !contPrintGravity;
		}
		break;

		case 'l': {
			// Configures the motionSensor rotation offset to believe it is level
			// Stores the configuration in EEPROM
			Serial.println("AutoLevel");
			motionSensor.motionProcessor.autoLevel();
		}
        break;

		case 'm': {
			int32_t ax, ay, az;
			motionSensor.findMaximumReadings(&ax, &ay, &az);
		}
		break;

		case 'n': {
			// Removes any rotation offset from the motionSensor
			Serial.println("Clear offset");
			motionSensor.motionProcessor.clearLevelOffset();
		}
				break;

		case 'r': {
			Serial.println("Reset Calibration");
			motionSensor.resetOffsetCalibration();
		}
				break;

		case 's': {
			Serial.println("Set Calibration");
			Serial.setTimeout(100);
			int axis_id = Serial.parseInt();
			int cal_val = Serial.parseInt();
			Serial.println(axis_id);
			Serial.println(cal_val);
			if (cal_val == 0) {
				Serial.println("cal_val bad");
				break;
			}

			if (axis_id == 0) {
				Serial.println(cal.accelXoffset);
				cal.accelXoffset = cal_val;
			}
			else if (axis_id == 1) {
				Serial.println(cal.accelYoffset);
				cal.accelYoffset = cal_val;
			}
			else if (axis_id == 2) {
				Serial.println(cal.accelZoffset);
				cal.accelZoffset = cal_val;
			}

			motionSensor.setCalibration(&cal);
		}
		break;

		}
	}
}
