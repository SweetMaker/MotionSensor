#include <Wire.h>
#include <SweetMaker.h>
#include <MotionSensor.h>

using namespace SweetMaker;

MotionSensor motionSensor;

void myEventHandler(uint16_t eventId, uint8_t src, uint16_t eventInfo);

static unsigned long lastUpdateTime_ms;

#ifdef ARDUINO_ARCH_AVR
const static uint8_t ms_scl = 19;    // A5+
const static uint8_t ms_sda = 18;    // A4
const static uint8_t ms_5v_pin = 17; // A3
const static uint8_t ms_0v_pin = 16; // A2

const static uint8_t ledStripSigPin = 11;
#endif

#ifdef ARDUINO_ARCH_ESP32
const static uint8_t ms_scl = 21;    // A5+
const static uint8_t ms_sda = 22;    // A4
const static uint8_t ms_5v_pin = 17; // A3
const static uint8_t ms_0v_pin = 16; // A2

const static uint8_t ledStripSigPin = 27;
#endif

//MotionSensor::CALIBRATION cal = { -960, 602, 816, 88, 5, -7 };
MotionSensor::CALIBRATION cal = { -655, 914, 1842, 29, 115, -1 };
bool contCalcAccel = false;

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
		if(contCalcAccel)
    		motionSensor.calcAccel();
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
			// Calibrates the motionSensor and stores result in EEPROM
			Serial.println("Calculate acceleration");
			motionSensor.gravity_m.printQ();
			motionSensor.linearAccel_m.printQ();
			motionSensor.calcAccel();
		}
		break;

		case 'b': {
			contCalcAccel = !contCalcAccel;
		}
		break;

		case 'c': {
			// Calibrates the motionSensor and stores result in EEPROM
			MotionSensor::CALIBRATION calibration;
			Serial.println("MotionSensor must be level and stationary");
			Serial.println("Starting to calibrate");
			motionSensor.runSelfCalibrate(&calibration);
            Serial.println("Calibration complete");
		}
		break;

		case 'l': {
			// Configures the motionSensor rotation offset to believe it is level
			// Stores the configuration in EEPROM
			Serial.println("AutoLevel");
			motionSensor.autoLevel();
		}
		break;

		case 'z': {
			// Removes any rotation offset from the motionSensor
			Serial.println("Clear offset");
			motionSensor.clearOffsetRotation();
		}
		break;
		}
	}
}
