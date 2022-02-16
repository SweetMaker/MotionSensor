#include <Wire.h>
#include <SweetMaker.h>
#include <FastLED.h>
#include <MotionSensor.h>

using namespace SweetMaker;

MotionSensor motionSensor;

void myEventHandler(uint16_t eventId, uint8_t src, uint16_t eventInfo);

static unsigned long lastUpdateTime_ms;

void setup()
{
	int ret_val;

	pinMode(A3, OUTPUT);
	digitalWrite(A3, HIGH);
	pinMode(A2, OUTPUT);
	digitalWrite(A2, LOW);

	/* Start Serial at a speed (Baud rate) of 112500 Bytes per second */
	Serial.begin(112500);
	Serial.println("Motion Sensor Test Says Hello");

	EventMngr::getMngr()->configCallBack(myEventHandler);

	lastUpdateTime_ms = millis();

	while (motionSensor.init() != 0)
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
//#define QAT
#ifdef YPR
		Serial.print(mssp.yawPitchRoll[0]);
		Serial.print(":");
		Serial.print(mssp.yawPitchRoll[1]);
		Serial.print(":");
		Serial.println(mssp.yawPitchRoll[2]);
#endif
#ifdef QAT
		Serial.print(mssp.q_float.w);
		Serial.print(":");
		Serial.print(mssp.q_float.x);
		Serial.print(":");
		Serial.print(mssp.q_float.y);
		Serial.print(":");
		Serial.println(mssp.q_float.z);
#endif
		break;

	case TimerTickMngt::TIMER_TICK_10S: // Generated once every ten seconds
										//		PerfMon::getPerfMon()->print();
		break;

	case IMotionSensor::MOTION_SENSOR_INIT_ERROR:
		Serial.print("Motion Sensor Init Failed: ");
		Serial.println(eventInfo);
		break;

	case IMotionSensor::MOTION_SENSOR_READY:
		Serial.print("MOTION_SENSOR_READY: ");
		break;

	case IMotionSensor::MOTION_SENSOR_RUNTIME_ERROR:
		Serial.println("Motion Sensor Error");
		break;

	case IMotionSensor::MOTION_SENSOR_NEW_SMPL_RDY:
		break;

	case SigGen::SIG_GEN_STOPPED: // A Signal Generator has been stopped
	case TimerTickMngt::TIMER_TICK_UPDATE: // Generated every time fizzyMint is updated - could be every 500us (micro seconds) e.g. less than a millisecond
	case TimerTickMngt::TIMER_TICK_100MS: // Generated ten times a second
	case TimerTickMngt::TIMER_FREQ_GEN: // Generated a certain number of times a seconds
		break;
	}
}

