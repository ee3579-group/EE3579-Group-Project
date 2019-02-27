#ifndef RPMSensor_h
#define RPMSensor_h

#include <Arduino.h>
#include "IntervalCheckTimer.h"
#include "InterruptBasedSpeedMeasure.h"

class RPMSensor {
protected:

	// speed and timer objects
	InterruptSpeedMeasure rotation_count;
	IntervalCheckTimer speed_check;

	boolean interrupt_enabled;	// flag for interrupt pin
	boolean interval_enabled;	// flag to check if time has been specified

	double RPM;	// variable for measured speed in RPM

public:
	// constructor, initialises flags to false
	RPMSensor() 
	{
		interrupt_enabled = false;
		interval_enabled = false;
	}

	// enable interrupt pin (int_0 for pin2, int_1 for pin3)
	void setupInterrupt(ArduinoInterruptNames inter_num)
	{
		rotation_count.setupSpeedMeasure(inter_num);
		interrupt_enabled = true;
	}

	// setup time between speed measurements (in ms)
	void setupInterval(int inp_interval_ms)
	{
		speed_check.setInterCheck(inp_interval_ms);
		interval_enabled = true;
	}

	// use interrupt RPM measurement if parameters enabled
	// display values to monitor if valid
	void measureSpeed()
	{
		if ((interrupt_enabled) && (interval_enabled))
		{
			if (speed_check.isMinChekTimeElapsedAndUpdate())
			{
				RPM = rotation_count.getRPMandUpdate();
				if (RPM >= 0)
				{
					Serial.print("Measured Speed = ");
					Serial.print(RPM);
					Serial.print(" RPM");
					Serial.println("");
				}
				else
				{
					Serial.println("Speed measurement failed");
				}
			}
		}
	}

	double getRPM() { return RPM; }

	/*void displaySpeed()
	{
		if (speed_check.isMinChekTimeElapsedAndUpdate())
		{
			if (RPM >= 0)
			{
				Serial.print("Measured Speed = ");
				Serial.print(RPM);
				Serial.print(" RPM");
				Serial.println("");
			}
			else
			{
				Serial.println("Speed measurement failed");
			}
		}
	}*/

};



#endif