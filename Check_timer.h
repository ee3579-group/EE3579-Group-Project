
#ifndef Controller_Act2
#define Controller_Act2
#ifdef COMPILE_CPP_NOT_ARDUINO
#include "..\All_Arduino_or_Cpp_symboldefines\All_Arduino_or_Cpp_symboldefines.h"
// this is only needed for the shine_combination_commands
#else
#include <Arduino.h>
//#include "..\Act2\Act2.h"
//#include "..\Fan_operation\Fan_operation.h"
#include "..\IntervalCheckTimer\IntervalCheckTimer.h"
#endif

// this library implements the "control unit" of my system
class controller_act2 {
protected:

	// default value for the interval between measuremtns (in millisec)
	static const int default_check_interval_ms = 200;
	unsigned long check_interval_ms, last_check_time;
	//Sensor_Units Sensing;
	//fan_operation Fans;
	//int mode;
	//int fan_speed, fan_speed_converted;
public:
	// default constructor:
	controller_act2()
	{
		check_interval_ms = default_check_interval_ms;
		last_check_time = 0;
	}

	// constructor with argiuments:
	/*controller_act2(int in_check_interval_ms, int Up_pin_button1, int Up_pin_button2, int Up_pin_button3, int Up_pin_potentiometer, int pwmpin)
	{
		Sensing.Sensor_setup(Up_pin_button1, Up_pin_button2, Up_pin_button3, Up_pin_potentiometer);
		//Fans.Fan_Setup(pwmpin, pwmpin2);
		setup_controller(in_check_interval_ms);
		last_check_time = 0;
	}*/

	// set up with argiuments:
	void setup_controller(int in_check_interval_ms)
	{
		check_interval_ms = abs(in_check_interval_ms);
	}

	// verifies if it's time to take a new input measurement
	bool isTimeToTakeMeasurement()
	{
		// check current time
		unsigned long current_time = millis();
		if ((current_time - last_check_time) >= check_interval_ms)
		{
			return true;
		}
		else
		{
			return false;
		}
	}

	// issues an action, depending on the PushButton
	/*void issueSimpleCommand()
	{
		// avoid making decisions if it's too early
		if (isTimeToTakeMeasurement())
		{
			fan_speed = Sensing.Potentiometer_input();
			mode = Sensing.check_button();
			Serial.print("Fan speed= ");
			Serial.println(fan_speed);
			Serial.print("Button pressed= ");
			Serial.println(mode);
			/*
			 //Serial.print(mode);
		  //fan_speed_converted=127+(fan_speed*128)/10;
			 fan_speed_converted=(fan_speed*255)/10;

			 //Serial.print(fan_speed);
			 //Serial.print(", ");
			 //Serial.println(fan_speed_converted);
			 Fans.turn_on_fan(mode,fan_speed_converted);

		  // take the current time as the most recent when a successful measurement was done
		  last_check_time=millis();
		  // decide which action should be performed
		  
		}

	}

	void issueComplexCommand()
	{
		 if( isTimeToTakeMeasurement() )
	  {
	  // take the current time as the most recent when a successful measurement was done
	  // decide which action should be performed

		  fan_speed = Sensing.user_input();
		 mode = Sensing.check_button();
		 fan_speed_converted=(fan_speed*255)/10;

		  if(mode == true)
		  {
			Fans.set_speed(fan_speed_converted);
		  }
		  else
		  {
			 Fans.set_both_speed(fan_speed_converted);
		  }


	  last_check_time=millis();

	  }
	  
	}*/


};
#endif
