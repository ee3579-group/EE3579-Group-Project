#ifndef Task_3_H
#define Task_3_H	
#include "Check_timer.h"


class Task_3 {
protected:
	PID_System PID;
	OpenLoopSystem OLS;
	IntervalCheckTimer Time;
	InterruptSpeedMeasure_SMA interrupt;

public:
	Task_3()
	{interrupt.setupSpeedMeasure(int_0);}								//Constructor for Hall effect sensor
	
	double rise_time;													//Global Variable for rist time
	double peak_time; 													//Global Variable for peak time
	double min_time; 													//Global Variable for minimum settling time
	double settling_time;												//Global Variable for settling time
	int peak_val;														//Global Variable for peak value
	int min_val;														//Global Variable for minimum settling value
	bool peak_val_flag;													//Peak Value Flag
	bool min_val_flag;													//Minimum Value Flag


	double rise_time_fun(int Target_Speed, int current_output)			
	{
		double g_rise_time;												//Initialise Local Variable to store Rise Time
		long timer = millis();											//Start timer
		if ((double)current_output >= (double)Target_Speed * 0.9)		//If current speed = 90% of target...
		{
			double rise_time = timer;
			g_rise_time = rise_time;
		}
		return g_rise_time;												//Return to allow external access
	}

	double peak_vals_fun(int Target_Speed, int& current_output)			//Function to record peak value, and time of incident
	{
		int current_RPM;
		Time.setInterCheck(50);											//Set Interval Check Time to 50ms
		long timer = millis();											//Start timer					
		int prev_RPM = current_output;									//Record RPM
		if (Time.isMinChekTimeElapsed() == true)						//If timeer check has elapsed
		{
			current_RPM = current_output;								//Save current RPM
		}
		if (current_RPM >= prev_RPM)									//If measured value is greater than previous value...
		{
			peak_val = current_RPM;										//Record Speed
		}
		peak_time = timer;												//Record Time
		peak_val_flag = true;											//Peak Value Flag set to true
		return peak_val;												//Return to allow external access
	}

	double min_vals_fun(int Target_Speed, int current_output)
	{
		int current_RPM;												//Variable to store current RPM
		long timer;														//Blank timer variable
		Time.setInterCheck(50);											//Set Interval Check Time to 50ms
		if (peak_val_flag)												//If peak value flag is true...
		timer = millis();												//Start timer					
		int RPM = current_output;										//Obtain Current RPM
		int prev_RPM = RPM;												//Obtain Current RPM
		if (Time.isMinChekTimeElapsed() == true)						//If check time has elapsed
		{
			current_RPM = current_output;								//Overwrite current RPM with measured value
		}																		
		if (current_RPM < prev_RPM)										//If measured value is less than previous value...
		min_val = current_RPM;											//Record Value
		min_time = timer;												//Record Time of ocurrance
		return min_val;													//Return to allow external access		
	}

	double settling_time_fun(int Target_Speed, int current_output)
	{
		long timer = millis();											//Start timer	
		int Current_RPM = current_output;								//Obtain Current RPM
		int Target_RPM = Target_Speed;									//Obtain Target RPM
			//If value is within 10% tollerances, and minumum flag is tripped
			if (((double)Current_RPM == 1.1*(double)Target_RPM || (double)Current_RPM == 0.9*(double)Target_RPM) && min_val_flag)
			return settling_time;										//Return to allow external access
	}

	void print_task_3_vals(int Target_Speed, int current_output)		
	{
		int inp_target_speed = Target_Speed;
		int inp_current_output = current_output;
		Serial.println("Rise Time: ");
		Serial.println(rise_time_fun(inp_target_speed, inp_current_output));
		Serial.println("Peak Value: ");
		Serial.println(peak_vals_fun(inp_target_speed, inp_current_output));
		Serial.println("Time Of Last Peak Value: ");
		Serial.println(peak_time);
		Serial.println("Minimum Settling Value: ");
		Serial.println(min_vals_fun(inp_target_speed, inp_current_output));
		Serial.println("Minimum Settling Time: ");
		Serial.println(min_time);
		Serial.println("Settling Time: ");
		Serial.println(settling_time_fun(inp_target_speed, inp_current_output));	
	}
};
#endif
