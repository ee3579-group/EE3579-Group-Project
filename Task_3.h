#ifndef Task_3_H
#define Task_3_H	
#include "Check_timer.h"

	class Task_3{
	protected:
	PID_System PID;
	OpenLoopSystem OLS;
	IntervalCheckTimer Time;
	
	public:
		double rise_time;															//Global Variable for rist time
		double peak_time; 															//Global Variable for peak time
		double min_time; 															//Global Variable for peak time
		double settling_time;
		int peak_val;																//Global Variable for peak value
		int min_val;																//Global Variable for peak value
		bool peak_val_flag;
		bool min_val_flag;
		
		double rise_time_fun()														//Function To Return Value at 90% of target
		{
			unsigned long timer = millis();											//Start timer
			int Current_output = PID.getRPM();									//Obtain Current PWM output
			int Target_Speed = PID.getTargetSpeed();								//Obtain Target speed
			if (PID.PID_PWM && (current_output = Target_Speed * 0.9))				//If pid-pwm is called and 90% target...
			double rise_time = timer; 												//Save time
			return rise_time;														//Return to allow external access
		}

		double peak_vals_fun()														//Function to record peak value, and time of incident
		{
			int current_RPM;
			Time.IntervalCheckTimer(50);
			unsigned long timer = millis();											//Start timer					
			int RPM = OLS.getRPM();													//Obtain Current RPM
			int prev_RPM = RPM;														//Record RPM
			if(Time.isMinChekTimeElapsed()==true)
			{
				current_RPM = OLS.getRPM();	
			}										
			if (PID.PID_PWM && current_RPM > prev_RPM)								//If measured value is greater than previous value...
			peak_val = currentRPM;													//Record Speed
			peak_time = timer;														//Record Time
			peak_val_flag = true;													//Peak Value Flag is tripped
			return peak_val;														//Return to allow external access										
		}
		
		double min_vals_fun()
		{
			int current_RPM;
			Time.IntervalCheckTimer(50);
			if(peak_val_flag)
			unsigned long timer = millis();										//Start timer					
			int RPM = OLS.getRPM();												//Obtain Current RPM
			int prev_RPM = RPM;													//Record RPM
			if(Time.isMinChekTimeElapsed()==true)
			{
				current_RPM = OLS.getRPM();	
			}									//Do it again									
			if (current_RPM < prev_RPM)											//If measured value is less than previous value...
			min_val = currentRPM;												//Record Value
			min_time = timer;													//Record Time
			return min_val;														//Return to allow external access		
		}
		
		double settling_time_fun()
		{
			unsigned long timer = millis();										//Start timer	
			int Current_RPM = OLS.getRPM();										//Obtain Current RPM
			int Target_RPM = PID.getTargetSpeed();								//Obtain Target RPM
			//If value is within 10% tollerances, and minumum flag is tripped
			if ((Current_RPM = 1.1*Target_RPM || Current_RPM = 0.9*Target_RPM )&& min_val_flag)
			return settling_time;												//Return to allow external access
		}
		
		void print_task_3_vals()												//Function to print to serial monitor
		{
			rise_time_fun();
			peak_vals_fun();
			min_vals_fun();
			settling_time_fun();
			Serial.println("Rise Time: ");
			Serial.print(rise_time);
			Serial.println("Peak Value: ");
			Serial.print(peak_val);
			Serial.println("Time Of Last Peak Value: ");
			Serial.print(peak_time);
			Serial.println("Minimum Settling Value: ");
			Serial.print(min_val);
			Serial.println("Minimum Settling Time: ");
			Serial.print(min_time);
			Serial.println("Settling Time: ");
			Serial.print(settling__time);			
		}
	};

#endif
