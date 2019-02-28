#ifndef OPENLOOPSYSTEM_H
#define OPENLOOPSYSTEM_H

#include <Basic_Input.h>
#include <PushButton.h>
#include <InterruptBasedSpeedMeasure.h>
#include <DCmotor.h>
#include <SystemControlUnitOpenLoop.h>

class OpenLoopSystem{
	protected:
		// all pushbuttons
	inputs motor_pushbuttons;
	
	// the potentiometer input represents the target RPM in a given range
    in_analog motor_potentiometer;
	// these are the ranges for the target RPM, correponding to potentiometer 0 and 1023
	int min_RPM, max_RPM;
	
	// the motor
	HBridgeDCmotor motor;

	// the speedometer (it computes "moving-averaged" speed)
	InterruptSpeedMeasure_SMA speedometer;
	
	// open loop controller 
	SpeedControl0OpenLoop controller;
	
	// se to true to print to screen everything (off by defult)
	bool verbose;

	// these keep track of the internal variables are all the 
	int current_RPM, curr_PWM, target_RPM; 

	// default valus
	static const int DEFAULT_MIN_RPM=1000;
	static const int DEFAULT_MAX_RPM=8000;
	
		void update_target_RPM()
	{
		int readval;
		bool success;

		if(verbose)
			Serial.println("Checking Target RPM...");
			
	// verify the  new input. This is written to "readval"
		success=motor_potentiometer.read_input(readval);
		
		if(success)
		{
			target_RPM=map(readval, 0, 1023, min_RPM, max_RPM); // map (value, lower bound of val, upper bound of val, lower bound of target val, upper bound of target val)
			if(verbose)
			{
				Serial.print("Target RPM is ");
				Serial.println(target_RPM);
			}
		}
		if(verbose)
			Serial.println("");
			
	void OpenLoopControlAndUpdateMotorPMW()
	{
		int actualPWM;
		actualPWM = map(target_RPM, min_RPM, max_RPM, 0, 255);
		motor.setSpeedPWM(actualPWM);
	}		
	
	void checkButtonsAndUpdateMotor()
	{
		command_list_enum in_smpl_cmd;
		bool success;

		//check the buttons; any command found is written to in_smpl_cmd
		success=motor_pushbuttons.check_n_get_command(in_smpl_cmd);

		if(success)
		{
			switch (in_smpl_cmd){
			case switch_on:
				if(verbose)
					Serial.println("Motor Start");
				motor.start();
				break;
			case switch_off:
				if(verbose)
					Serial.println("Motor Stop");
				motor.stop();
				break;
			case change_spin_dir:
				if(verbose)
					Serial.println("Motor Reverse");
				motor.changedir();
				break;
			default:
				if(verbose)
					Serial.println("Unknown Button Pressed");
			}

			// this is the actual PWM following the button press
			curr_PWM = motor.getSpeedPWM();

			if(verbose)
				Serial.println("");
		}
	}
	
	void print_internal_vals()
	{
		if(verbose)
		{
			Serial.print("RPM = ");
			Serial.print(current_RPM);
			Serial.print("; target RPM = ");
			Serial.print(target_RPM);
			Serial.print("; PWM = ");
			Serial.print(curr_PWM);
			Serial.println (".");
		}
	}
	
	public:
		OpenLoopSystem(){} // default constructor
		OpenLoopSystem()
		{
		min_RPM=DEFAULT_MIN_RPM;
		max_RPM=DEFAULT_MAX_RPM;
		verbose=false;
		target_RPM=DEFAULT_MIN_RPM;

		// by default the motor is off;
		motor.stop();
		curr_PWM = motor.getSpeedPWM();

		// set the motor so that is does/doesnot jump-start after being stopped
		motor.set_jumpstart(false);


		// can set the size of the moving average (otherwise, the default size is 10)
		int smasize=5;
		speedometer.setupSMAarray(smasize);
		}
		
	// this sets the interval between: 
	//two speed-adjustments; two taget-speed checks; two button checks.
	// if this funciton is not called, default values are used
	void set_interval_vals(int adjust_speed_interval_ms, int target_speed_check_interval_ms, int buttons_check_interval_ms)
	{
		controller.set_interval_vals(adjust_speed_interval_ms,target_speed_check_interval_ms,buttons_check_interval_ms);
	}

	// this is to add the pin and "label" of a new push button
	void add_buttonpin_and_label(int butpinnum, command_list_enum butlabel)
	{
		// temporary variable of type in_push_button
		in_push_button new_button(butpinnum, butlabel);
		
		// the temp button is now copied into motor_pushbuttons
		motor_pushbuttons.add_in_push_button(new_button);
	}

	void setup_HBridgeDCmotor(int motorpin, int directionpin) {motor.setup_HBridgeDCmotor(motorpin,directionpin);}

	// potentiometer-in and ranges for the target RPM (correponding to potentiometer 0 and 1023)
	void add_potentiometer(int analogpinnum,  int min_RPM_val, int max_RPM_val)
	{
		motor_potentiometer.setup_in_analog(analogpinnum);
		
		// some checks on set the potentiometer range
		min_RPM_val=abs(min_RPM_val);
		max_RPM_val=abs(max_RPM_val);
		min_RPM=min(min_RPM_val, max_RPM_val);
		max_RPM=max(min_RPM_val, max_RPM_val);
	}
	// overloaded version that takes only the pin and uses default range values
	void add_potentiometer(int analogpinnum){motor_potentiometer.setup_in_analog(analogpinnum);}

	// setup the speedometer; this does not require external pullup (it is enabledinternally)
	// note: it assumes the default values of "magnets along a circle" (i.e. 6);
	void setupSpeedMeasure(ArduinoInterruptNames in_interrname){speedometer.setupSpeedMeasure(in_interrname);}

	// to enable the verbose mode (if not used, it is off by default)
	void setVerbose(bool verbose_mode)
	{
		verbose=verbose_mode;
		if(verbose)
			Serial.begin(9600);
	}
	
	void CheckInputsAndControlMotor()
	{
		
		// check buttons and react to that;
		if(controller.isTimeToCheckInputButtons())
			checkButtonsAndUpdateMotor();
		
		// check the target speed
		if(controller.isTimeToCheckTargetSpeedChange())
			update_target_RPM();
		
		// measure current speed and adjust the PWM
		if(controller.isTimeToAdjustSpeed())
		{
			// measure current RPM
			current_RPM = speedometer.getRPMandUpdate();
			
			if(verbose)
			{
				Serial.println("Performing Speed Adjustment");
				Serial.println("Before Speed Adjustment");
				print_internal_vals();
				Serial.println("");
			}

			// adjust PWM if motor has not been stopped 
			if(motor.isStarted())
			{
				// read current PWM off the motor;
				// perform closed loop adjustment of PWM; set RPM value to the motor
				OpenLoopControlAndUpdateMotorPMW();
				
				if(verbose)
				{
					Serial.println("After speed adjustmen");
					print_internal_vals();
					Serial.println("");
				}
			}//if(motor.isStarted())
		}//if(controller.isTimeToAdjustSpeed())
	}

};

#endif
