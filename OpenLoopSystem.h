#ifndef OPENLOOPSYSTEM_H
#define OPENLOOPSYSTEM_H

#include <Basic_Input.h>
#include <PushButton.h>
#include <InterruptBasedSpeedMeasure.h>
#include <DCmotor.h>
#include <SystemControlUnitOpenLoop.h>

class OpenLoopSystem {
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
	SpeedControlOpenLoop controller;

	// se to true to print to screen everything (off by defult)
	bool verbose;

	// these keep track of the internal variables are all the 
	int current_RPM, curr_PWM, target_RPM;

	// default valus
	static const int DEFAULT_MIN_RPM = 60;
	static const int DEFAULT_MAX_RPM = 200;

	void update_target_RPM()
	{
		int readval;
		bool success;

		if (verbose)
			Serial.println("Checking Target RPM...");

		// verify the  new input. This is written to "readval"
		success = motor_potentiometer.read_input(readval);

		if (success)
		{
			target_RPM = map(readval, 0, 1023, min_RPM, max_RPM); // map (value, lower bound of val, upper bound of val, lower bound of target val, upper bound of target val)
			if (verbose)
			{
				Serial.print("Target RPM is ");
				Serial.println(target_RPM);
			}
		}
		if (verbose)
			Serial.println("");

	}
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
		success = motor_pushbuttons.check_n_get_command(in_smpl_cmd);

		if (success)
		{
			switch (in_smpl_cmd) {
			case switch_on:
				if (verbose)
					Serial.println("Motor Start");
				motor.start();
				break;
			case switch_off:
				if (verbose)
					Serial.println("Motor Stop");
				motor.stop();
				break;
			case change_spin_dir:
				if (verbose)
					Serial.println("Motor Reverse");
				motor.changedir();
				break;
			default:
				if (verbose)
					Serial.println("Unknown Button Pressed");
			}

			// this is the actual PWM following the button press
			curr_PWM = motor.getSpeedPWM();

			if (verbose)
				Serial.println("");
		}
	}

	void print_internal_vals()
	{
		if (verbose)
		{
			Serial.print("RPM = ");
			Serial.print(current_RPM);
			Serial.print("; target RPM = ");
			Serial.print(target_RPM);
			Serial.print("; PWM = ");
			Serial.print(curr_PWM);
			Serial.println(".");
		}
	}

	public:
		OpenLoopSystem()
		{
			min_RPM = DEFAULT_MIN_RPM;
			max_RPM = DEFAULT_MAX_RPM;
			verbose = false;
			target_RPM = DEFAULT_MIN_RPM;

			// by default the motor is off;
			motor.stop();
			curr_PWM = motor.getSpeedPWM();

			// set the motor so that is does/doesnot jump-start after being stopped
			motor.set_jumpstart(false);


			// can set the size of the moving average (otherwise, the default size is 10)
			int smasize = 5;
			speedometer.setupSMAarray(smasize);
		}

		// this sets the interval between: 
		//two speed-adjustments; two taget-speed checks; two button checks.
		// if this funciton is not called, default values are used
		void set_interval_vals(int adjust_speed_interval_ms, int target_speed_check_interval_ms, int buttons_check_interval_ms)
		{
			controller.set_interval_vals(adjust_speed_interval_ms, target_speed_check_interval_ms, buttons_check_interval_ms);
		}

		// this is to add the pin and "label" of a new push button
		void add_buttonpin_and_label(int butpinnum, command_list_enum butlabel)
		{
			// temporary variable of type in_push_button
			in_push_button new_button(butpinnum, butlabel);

			// the temp button is now copied into motor_pushbuttons
			motor_pushbuttons.add_in_push_button(new_button);
		}

		void setup_HBridgeDCmotor(int motorpin, int directionpin) { motor.setup_HBridgeDCmotor(motorpin, directionpin); }

		// potentiometer-in and ranges for the target RPM (correponding to potentiometer 0 and 1023)
		void add_potentiometer(int analogpinnum, int min_RPM_val, int max_RPM_val)
		{
			motor_potentiometer.setup_in_analog(analogpinnum);

			// some checks on set the potentiometer range
			min_RPM_val = abs(min_RPM_val);
			max_RPM_val = abs(max_RPM_val);
			min_RPM = min(min_RPM_val, max_RPM_val);
			max_RPM = max(min_RPM_val, max_RPM_val);
		}
		// overloaded version that takes only the pin and uses default range values
		void add_potentiometer(int analogpinnum) { motor_potentiometer.setup_in_analog(analogpinnum); }

		// setup the speedometer; this does not require external pullup (it is enabledinternally)
		// note: it assumes the default values of "magnets along a circle" (i.e. 6);
		void setupSpeedMeasure(ArduinoInterruptNames in_interrname) { speedometer.setupSpeedMeasure(in_interrname); }

		// to enable the verbose mode (if not used, it is off by default)
		void setVerbose(bool verbose_mode)
		{
			verbose = verbose_mode;
			if (verbose)
				Serial.begin(9600);
		}

		void CheckInputsAndControlMotor()
		{

			// check buttons and react to that;
			if (controller.isTimeToCheckInputButtons())
				checkButtonsAndUpdateMotor();

			// check the target speed
			if (controller.isTimeToCheckTargetSpeedChange())
				update_target_RPM();

			// measure current speed and adjust the PWM
			if (controller.isTimeToAdjustSpeed())
			{
				// measure current RPM
				current_RPM = speedometer.getRPMandUpdate();

				if (verbose)
				{
					Serial.println("Performing Speed Adjustment");
					Serial.println("Before Speed Adjustment");
					print_internal_vals();
					Serial.println("");
				}

				// adjust PWM if motor has not been stopped 
				if (motor.isStarted())
				{
					// read current PWM off the motor;
					// perform closed loop adjustment of PWM; set RPM value to the motor
					OpenLoopControlAndUpdateMotorPMW();

					if (verbose)
					{
						Serial.println("After speed adjustmen");
						print_internal_vals();
						Serial.println("");
					}
				}//if(motor.isStarted())
			}//if(controller.isTimeToAdjustSpeed())
		}

		int getRPM()
		{
			current_RPM = speedometer.getRPMandUpdate();
			return current_RPM;
		}

		void setPWM(int PWM_val)
		{
			motor.setSpeedPWM(PWM_val);
		}

		void CheckInputsAndControlMotor_1()
		{

			// check buttons and react to that;
			if (controller.isTimeToCheckInputButtons())
				checkButtonsAndUpdateMotor();


			// measure current speed and adjust the PWM
			
		}

	};

	class PID_System {
	protected:
		double ref_Kp;                      //P from Input
		double ref_Ki;                      //I from Input
		double ref_Kd;                      //D from Input
		double ref_contr_inter_time_ms;     //Refference Timer to scale PID parameters

		double Kp;                          //P used in calculations
		double Ki;                          //I used in calculations
		double Kd;                          //D used in calculations
		double Ku = 0;                        //Ultimate gain (lowest possible value for constant oscillation)
		double Pu = 0;                        //Period of above oscillations
		double PID_PWM_Min, PID_PWM_Max;    //PWM range
		unsigned long last_control_ms;      //Stores last time
		double previous_error;              //stores last error value
		double error_sum;                   //Stores sum of previous errors
		double static const REF_CONTR_INTER_MS = 1000;
		bool enable_flag;                   //Enabled Flag
		bool echopidcontrol;                //Echo PID Controll Flag


		void set_default_parameters()
		{
			ref_Kp = 0.7;
			ref_Ki = 0.45;
			ref_Kd = 0.5;
			ref_contr_inter_time_ms = REF_CONTR_INTER_MS;
			PID_PWM_Min = 0.0;
			PID_PWM_Max = 255.0;
			last_control_ms = millis();
			previous_error = 0.0;
			error_sum = 0.0;
			echopidcontrol = false;
			enable_flag = true;             //Set Enable Flag to True
		}

	public:

		basic_speed_PID() { set_default_parameters(); }
		basic_speed_PID(double inp_ref_Kp, double inp_ref_Ki, double inp_ref_Kd, double inp_PIDoutMin, double inp_PIDoutMax, int inp_ref_contr_inter_time_ms = REF_CONTR_INTER_MS)
		{
			set_default_parameters();
			set_gain_parameters(inp_ref_Kp, inp_ref_Ki, inp_ref_Kd);
			set_ref_control_interval_ms(inp_ref_contr_inter_time_ms);
			set_bounds(inp_PIDoutMin, inp_PIDoutMax);
		}

		void set_gain_parameters(double inp_ref_kp, double inp_ref_ki, double inp_ref_kd)
		{
			ref_Kp = inp_ref_kp;            //Overwrite Global Variable with local Input
			ref_Ki = inp_ref_ki;            //Overwrite Global Variable with local Input
			ref_Kd = inp_ref_kd;            //Overwrite Global Variable with local Input
			enable_flag = true;             //Set Enable Flag to True
		}
		void set_ref_control_interval_ms(int inp_ref_contr_inter_time_ms)
		{
			ref_contr_inter_time_ms = inp_ref_contr_inter_time_ms;
		}
		void set_bounds(double inp_PID_PWM_Min, double inp_PID_PWM_Max)
		{
			PID_PWM_Min = inp_PID_PWM_Min;          //Overwrite Global Variable with local Input
			PID_PWM_Max = inp_PID_PWM_Max;          //Overwrite Global Variable with local Input
		}

		void Ziegler_Nichols_tuning()   //Primary Method of Acheiving Critical Response
		{
			ref_Kp = Ku / 1.7;                //Set Global Proportional Gain as Ultimate Gain * 1.7
			ref_Ki = Pu / 2;                  //Set Global Intergral Gain as Ultimate Period/2
			ref_Kd = Pu / 8;                  //Set Global Differential Gain as Ultimate Period/8
			enable_flag = true;              //Set Enable Flag to True
		}

		void Tyreus_Luyben_Tuning()      //Alternative Method of Acheiving Critical Response
		{
			ref_Kp = Ku / 2.2;                //Set Global Proportional Gain as Ultimate Gain/2.2
			ref_Ki = 2.2*Pu;                //Set Global Intergral Gain as Ultimate Period * 2.2
			ref_Kd = 6.3 / Pu;                //Set Global Differential Gain as 6.3/Ultimate Period
			enable_flag = true;              //Set Enable Flag to True
		}

		double PID_PWM(double target_speed, double curr_speed)
		{
			unsigned long current_time = millis();
			double output = 0.0;
			unsigned long tempcontrol_interval = (current_time - last_control_ms);
			int contr_inter_time_ms = (int)tempcontrol_interval;

			double error = target_speed - curr_speed;
			double contr_inter_time_ratio = ref_contr_inter_time_ms / contr_inter_time_ms;
			double error_diff = error - previous_error;
			double error_derivative = error_diff / ((double)contr_inter_time_ms);

			error_sum += (error*contr_inter_time_ratio);
			Kp = ref_Kp;
			Ki = ref_Ki;
			Kd = ref_Kd;

			output = Kp * error + Ki * error_sum + Kd * error_derivative;

			if (output > PID_PWM_Max)
				output = PID_PWM_Max;
			else
				if (output < PID_PWM_Min)
					output = PID_PWM_Min;

			if (echopidcontrol)
			{
				Serial.println();
				Serial.print("control interval ms ");
				Serial.println(contr_inter_time_ms);
				Serial.print("error ");
				Serial.println(error);
				Serial.print("cumulative error ");
				Serial.println(error_sum);
				Serial.print("error derivative ");
				Serial.println(error_derivative);
				Serial.print(" PWM output: ");
				Serial.println(output);
				Serial.println();
			}

			last_control_ms = current_time;
			previous_error = error;
			return output;
		}

		double GetKp() { return Kp; }           //By Calling this function, Kp may be obtained within a .ino file
		double GetKi() { return Ki; }           //By Calling this function, Ki may be obtained within a .ino file
		double GetKd() { return Kd; }           //By Calling this function, Kd may be obtained within a .ino file

		void set_echopidcontrol(bool inp_echo) { echopidcontrol = inp_echo; }
		bool get_echopidcontrol() { return echopidcontrol; }

		void reset_pidcontrol()
		{
			last_control_ms = millis();
			previous_error = 0.0;
			error_sum = 0.0;
		}

	};

#endif
