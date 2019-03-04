#ifndef OPENLOOPSYSTEM_H
#define OPENLOOPSYSTEM_H

#include <Basic_Input.h>
#include <PushButton.h>
#include <InterruptBasedSpeedMeasure.h>
#include <DCmotor.h>
#include <SystemControlUnitOpenLoop.h>
#include <Closed_Loop_PID.h>
#include <Check_timer.h>

int target_val;

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

	PID pid_motor;

	// PID control

	int MAX_RPM = 6000;

	InterruptSpeedMeasure_SMA interrupt;

	// the speedometer (it computes "moving-averaged" speed)
	InterruptSpeedMeasure_SMA speedometer;

	// open loop controller 
	SpeedControlOpenLoop controller;

	// se to true to print to screen everything (off by defult)
	bool verbose;

	// these keep track of the internal variables are all the 
	int curr_PWM, target_RPM;

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
			int RPM = getRPM();

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
			case thirty_percent:
				if (verbose) {
					Serial.println("Motor speed 30%");
					target_val = MAX_RPM * 0.3;
				}
				

				break;
			case fifty_percent:
				if (verbose) {
					Serial.println("Motor speed 50%");
					target_val = 3000;
				}
				//PWM_val = pid_motor.PID_PWM(3000, RPM);
				//motor.setSpeedPWM(PWM_val);
				
				break;
			case eighty_percent:
				if (verbose) {
					Serial.println("Motor speed 80%");
					target_val = 4800;
					//PWM_val = pid_motor.PID_PWM(4800, RPM);
					//motor.setSpeedPWM(PWM_val);
				}
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
			interrupt.setupSpeedMeasure(int_0);
			// by default the motor is off;
			motor.stop();
			curr_PWM = motor.getSpeedPWM();

			// set the motor so that is does/doesnot jump-start after being stopped
			motor.set_jumpstart(false);


			// can set the size of the moving average (otherwise, the default size is 10)
			int smasize = 5;
			speedometer.setupSMAarray(smasize);
		}

		double current_RPM;

		int get_target()
		{
			return target_val;
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

		double getRPM()
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
		long last_control_ms;      //Stores last time
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
			long current_time = millis();
			double output = 0.0;
			long tempcontrol_interval = (current_time - last_control_ms);
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
				/*Serial.println();
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
				Serial.println();*/
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

	/*class Task_3 {
	protected:
		PID_System PID;
		OpenLoopSystem OLS;
		IntervalCheckTimer Time;
		InterruptSpeedMeasure_SMA interrupt;

	public:
		Task_3()
		{
			interrupt.setupSpeedMeasure(int_0);
		}
		double rise_time;															//Global Variable for rist time
		double peak_time; 															//Global Variable for peak time
		double min_time; 															//Global Variable for peak time
		double settling_time;
		int peak_val;																//Global Variable for peak value
		int min_val;																//Global Variable for peak value
		bool peak_val_flag;
		bool min_val_flag;
		bool settling_flag;


		double rise_time_fun(int Target_Speed, int current_output)														//Function To Return Value at 90% of target
		{
			double g_rise_time;
			long timer = millis();											//Start timer
			//int current_output = OLS.getRPM();									//Obtain Current PWM output
			//int Target_Speed = OLS.get_target();								//Obtain Target speed
			if ((double)current_output >= (double)Target_Speed * 0.9)				//If pid-pwm is called and 90% target...
			{
				double rise_time = timer;
				g_rise_time = rise_time;
			}
			return g_rise_time;														//Return to allow external access
		}

		double peak_vals_fun(int Target_Speed, int current_output)														//Function to record peak value, and time of incident
		{
			int current_RPM;
			Time.setInterCheck(50);
			long timer = millis();											//Start timer					
															//Obtain Current RPM
			int prev_RPM = current_output;														//Record RPM
			if (Time.isMinChekTimeElapsed() == true)
			{
				current_RPM = current_output;
			}
			if (current_RPM >= prev_RPM)						//If measured value is greater than previous value...
			{
				peak_val = current_RPM;													//Record Speed
			}
			
			peak_time = timer;														//Record Time
			peak_val_flag = true;													//Peak Value Flag is tripped

			return peak_val;
		}

		double min_vals_fun(int Target_Speed, int current_output_1)
		{
			int current_RPM;
			long timer;
			double g_min_time;
			Time.setInterCheck(50);
			if (peak_val_flag) {
				timer = millis();										//Start timer				
				g_min_time = timer;
			}
			int RPM = current_output_1;												//Obtain Current RPM
			int prev_RPM = RPM;													//Record RPM
			if (Time.isMinChekTimeElapsed() == true)
			{
				current_RPM = current_output_1;


			}									//Do it again
			
			if (current_RPM <= prev_RPM)											//If measured value is less than previous value...
				min_val = current_RPM;												//Record Value
			min_time = g_min_time;													//Record Time
			min_val_flag = true;
			return min_val;														//Return to allow external access		
		}

		double settling_time_fun(int Target_Speed, int current_output)
		{
			long timer = millis();										//Start timer	
			int Current_RPM = current_output;										//Obtain Current RPM
			int Target_RPM = Target_Speed;
			//Obtain Target RPM
			//If value is within 10% tollerances, and minumum flag is tripped
			if ((((double)Current_RPM <= 1.1*(double)Target_RPM) && ((double)Current_RPM >= 0.9*(double)Target_RPM)) && min_val_flag == true){
				settling_time = rise_time + timer;											//Return to allow external access
			}
			settling_flag = true;
			return settling_time;

		}

		bool get_settling_flag() { return settling_flag; }

		void print_task_3_vals(int Target_Speed, int current_output)												//Function to print to serial monitor
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
	};*/

	class Task_4 {

	protected:
		IntervalCheckTimer Time;
		//controller_act2 controller;

		int count = 0;


		boolean sample_enabled;	// initialisation flag
		//boolean pid_enabled;

		static const int MAX_SIZE = 100;

		int RPM_vals[MAX_SIZE];	// sampled values stored as char to save memory (need to be mapped)

	public:
		// constructor
		Task_4()
		{
			sample_enabled = false;
			//pid_enabled = false;
		}

		bool settling_flag;

		void setupSampling(int sample_time)
		{
			Time.setInterCheck(sample_time);	// initialise sampling time from input
			sample_enabled = true;
		}

		boolean isEnabled()
		{
			if (sample_enabled)
			{
				return true;
			}
			else
				return false;
		}

		/*void stepResponse(int target_RPM, int current_RPM)
		{
			int PWM_val_pass;
			if (isEnabled())
			{
				if (controler.isTimeToTakeMeasurement())
				{
					int PWM_val = pid_test.PID_PWM(target_RPM, current_RPM);
					PWM_val_pass = PWM_val;
					//pid_test2.print_task_3_vals(target_RPM, current_RPM);
				}
				OLS.setPWM(PWM_val_pass);
				PID.set_echopidcontrol(1);
			}
		}*/

		double settling_time_fun_percentage(int Target_Speed, int current_output, int percentage)
		{
			long timer = millis();										//Start timer	
			int Current_RPM = current_output;										//Obtain Current RPM
			int Target_RPM = Target_Speed;								//Obtain Target RPM
			double settling_time;
			//If value is within 10% tollerances, and minumum flag is tripped
			if ((((double)Current_RPM == (1 + (percentage / 100))*(double)Target_RPM) && ((double)Current_RPM == (1 - (percentage / 100))*(double)Target_RPM))) {
				settling_time = timer;
				settling_flag = true;
			}
			return settling_time;												//Return to allow external access
		}

		void storeRPM(int current_RPM)
		{
			if (!settling_flag)
			{
				if (isEnabled())
				{
					RPM_vals[count] = current_RPM;	// store RPM in char array in range [0 255]
					count++;
				}
			}
			
		}

		void printData()
		{
			if (!settling_flag)
			{
				for (int i = 0; i < MAX_SIZE; i++)
				{
					Serial.println(RPM_vals[i]);
				}
			}
			
		}

	};

#endif
