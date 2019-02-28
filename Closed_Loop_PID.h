#ifndef Closed_Loop_PID_H
#define Closed_Loop_PID_H

#include <Arduino.h>

class PID
{

protected:

	double ref_Kp;                      //P from Input
	double ref_Ki;                      //I from Input
	double ref_Kd;                      //D from Input
	double ref_contr_inter_time_ms;     //Refference Timer to scale PID parameters

  double Kp;                          //P used in calculations
  double Ki;                          //I used in calculations
  double Kd;                          //D used in calculations
  double Ku=0;                        //Ultimate gain (lowest possible value for constant oscillation)
  double Pu=0;                        //Period of above oscillations
	double PID_PWM_Min, PID_PWM_Max;    //PWM range
	unsigned long last_control_ms;      //Stores last time
	double previous_error;              //stores last error value
	double error_sum;                   //Stores sum of previous errors
	double static const REF_CONTR_INTER_MS=1000;
	bool enable_flag;                   //Enabled Flag
	bool echopidcontrol;                //Echo PID Controll Flag
	
	
	void set_default_parameters()
	{
		ref_Kp=0.7;
		ref_Ki=0.45;
		ref_Kd=0.5;
		ref_contr_inter_time_ms=REF_CONTR_INTER_MS;
		PID_PWM_Min=0.0;
		PID_PWM_Max=255.0;
		last_control_ms=millis();
		previous_error=0.0;
		error_sum=0.0;
		echopidcontrol=false;
		enable_flag=true;             //Set Enable Flag to True
	}



public:
	basic_speed_PID(){set_default_parameters();}
	basic_speed_PID(double inp_ref_Kp, double inp_ref_Ki, double inp_ref_Kd, double inp_PIDoutMin, double inp_PIDoutMax, int inp_ref_contr_inter_time_ms=REF_CONTR_INTER_MS)
	{
		set_default_parameters();
		set_gain_parameters(inp_ref_Kp, inp_ref_Ki, inp_ref_Kd);
		set_ref_control_interval_ms(inp_ref_contr_inter_time_ms);
		set_bounds(inp_PIDoutMin, inp_PIDoutMax);
	}
	
	void set_gain_parameters(double inp_ref_kp, double inp_ref_ki, double inp_ref_kd)
	{	
		ref_Kp=inp_ref_kp;            //Overwrite Global Variable with local Input
		ref_Ki=inp_ref_ki;            //Overwrite Global Variable with local Input
		ref_Kd=inp_ref_kd;            //Overwrite Global Variable with local Input
    enable_flag=true;             //Set Enable Flag to True
	}
	void set_ref_control_interval_ms(int inp_ref_contr_inter_time_ms)
	{
		ref_contr_inter_time_ms=inp_ref_contr_inter_time_ms;
	}
	void set_bounds(double inp_PID_PWM_Min, double inp_PID_PWM_Max)
	{
		PID_PWM_Min=inp_PID_PWM_Min;          //Overwrite Global Variable with local Input
		PID_PWM_Max=inp_PID_PWM_Max;          //Overwrite Global Variable with local Input
	}
  
  void Ziegler_Nichols_tuning()   //Primary Method of Acheiving Critical Response
  {
    ref_Kp= Ku/1.7;                //Set Global Proportional Gain as Ultimate Gain * 1.7
    ref_Ki= Pu/2;                  //Set Global Intergral Gain as Ultimate Period/2
    ref_Kd= Pu/8;                  //Set Global Differential Gain as Ultimate Period/8
    enable_flag=true;              //Set Enable Flag to True
  }

  void Tyreus_Luyben_Tuning()      //Alternative Method of Acheiving Critical Response
  {
    ref_Kp= Ku/2.2;                //Set Global Proportional Gain as Ultimate Gain/2.2
    ref_Ki= 2.2*Pu;                //Set Global Intergral Gain as Ultimate Period * 2.2
    ref_Kd= 6.3/Pu;                //Set Global Differential Gain as 6.3/Ultimate Period
    enable_flag=true;              //Set Enable Flag to True
  }

	double PID_PWM(double target_speed, double curr_speed)
	{
		unsigned long current_time = millis();		
		double output = 0.0;
		unsigned long tempcontrol_interval = (current_time - last_control_ms);
		int contr_inter_time_ms=(int)tempcontrol_interval;

		double error = target_speed-curr_speed;
		double contr_inter_time_ratio=ref_contr_inter_time_ms/contr_inter_time_ms;
		double error_diff = error - previous_error;
		double error_derivative = error_diff/((double)contr_inter_time_ms); 

		error_sum+= (error*contr_inter_time_ratio);
		Kp = ref_Kp;
		Ki = ref_Ki;
		Kd = ref_Kd;
		
		output = Kp*error + Ki*error_sum + Kd*error_derivative;

		if(output > PID_PWM_Max)
			output = PID_PWM_Max;
		else
			if(output < PID_PWM_Min)
				output = PID_PWM_Min;
		
		if(echopidcontrol)
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

		last_control_ms=current_time;
		previous_error=error;
		return output;
	}

	double GetKp(){return Kp;}           //By Calling this function, Kp may be obtained within a .ino file
	double GetKi(){return Ki;}           //By Calling this function, Ki may be obtained within a .ino file
	double GetKd(){return Kd;}           //By Calling this function, Kd may be obtained within a .ino file

	void set_echopidcontrol(bool inp_echo){echopidcontrol=inp_echo;}
	bool get_echopidcontrol(){return echopidcontrol;}
 
	void reset_pidcontrol()              
	{
		last_control_ms=millis();
		previous_error=0.0;
		error_sum=0.0;
	}
};

#endif
