#include "OpenLoopSystem_Task4.h"
#include "Check_timer.h"
// this is the open loop system 
OpenLoopSystem testsystem;
PID_System pid_test;
controller_act2 controller;
//Task_3 pid_test2;
Task_4 pid_test3;
void setup()
{
  // setup the potentiometer
  // this communicate the target RPM in range [1000 8000]
  //testsystem.add_potentiometer(A2);

  // add three pushbuttons (pin and label)
  // change the pin number as appropriate but keep the labels (switch_on, switch_off, change_spin_dir)
  testsystem.add_buttonpin_and_label(4, switch_off);
  testsystem.add_buttonpin_and_label(6, thirty_percent);
  testsystem.add_buttonpin_and_label(7, change_spin_dir);
  testsystem.add_buttonpin_and_label(8, fifty_percent);
  testsystem.add_buttonpin_and_label(9, eighty_percent);

  // setup the Hbridgemotor
  int pwmmotorpin=11;
  int directionpin=12;
  testsystem.setup_HBridgeDCmotor(pwmmotorpin, directionpin);
  
  // setup the speed measuring unit on pin 2 (interrupt 0)
  testsystem.setupSpeedMeasure(int_0);

  // adjust the itervals between 
  //two speed-adjustments; two taget-speed checks; two button checks; 
  int adjust_speed_interval_ms=250;
  int target_speed_check_interval_ms=1000;
  int buttons_check_interval_ms=200;
  testsystem.set_interval_vals(adjust_speed_interval_ms,target_speed_check_interval_ms,buttons_check_interval_ms);

  // to enable the verbose mode to print info on screen (off by default, makes program slower)
  testsystem.setVerbose(true);
  
  Serial.begin(9600);

  pid_test.set_gain_parameters(0.7*(1.0/32.0),0.45*(0.25/4.0),5.0);
  pid_test.set_ref_control_interval_ms(100);
  pid_test.set_bounds(0,255);
  controller.setup_controller(50);

  pid_test3.setupSampling(1);
  
}


void loop()
{
  int PWM_val_pass;
  testsystem.CheckInputsAndControlMotor_1();
  int curr_RPM = testsystem.getRPM();
  int target_RPM = testsystem.get_target();
  
  if (controller.isTimeToTakeMeasurement())
  {
    int PWM_val=  pid_test.PID_PWM(target_RPM,curr_RPM);
    PWM_val_pass = PWM_val;
    //pid_test2.print_task_3_vals(target_RPM, curr_RPM);
    pid_test3.storeRPM(curr_RPM);
  }
  //Serial.println(PWM_val_pass);
  testsystem.setPWM(PWM_val_pass);
  Serial.println(curr_RPM);
  //pid_test3.printData();
  
  //int PWM_val=  pid_test.PID_PWM(4800,RPM);
  //testsystem.setPWM(PWM_val);
  pid_test.set_echopidcontrol(1);
}
