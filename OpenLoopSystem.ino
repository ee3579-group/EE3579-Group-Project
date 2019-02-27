#include <OpenLoopSystem.h>

// this is the open loop system 
OpenLoopSystem testsystem;


void setup()
{
  // setup the potentiometer
  // this communicate the target RPM in range [1000 8000]
  testsystem.add_potentiometer(A2);

  // add three pushbuttons (pin and label)
  // change the pin number as appropriate but keep the labels (switch_on, switch_off, change_spin_dir)
  testsystem.add_buttonpin_and_label(6, switch_on);
  testsystem.add_buttonpin_and_label(4, switch_off);
  testsystem.add_buttonpin_and_label(7, change_spin_dir);

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
  
}


void loop()
{
  testsystem.CheckInputsAndControlMotor();
}
