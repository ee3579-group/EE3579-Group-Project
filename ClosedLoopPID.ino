#include "Closed_Loop_PID.h"
//#include "RPMSensor.h"
PID myPID;
//RPMSensor myRPM;

void setup() {
  myPID.set_gain_parameters(0.7,0.45,0.5);
  myPID.set_ref_control_interval_ms(1000);
  myPID.set_bounds(0,255);
  Serial.begin(4800);
}

void loop() {

  //int Current_Speed = myRPM.getRPM();
  int PWM_val=  myPID.PID_PWM(100,Current_Speed);  //(target,current)
  //PWM_val can now be passed to the motor to hopefully make the motor go as desired.
}
