// file Act1_Unit1.h
#ifndef Act1_Unit1_h
#define Act1_Unit1_h
#ifdef COMPILE_CPP_NOT_ARDUINO
#include "..\All_Arduino_or_Cpp_symboldefines\All_Arduino_or_Cpp_symboldefines.h"
// include the library where the classes I need as components are defined
#include "..\Basic_Input\Basic_Input.h"
#include "..\PushButton\PushButton.h"

#else
#include <Basic_Input.h>
#endif
// this library implements the "sensing unit" of my system
class Act2{
   // In the case of this project the two sensing units are a push button and the potentiometer
protected:
bool enabled;
int inp_button;
int temp_fan_speed;
inputs button1,button2,button3;
int fan_speed;
int input_byte_size;
boolean cmd_found;
command_list_enum in_command;
  in_push_button testbuttonUp_button1,testbuttonUp_button2,testbuttonUp_button3; 
potWmap potentiometer;
public:

   
int check_button()
{
    cmd_found=button1.check_n_get_command(in_command);
    if(cmd_found)
    {
    inp_button=1;
    }
    else 
    {
      cmd_found=button2.check_n_get_command(in_command);
      if(cmd_found)
       {
         inp_button=2;
       }
      else 
      {
          cmd_found=button3.check_n_get_command(in_command);
          if(cmd_found)
           {
            inp_button=3;
           }
           else
           {
           inp_button=0;
           }
      }
    }
    return inp_button;

}
  
void Sensor_setup(int Up_pin_button1,int Up_pin_button2, int Up_pin_button3, int Up_pin_potentiometer)
{
   potentiometer.setup_pot(int Up_pin_potentiometer, int 0, int 1024)
  button1.enable_setup_serial();
  button2.enable_setup_serial();
  button3.enable_setup_serial();
    Serial.print("Please enter the desired fan speed:");

  testbuttonUp_button1.assign_pin_command(Up_pin_button1, switch_on);
  button1.add_in_push_button(testbuttonUp_button1);
  
      testbuttonUp_button2.assign_pin_command(Up_pin_button2, switch_on);
      button2.add_in_push_button(testbuttonUp_button2);
      
          testbuttonUp_button3.assign_pin_command(Up_pin_button3, switch_on);
          button3.add_in_push_button(testbuttonUp_button3);
}  

int Potentiometer_input()
{
return(potentiometer.map_input());
}

};
#endif
