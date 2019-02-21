#include <potentiometer.h>
potWmap an_inp;                     //Class.Object

void setup() {
  Serial.begin(9600);               //Baud Rate
  an_inp.setup_pot(A0,-1,1);        //(Input Pin, Lower Parameter, Upper Parameter)
}

void loop() {
  float val = an_inp.map_input();  //Call function from class and assign to float
  Serial.pintln(val);              //Print Val to new line
}
