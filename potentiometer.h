#ifndef potentiometer_h													//Mark Dawson (51769151) 21/02/2019
#define potentiometer_h
class potWmap{                                                      	//Create Class "potWmap"
	private:                                                          	//Private Variables
	int AnalogPin;                                                    	//Blank Integer to Store Pin Adress
	protected:                                                        	//Protected Variables
		double fromLow, fromHigh, toLow, toHigh;                        //Blank doubles to store Mapping Limits
		bool to_set, low_set, high_set;
		
	public:                                                          	//Public Variables/Member functions
		float G_val;                                                    //Blank float to store global variables 
		potWmap(){low_set=0; high_set=0;fromLow=0; fromHigh=1023;}      //Sets Limits
		void setup_pot(int in_Pin, int in_toLow, int in_toHigh)         //Function to setup potentiometer
		{
			toLow=in_toLow;                                              //Overwrite Variable
			toHigh=in_toHigh;                                            //Overwrite Variable
			to_set=true;                                                 //Set set flag to true
			AnalogPin=in_Pin;                                            //Overwrite Global variable with local input integer
		}
		
		float map_input()
		{
			float B_val=analogRead(AnalogPin);							//Floating Point Integerto store value read from input pin
			const int min_beta_val = 100;								//Constant Integer to store Minimum value = -1
			float val= min_beta_val + B_val/(1023/155);					//Equation to return result between 1 and -1 stored as floating point variable
			B_val=val;                                                  //Overwirte B_val with val
			return val;                                                 //Return value to allow acess by other functons/classes
		}
};																		//End Class Statement
#endif
