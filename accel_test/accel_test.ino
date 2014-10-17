// Compatible with test branch of LSM303

/*
    Basic script to start working with the LSM303DLHC 3-axis
    accel populated on the Pololu IMU v2.
    
    (c) Zac Staples zacsketches (at) github.
    
    You can use this software as you see fit provided you 
    acknowledge the source.
    
    This software builds on the LSM303 library provided by Pololu
    and forked by me at https://github.com/zacsketches/lsm303-arduino.
    My fork of this library adds several templated vector math
    functions in the header file, rearranges the file structure so
    it can be included easiliy in Arduino sketches if you clone the
    whole repo into your Arduino/library folder, and changes the 
    vector names to capital letters so the nomenclature in the program
    reflects the mathematics texts a little better.
    
    Accelerometer calculation to determine pitch based off the
    outstanding explanation in Freescale App Note AN3461 here:
    http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf
    
    *** From Pololu ***
    The sensor outputs provided by the library are the raw 16-bit values
    obtained by concatenating the 8-bit high and low accelerometer and
    magnetometer data registers. They can be converted to units of g and
    gauss using the conversion factors specified in the datasheet for your
    particular device and full scale setting (gain).
       
    In the LSM303DLHC, LSM303DLM, and LSM303DLH, the acceleration data
    registers actually contain a left-aligned 12-bit number, so the lowest
    4 bits are always 0, and the values should be shifted right by 4 bits
    (divided by 16) to be consistent with the conversion factors specified
    in the datasheets.
    
    Example: An LSM303DLH gives an accelerometer Z axis reading of -16144
    with its default full scale setting of +/- 2 g. Dropping the lowest 4
    bits gives a 12-bit raw value of -1009. The LA_So specification in the
    LSM303DLH datasheet (page 11) states a conversion factor of 1 mg/digit
    at this FS setting, so the value of -1009 corresponds to -1009 * 1 =
    1009 mg = 1.009 g.
    *** End Pololu quote ***
    
    .enableDefault() results in the following sensor config:
      50hz
      All axes enabled (Xen, Yen, Zen)
      +/- 2g full scale
      Normal mode (vice low-power mode)
      High resolution enabled
      
    This configuration results in a conversion factor of 
    
    1 mG      G
    ----- X -------  X Reading = Val in G's
    LSB     1000 mG
*/

#include <LSM303.h>
#include <Wire.h>

#define rad_to_deg 57.2958  		//180 / pi
#define deg_to_rad .0174533 		//pi / 180

typedef unsigned long Time;
typedef LSM303::vector<float> vector;

LSM303 accel;


const double sensitivity = 1.0;     //mG / LSB, datasheet page 9
const int    sample_num  = 1000;     //scalar
const int    sample_time = 20;      //ms...try to run at 50hz
vector             raw_g;
double         raw_angle = 0.0;
double compensated_angle = 0.0;
double  theta_angle_bias = 0.0;
Time                 now = 0; 

void setup() {
	Serial.begin(115200);
	Serial.println();
    
	Wire.begin();

	while(!accel.init());
	Serial.println("Accel initialized");
    
	accel.enableDefault();

	//Measure DC Offset and/or mounting position errors
	Serial.println("Computing bias");
	for(int i = 0; i < sample_num; ++i) {
		accel.read();
		raw_g.x += (int)accel.A.x;
		raw_g.y += (int)accel.A.y;
		raw_g.z += (int)accel.A.z;
	}
	raw_g.x /= sample_num;
	raw_g.y /= sample_num;
	raw_g.z /= sample_num;
    
	raw_g.x = (long)raw_g.x >> 4;
	raw_g.y = (long)raw_g.y >> 4;
	raw_g.z = (long)raw_g.z >> 4;

    //direct solution for pitch angle bias
    float Gx = accel.A.x;
    float Gy = accel.A.y;
    float Gz = accel.A.z; 
    double tan_theta_bias = -Gx / sqrt(pow(Gy, 2) + pow(Gz, 2));
    theta_angle_bias = atan(tan_theta_bias);
    
	//Display setup info
	print_config();
	delay(5000);
}

void loop() {
    //measure gravity and pitch angle.
   if (millis() - now > sample_time) {
       now = millis();
   	   accel.read();
	   accel.shift_accel();
       float Gx = accel.A.x;
       float Gy = accel.A.y;
       float Gz = accel.A.z; 
       double tan_theta = -Gx / sqrt(pow(Gy, 2) + pow(Gz, 2));
       
	   raw_angle = atan(tan_theta);
       compensated_angle = raw_angle - theta_angle_bias;
	   //Serial.println(angle);
	   raw_angle *= rad_to_deg;
	   compensated_angle *= rad_to_deg;
	   print_angle();
   }
}

void print_config() {
	Serial.print("Raw gravity vector:\n");
	vector_print(raw_g);
	
    Serial.print("Theta angle bias: ");
    Serial.println(theta_angle_bias*rad_to_deg,5);
    
    Serial.println("-------------------------");
}     

void print_angle() {
	Serial.print("Raw pitch is: ");
	Serial.print(raw_angle, 3);
    Serial.print("\tCompensated pitch is: ");
    Serial.println(compensated_angle, 3);
}
