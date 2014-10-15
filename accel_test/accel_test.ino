/*
    Basic script to start working with the LSM303DLHC 3-axis
    accel populated on the Pololu IMU v2.
    
    (c) Zac Staples zacsketches (at) github.
    
    You can use this software as you see fit provided you 
    acknowledge the source.
    
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
vector              bias;
vector            comp_g;
double                angle = 0.0;
Time         now         = 0; 

void setup() {
	Serial.begin(115200);
	Serial.println();
    
	Wire.begin();
    
	//may need to add specifics to .init fuction...test with default first.
	//see L3G.h for alt definitions
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
	
	LSM303::vector_normalize(&raw_g);
	// vector_scale(raw_g, 1000);

	bias.x = -raw_g.x;
	bias.y = -raw_g.y;
	bias.z = 1.0-raw_g.z;
	
	vector_add(&raw_g, &bias, &comp_g);
    
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
	   vector temp;
	   temp.x = accel.A.x;
	   temp.y = accel.A.y;
	   temp.z = accel.A.z;
	   LSM303::vector_normalize(&temp);

	   // Serial.print("normalized temp: ");
	   // vector_print(temp);
	   
	   // vector_scale(temp, 1000);

	   // Serial.print("scaled temp: ");
	   // vector_print(temp);
	   
	   // accel.A.x = (int)round(temp.x);
	   // accel.A.y = (int)round(temp.y);
	   // accel.A.z = (int)round(temp.z);
	   
	   // Serial.print("rounded A: ");
	   // vector_print(accel.A);
	   
	   vector_add(&temp, &bias);
	   

	   
	   // Serial.print("compensated A: ");
	   // vector_print(accel.A);
	   
	   	    
	   // A dot Z axis is equal to mag(A) * 1 * cos(theta)
	   // if you normalize A this reduces to:
	   //	1 * 1 * cos(theta)
	   vector z_axis;
	   z_axis.x = 0.0;
	   z_axis.y = 0.0;
	   z_axis.z = 1.0;
   
	   double cos_theta = vector_dot(&temp, &z_axis);
	   if(cos_theta > 1) {
		   //this extra normalization prevents dot product values that
		   //exceed 1.0 and crash the acos function.
		   LSM303::vector_normalize(&temp);
		   cos_theta = vector_dot(&temp, &z_axis);
	   }
//	   Serial.println(cos_theta, 6);
	   angle = acos(cos_theta);
	   //Serial.println(angle);
	   angle *= rad_to_deg;

	   print_angle();
	   // print_accel_vec();
   }
}

void print_config() {
	Serial.print("Raw gravity vector:\n");
	vector_print(raw_g);
	
	Serial.print("Bias vector:\n");
	vector_print(bias);

	Serial.print("Compensated gravity vector:\n");
	vector_print(comp_g);
}     

void print_angle() {
	Serial.print("Angle is: ");
	Serial.println(angle);
}

void print_accel_vec() {
	Serial.print("Compensated Accel vector: ");
	vector_print(accel.A);		
}

double conv_unit_to_g(long g) {
  long res = g;
  //right shift four becuase lowest four bits are garbage
  res = (res >> 4);
  double d_res = res;
  d_res = d_res * sensitivity;
  d_res = d_res *.001;
  return d_res;
}
