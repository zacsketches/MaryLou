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

LSM303 accel;

typedef unsigned long Time;

const double sensitivity = 1.0;     //mG / LSB, datasheet page 9
const int    sample_num  = 1000;     //scalar
const int    sample_time = 20;      //ms...try to run at 50hz
long         gravity_vec_x = 0;       //units
long         gravity_vec_y = 0;       //units
long         gravity_vec_z = 0;       //units
int               x_bias = 0;
int               y_bias = 0;
int               z_bias = 0;
long         bias        = 0;
int          noise       = 0;       //units
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

    Serial.println("Computing bias and noise bandwidth");
    //Measure DC Offset (datasheet refers to Zero-rate-level)
    for(int i = 0; i < sample_num; ++i) {
        accel.read();
        gravity_vec_x += (int)accel.A.x;
        gravity_vec_y += (int)accel.A.y;
        gravity_vec_z += (int)accel.A.z;
    }
    gravity_vec_x /= sample_num;
    gravity_vec_y /= sample_num;
    gravity_vec_z /= sample_num;
    
	gravity_vec_x = gravity_vec_x >> 4;
	gravity_vec_y = gravity_vec_y >> 4;
	gravity_vec_z = gravity_vec_z >> 4;

	x_bias = -gravity_vec_x;
	y_bias = -gravity_vec_y;
	z_bias = 1000-gravity_vec_z;
        
    //Display setup info
    print_config();
    delay(5000);
}

void loop() {
    //measure gravity and pitch angle.
   if (millis() - now > sample_time) {
       now = millis();
	   accel.read();
	   
	   //A dot Z is equal to mag(A) * 1 * cos(theta)
	   //if you normalize A this reduces to:
	   //	1 * 1 * cos(theta)
	   
//        gyro.read();
//        rate = ( (int)gyro.G.y-dc_offset);
//        
//    /*
//        TODO I could measure the actual time_step for a better integration
//        ...or I could take measurement on a timer interrupt
//    */
//        int mean_rate = (prev_rate + rate) /2;
//        double conv_sample_time = (double)sample_time / 1000; //convert to sec
//        angle += (double)mean_rate * conv_sample_time;
//  
//        prev_rate = rate;
//    }
//
//    print_data();
}

void print_config() {
	Serial.print("Gravity vector:\n");
	Serial.println(gravity_vec_x);
	Serial.println(gravity_vec_y);
	Serial.println(gravity_vec_z);
	
	Serial.print("Bias vector:\n");
	Serial.println(x_bias);
	Serial.println(y_bias);
	Serial.println(z_bias);	
}     

void print_data() {
//    char buf[75];
//    sprintf(buf, "rate: %+05d\tangle: ", rate);
//    Serial.print(buf);
//    Serial.print(angle);
//    Serial.print("\tangle in deg: ");
//    Serial.println(conv_angle_to_deg(angle));
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
