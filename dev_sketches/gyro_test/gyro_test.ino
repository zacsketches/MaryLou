//Compatible with test branch of L3G

/*
    Basic script to start working with the L3GD20 3-axis
    gyro populated on the Pololu IMU v2.
    
    (c) Zac Staples zacsketches (at) github.
    
    You can use this software as you see fit provided you 
    acknowledge the source.
    
    Based in large measure on the lab sheet provided at this link
    http://www.csulb.edu/~hill/ee444/Labs/5%20Gyro/5%20Arduino%20IDE%20Gyro.pdf
   
   I made a minor change in the L3G.h file...the vector 'g' was the
   latest read from the gyro.  I changed this to 'G' because it's more familiar
   to me to use vectors with a capital letter  
   
   Additionally, I configured the L3G library to work on both ports of the
   Arduino Due
   
   I also need to look at why the Pololu library uses floats for the values
   of G when, based off the datasheet page 26, the sensor returns twos 
   complement integer values for its readings.
   
                      250 deg 
   At sensitivity of  ------- the scaler to convert to deg/sec (datasheet pg9) 
                       1 sec                                               
      8.750 mdeg/sec      1 deg/sec         Reading unit
   is -------------  X ---------------  X ---------------             
          1 unit        1000 mdeg/sec
*/

#include <L3G.h>
#include <Wire.h>

L3G gyro(L3G::I2C_port::secondary);

//determined by the physical installtion of the device in the robot
float& pitch_rate = gyro.G.x;

typedef unsigned long Time;

const double sensitivity = 8.75;    //datasheet page 9.  mdeg/sec
const int    sample_num  = 1000;     //scalar
const int    sample_time = 15;      //ms
long         dc_offset   = 0;       //units
int          noise       = 0;       //units
int          rate        = 0;       //units per sec
int          prev_rate   = 0;
double       angle       = 0.0;     //units
Time         now         = 0; 

void setup() {
    Serial.begin(115200);
    Serial.println();
    
    Wire1.begin();
    
    //may need to add specifics to .init fuction...test with default first.
    //see L3G.h for alt definitions
    while(!gyro.init());
    Serial.println("Gyro initialized");
    
    gyro.enableDefault();

    Serial.println("Computing bias and noise bandwidth");
    //Measure DC Offset (datasheet refers to Zero-rate-level)
    for(int i = 0; i < sample_num; ++i) {
        gyro.read();
        dc_offset += int(pitch_rate);
    }
    dc_offset /= sample_num;
    
    //Measure max noise threshold
    for(int i = 0; i < sample_num; ++i) {
        gyro.read();
        if( (int)pitch_rate-dc_offset > noise)
            noise = ((int)pitch_rate - dc_offset);
        else if( (int)pitch_rate-dc_offset < -noise)
            noise = -((int)pitch_rate - dc_offset);
    }
        
    //Display setup info
    print_config();
    Serial.println("Beginnning 5 sec delay."
    delay(5000);
}

void loop() {
    //measure rate and angle.
    //use trapezoidal integration for angle
    if (millis() - now > sample_time) {
        now = millis();
        gyro.read();
        rate = (int)(pitch_rate-dc_offset);
        
		/*
		TODO I could measure the actual time_step for a better integration
		...or I could take measurement on a timer interrupt
		*/
		/*
		TODO the line below is where I make my mark.  
		From page 15 of the DCM paper, instead of using
		raw mean_rate in the trapezoidal integration I would 
		be better off making Omega = measured + correction.  The
		correction value can come from my PI controller with an
		accel input like this:
			int adjusted_rate = mean_rate + drift_correction;
		*/
        int mean_rate = (prev_rate + rate) /2;
        double sample_time_in_sec = (double)sample_time / 1000; //convert to sec
        angle += (double)mean_rate * sample_time_in_sec;
  
        prev_rate = rate;
    }

    print_data();
}

void print_config() {
    char buf[75];
    sprintf(buf, "DC Offset:%d\tNoise: ", dc_offset);
    Serial.print(buf);
	Serial.println(noise);
}     

void print_data() {
    char buf[75];
    sprintf(buf, "rate: %+05d\tangle: ", rate);
    Serial.print(buf);
    Serial.print(angle);
    Serial.print("\tangle in deg: ");
    Serial.println(conv_angle_to_deg(angle),3);
}

double conv_angle_to_deg(double a) {
  double res;
  res = a * sensitivity;
  res /= 1000;
  return res;
}
