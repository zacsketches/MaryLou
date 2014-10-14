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

L3G gyro;

typedef unsigned long Time;

const int sample_num  = 500;     //scalar
const int sample_time = 10       //ms...try to run at 100hz
int       dc_offset   = 0;       //usits
int       noise       = 0;       //units
int       rate        = 0;       //units per sec
int       prev_rate   = 0;
double    angle       = 0.0;       //units
Time      now         = 0; 

void setup() {
    Serial.begin(115200);
    Serial.println();
    
    Wire.begin();
    
    //may need to add specifics to .init fuction...test with default first.
    //see L3G.h for alt definitions
    while(!gyro.init());
    
    gyro.enableDefault();

    //Measure DC Offset (datasheet refers to Zero-rate-level)
    for(int i = 0; i < sample_num; ++i) {
        gyro.read();
        dc_offset += int(gyro.G.y);
    }
    dc_offset /= sample_num;
    
    //Measure max noise threshold
    for(int i = 0; i < sample_num; ++i) {
        gyro.read();
        if( (int)gyro.G.y-dc_offset > noise)
            noise = (int)gyro.G.y - dc_offset;
        else if( (int)gyro.G.y-dc_offset < -noise)
            noise = -(int)gyro.G.y - dc_offset;
    }
        
    //Display setup info
    print_config();
    delay(5000);
}

void loop() {
    //measure rate and angle.
    //use trapezoidal integration for angle
    if (millis() - now > sample_time) {
        now = millis();
        gyro.read();
        rate = ( (int)gyro.G.y-dc_offset);
        
    /*
        TODO I could measure the actual time_step for a better integration
        ...or I could take measurement on a timer interrupt
    */
        int mean_rate = (prev_rate + rate) /2;
        double conv_sample_time = (double)sample_time / 1000; //convert to sec
        angle += (double)mean_rate * conv_sample_time;
  
        prev_rate = rate;
    }

    print_data();
}

void print_config() {
    char buf[75];
    sprintf(buf, "DC Offset:%d\tNoise:%d\tvals in sensor units",
        dc_offset, noise);
    Serial.println(buf);
}     

void print_data() {
    char buf[75];
    sprintf(buf, "rate:%d\tangle%d", rate, angle);
    Serial.println(buf);
}