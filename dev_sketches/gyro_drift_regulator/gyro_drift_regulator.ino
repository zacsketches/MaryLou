//Compatible with test branch of LSM303 and L3G

/*
    An intermediate level script to use the accelerometer pitch
    reading as negative feedback to the gyro pitch estimation with
    the intent of reducing the gyro drift error.
    
    See the PowerPoint brief in the datasheets section for the block
    diagram implemented here.
   
    Defined values for gyro DC_offset and accel theta_bias were taken from
    values collected in gyro_test.ino and accel_test.ino run multiple
    times.  These values are a function of installation alignment and 
    inherent sensor error.  
    
    (c) Zac Staples zacsketches (at) github.
    
    You can use this software as you see fit provided you 
    acknowledge the source.
    
    accel.enable default() results in the following sensor config:
      50hz
      All axes enabled (Xen, Yen, Zen)
      +/- 2g full scale
      Normal mode (vice low-power mode)
      High resolution enabled
      
    This configuration results in a conversion factor of 
    
    1 mG      G
    ----- X -------  X Reading = Val in G's
    LSB     1000 mG

    Gyro calibration:

                      250 deg 
   At sensitivity of  ------- the scaler to convert to deg/sec (datasheet pg9) 
                       1 sec                                               
      8.750 mdeg/sec      1 deg/sec         Reading unit
   is -------------  X ---------------  X ---------------             
          1 unit        1000 mdeg/sec

*/

#include <LSM303.h>
#include <L3G.h>
#include <Wire.h>

#define RAD_TO_DEG 57.2958  		// 180 / pi
#define DEG_TO_RAD .0174533 		// pi / 180
#define GYRO_COUNTS_TO_DEG .00875   // see above
#define DEG_TO_GYRO_COUNTS 114.2857	// 1 / .00875

#define THETA_BIAS -0.43329    //FROM EXPERIMENTAL DATA WITH ACCEL_TEST.INO
#define DC_OFFSET -223         //From experimental data with gyro_test.ino


typedef unsigned long Time;
typedef LSM303::vector<float> vector;

LSM303 accel(LSM303::I2C_port::secondary);
L3G gyro(L3G::I2C_port::secondary);

// Timing constants
Time         now         = 0;   
double       timescale   = 1000.0;    //use millis() for timing
//double       timescale   = 1000000.0; //use micros() for timing
const int    sample_time = 20;      //ms


//*********************************************************************************
//                               DATA STRUCTURES
//*********************************************************************************

struct Gyro_reading {
    double    omega;
    Time timestamp;
};

struct Error {
    double epsilon;
    Time timestamp;
};


//*********************************************************************************
//                               MODEL COMPONENTS
//*********************************************************************************

class Calibrated_gyro {
public:
    Calibrated_gyro(L3G& gyroscope) : gyro(gyroscope), pitch_axis(gyro.G.x) {}
    
    bool begin() {
        bool result = false;
        while(!gyro.init());
        result = true;
        gyro.enableDefault();
        return result;
    }
    
    Gyro_reading read(){
        Gyro_reading theta_raw;
        gyro.read();
        theta_raw.omega = pitch_axis - dc_offset;
        theta_raw.timestamp = millis();
        
        return theta_raw;
    }
    
    double conv_factor() const { return gyro_sensitivity; }
    
private:
    L3G& gyro;
    static const double gyro_sensitivity = .00875;    //datasheet page 9.  mdeg/sec
    static const long   dc_offset = DC_OFFSET;              //units
    float& pitch_axis;
          
};

class Calibrated_accel {
public:
    Calibrated_accel(LSM303& accelerometer) 
        : accel(accelerometer), roll_axis(accel.A.y), yaw_axis(accel.A.z) {}
    
    bool begin() {
        bool result = false;
        while(!accel.init());
        result = true;
        accel.enableDefault();
        return result;
    }

    double pitch_angle() {
        double theta_accel = 0.0;
        accel.read();
        accel.shift_accel();
        float Gy = roll_axis;
        float Gz = -1 * yaw_axis;
        theta_accel = atan2(Gy, Gz);

        theta_accel *= RAD_TO_DEG;
        return theta_accel - theta_bias;
    }
    
private:
    LSM303& accel;
    static const float theta_bias = THETA_BIAS;
    int16_t& roll_axis;
    int16_t& yaw_axis;
};

class Drift_adjuster {
public:
    Drift_adjuster() {}
    
    Gyro_reading run(Gyro_reading omega_raw, double omega_adjustment) {
        Gyro_reading omega_compensated;
        omega_compensated.omega = omega_raw.omega + omega_adjustment;
        omega_compensated.timestamp = omega_raw.timestamp;
        return omega_compensated;
    }
};

class Gyro_plant {
public:
    Gyro_plant(Calibrated_gyro& gyroscope) : gyro(gyroscope)  {}
    
    struct Fifo_list{
        Gyro_reading list[4];
        int len;
        int pointer;  //keep the pointer on the most recent data
        
        Fifo_list(int list_length) {
            pointer = 0;
            len = list_length;
        }
        
        void add(Gyro_reading val) {
            //the pointer locates the position in the array where new data
            //gets inserted
            advance_pointer(pointer);
            list[pointer] = val;
        }
        
        void advance_pointer(int& ptr) {
            ++ptr;
            if(ptr >= len) {
                ptr = 0;
            }   
        }
        
        void regress_pointer(int& ptr) {
            --ptr;
            if(ptr < 0) {
                ptr = len-1;
            }   
        }
        
        void data(Gyro_reading* copy) {
            //fill the copy list with the current data.  I want a list that
            //has the oldest data in position[0] with progressively newer
            //data in subsequent locations
            int dup_pointer = pointer;
            for(int i=len-1; i<0; --i) {
                copy[i]=list[dup_pointer];
                regress_pointer(dup_pointer);   
            }
        }
        
        double w0() {
            int tmp_ptr = pointer;
            regress_pointer(tmp_ptr);
            regress_pointer(tmp_ptr);
            regress_pointer(tmp_ptr);
            return list[tmp_ptr].omega;
        }
        
        double w1() {
            int tmp_ptr = pointer;
            regress_pointer(tmp_ptr);
            regress_pointer(tmp_ptr);
            return list[tmp_ptr].omega;
        }
        
        double w2() {
            int tmp_ptr = pointer;
            regress_pointer(tmp_ptr);
            return list[tmp_ptr].omega;
        }
        double w3() {
            return list[pointer].omega;
        }
        Time t1() {
            int tmp_ptr = pointer;
            regress_pointer(tmp_ptr);
            regress_pointer(tmp_ptr);
            return list[tmp_ptr].timestamp;
        }
    };
    
    double run(Gyro_reading omega_compensated) {
        //time management
        Time interval = omega_compensated.timestamp - timestamp_prev;
        double dt = (double)interval;
        dt = dt / timescale;
        timestamp_prev = omega_compensated.timestamp;
        
        //find trapezoidal omega for integration
        double omega_mean = (omega_prev + omega_compensated.omega) / 2.0;
        
        //perform numerical integration
        theta_counts += omega_mean * dt;
        
        //get ready for the next run
        omega_prev = omega_mean;
    
        theta_angle = theta_counts * gyro.conv_factor();      
  
        return theta_angle;
    }
    
    /* 
       Run a simpson integration instead of trapezoidal.  See if a 
       slightly more complex numerical integration yields a more stable
       gyro.  The max mechanical response speed of the system is probably
       around 3hz so taking a little bit of time to do a better integration
       probably isn't going to delay the control command enough to 
       lose controlability.
       
       We basically run the simpson 3/8's rule for four data points and
       subtract the simpson 1/3 rule for three data points.  This gets 
       the little slice and adds it to the total.
       
       http://mathworld.wolfram.com/Simpsons38Rule.html
       http://mathworld.wolfram.com/SimpsonsRule.html
    */ 
    double run_simpson(Gyro_reading omega_compensated) {
        static Fifo_list hist(4);
        double simpson_counts;
        static double simpson_counts_prev;
        static Time timestamp_prev;
        static double simpson_angle;
        
        hist.add(omega_compensated);
        
        //time management
        Time interval = omega_compensated.timestamp - timestamp_prev;
        double dt = (double)interval;
        dt = dt / timescale;
        timestamp_prev = omega_compensated.timestamp;
        
        //debug
//        Serial.print("\two is: ");
//        Serial.println(hist.w0());
//        Serial.print("\tw1 is: ");
//        Serial.println(hist.w1());
//        Serial.print("\tw2 is: ");
//        Serial.println(hist.w2());
//        Serial.print("\tw3 is: ");
//        Serial.println(hist.w3());        
//        Serial.print("\tt0 is: ");
//        Serial.println(hist.t1());        
//        Serial.print("\tdt is: ");
//        Serial.println(dt,5);

        //would be more accurate if I ignored the first four samples
        simpson_counts = simpson_counts_prev + (9.0/24.0*hist.w3() + 19.0/24.0*hist.w2() - 5.0/24.0*hist.w1() + 1.0/24.0*hist.w0()) * dt;
                
        simpson_angle = simpson_counts * gyro.conv_factor();
        
        //get ready for the next run
        simpson_counts_prev = simpson_counts;
        
        return simpson_angle;
    }
    
private:
    Calibrated_gyro& gyro;
    double theta_counts;
    double theta_angle;
    double omega_prev;
    Time timestamp_prev;
    
};

class Error_computer{
public:
    Error_computer() {}
    
    Error run(double theta_gyro, double theta_accel) {
        Error result;
        result.epsilon = theta_gyro - theta_accel;
		result.epsilon *= DEG_TO_GYRO_COUNTS;
		result.epsilon *= -1;
        result.timestamp = millis();
        return result;
    }
};

class PI_controller {
public:
    PI_controller(double kp, double ki) :Kp(kp), Ki(ki) {}

    double run_parallel(Error input){
        double omega_adjustment = 0.0;
        
        // Time management
        Time interval = input.timestamp - timestamp_prev;
        double dt = (double)interval;
        dt = dt / timescale;
        timestamp_prev = input.timestamp;
        
        //trap z integration for epsilon
        epsilon_integrated += input.epsilon * dt;
        
        //solve for adjustment
        omega_adjustment = Kp * (input.epsilon) + Ki * (epsilon_integrated);
        
        return omega_adjustment;
    }
    
    double run_standard(Error input) {
        //see http://en.wikipedia.org/wiki/PID_controller for a discussion of standard form
        //Ki is related to Ti by the expression
        //         Kp
        //  Ki = -------
        //         Ti
        //
        //I want to bring the system into balance in ten timesteps so I am using 10 for Ti
        //therefore, Ki/Kp = 1/Ki
        static double one_over_Ti = Ki/Kp;
        
        double omega_adjustment = 0.0;
        
        // Time management
        Time interval = input.timestamp - timestamp_prev;
        double dt = (double)interval;
        dt = dt / timescale;
        timestamp_prev = input.timestamp;
        
        //trap z integration for epsilon
        epsilon_integrated += input.epsilon * dt;
        
        //solve for adjustment
        omega_adjustment = Kp * (input.epsilon +  one_over_Ti*epsilon_integrated);
        
        return omega_adjustment;  
    }
    
private:
    float Kp;
    float Ki;
    Time timestamp_prev;    
    double epsilon_integrated;
};

//*********************************************************************************
//                               CONSTRUCTION
//*********************************************************************************
Calibrated_gyro calibrated_gyro(gyro);
Calibrated_accel calibrated_accel(accel);
Drift_adjuster drift_adjuster;
Gyro_plant gyro_plant(calibrated_gyro);
Error_computer error_computer;
PI_controller pi_controller(4.0, 2.0);  //(float Kp, float Ki)

//*********************************************************************************
//                               GLOBAL MODEL VARIABLES
//*********************************************************************************
//inputs to drift adjuster
Gyro_reading omega_raw;
double omega_adjustment;

//inputs to gyro plant
Gyro_reading omega_compensated;

//inputs to error computer
double theta_gyro;
double theta_accel;

//inputs to PI controller
Error error_signal; 

//*********************************************************************************
//                               MAIN SKETCH
//*********************************************************************************
void setup() {
    Serial.begin(115200);
    Serial.println();
    
    Wire1.begin();
    
    Serial.println("starting gyro");
    while(!calibrated_gyro.begin());
    
    Serial.println("starting accel");
    while(!calibrated_accel.begin());
    
    Serial.println("setting initial conditions from accel data");
    double initial_condition = calibrated_accel.pitch_angle();
    theta_gyro = initial_condition;
    theta_accel = initial_condition;
    
    Serial.println("all components started, initiating 3 sec delay");
    delay(3000);
    
    Serial.println("going into loop");
}

void loop() {
    //create a few static variables to analyze the min and max data
    static double max_theta = 0.0;
    static double min_theta = 500.0;
    
    //start from error signal and work clockwise around model
    //run the error computer
    theta_accel = calibrated_accel.pitch_angle();
    error_signal = error_computer.run(theta_gyro, theta_accel);
    
    //run the pi_controller
    omega_adjustment = pi_controller.run_standard(error_signal);
    
    //run the drift adjuster
    omega_raw = calibrated_gyro.read();
    omega_compensated = drift_adjuster.run(omega_raw, omega_adjustment);
    
    //run the gyro plant
    theta_gyro = gyro_plant.run_simpson(omega_compensated);
    
//    char buf[130];
//    char float_buf1[10];
//    char float_buf2[10];
//    char float_buf3[10];
//    char float_buf4[10];
//    char float_buf5[10];
//    char float_buf6[10];
//    char float_buf7[10];
//    char float_buf8[10];
//    
//    sprintf(buf, "theta_acc: %s, err_sig: %s, adjust: %s, o_raw: %s, o_comp: %s, theta_gyro: %s", 
//      ftoa(float_buf1, theta_accel, 3),
//      ftoa(float_buf2, error_signal.epsilon, 3),
//      ftoa(float_buf3, omega_adjustment, 3),
//      ftoa(float_buf4, omega_raw.omega, 3),
//      ftoa(float_buf5, omega_compensated.omega, 3),
//      ftoa(float_buf6, theta_gyro, 3));
//    Serial.println(buf);
    max_theta = max(theta_gyro, max_theta);
    min_theta = min(theta_gyro, min_theta);
    
    Serial.print("Theta is: ");
    Serial.print(theta_gyro,3);
    Serial.print("\tmax: ");
    Serial.print(max_theta,3);    
    Serial.print("\tmin: ");
    Serial.println(min_theta,3);

}


//from Arduino forum
char* ftoa(char* a, double f, int precision)
{
  long p[] = {0,10,100,1000,10000,100000,1000000,10000000,100000000};
  
  char *ret = a;
  long heiltal = (long)f;
  itoa(heiltal, a, 10);
  while (*a != '\0') a++;
  *a++ = '.';
  long desimal = abs((long)((f - heiltal) * p[precision]));
  itoa(desimal, a, 10);
  return ret;
}


