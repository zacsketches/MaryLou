/*
    An intermediate level script to use the accelerometer pitch
    reading as negative feedback to the gyro pitch estimation with
    the intent of reducing the gyro drift error.  See diagram one
    in the DCM document in datasheets for a general idea of the 
    control system.  Imagine that we are only controlling for one
    axis instead of three and that is what I'm implementing here.
    
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

#define rad_to_deg 57.2958  		//180 / pi
#define deg_to_rad .0174533 		//pi / 180

typedef unsigned long Time;
typedef LSM303::vector<float> vector;

LSM303 accel;
L3G gyro;

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
    Calibrated_gyro(L3G& gyroscope) : gyro(gyroscope) {}
    
    bool begin() {
        bool result = false;
        while(!gyro.init());
        result = true;
        gyro.enableDefault();
        return resutl;
    }
    
    Gyro_reading read(){
        Gyro_reading theta_raw;
        gyro.read();
        theta_raw.omega = gyro.G.y-dc_offset;
        thata_raw.timestamp = millis();
        
        return theta_raw;
    }
    
private:
    L3G& gyro;
    static const double gyro_sensitivity = 8.75;    //datasheet page 9.  mdeg/sec
    static const long   dc_offset = 110;       //units
          
};

class Calibrated_accel {
public:
    Calibrated_accel(LSM303& accelerometer) : accel(accelerometer) {
        //load bias data
        bias.x = x_bias;
        bias.y = y_bias;
        bias.z = z_bias;
        
        //configure z_axis vector
        z_axis.x = 0.0;
        z_axis.y = 0.0;
        z_axis.z = 1.0;
    }
    
    bool.begin() {
        bool result = false;
        while(!accel.init());
        result = true;
        accel.enableDefault();
        return resutl;
    }

    double pitch_angle() {
        double theta_accel = 0.0;
        accel.read;
        accel.shift_accel();
        vector temp;
        temp.x = accel.A.x;
        temp.y = accel.A.y;
        temp.z = accel.A.z;
        LSM303::vector_normalize(&temp);
        vector_add(&temp, &bias); 
        double cos_theta = vector_dot(&temp, &z_axis);
        if(cos_theta > 1.0) {
            //this extra normalization prevents dot product values that
            //exceed 1.0 and crash the acos function.
            LSM303::vector_normalize(&temp);
            cos_theta = vector_dot(&temp, &z_axis);
        }
        theta_accel = acos(cos_theta);
        theta_accel *= rad_to_deg;

        return theta_accel;
    }
    
private:
    LSM303& accel;
    vector bias;
    vector z_axis;
    static const float x_bias = -50.0;
    static const float y_bias = -20.0;
    static const float z_bias =  22.0;
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
    Gyro_plant() {}
    
    double run(Gyro_reading omega_compensated) {
        //time management
        Time dt = omega_compensated.timestamp - timestamp_prev;
        dt = dt / timescale;
        timestamp_prev = omega_compensated.timestamp;
        
        //find trapezoidal omega for integration
        omega_prev = (omega_prev + omega_compensated.omega) / 2.0;
        
        //perform numerical integration
        theta_gyro += omega_prev * dt;
        
        return theta_gyro;
    }
    
private:
    double theat_gyro;
    double omega_prev;
    Time timestamp_prev;
};

class Error_computer{
public:
    Error_computer() {}
    
    Error run(double theta_gyro, double theta_accel) {
        Errro result;
        result.epsilon = theta_gyro - theta_accel;
        result.timestamp = millis();
        return result;
    }
};

class PI_controller {
public:
    PI_controller(double kp, double ki) :Kp(kp), ki(Ki) {}

    double run(Error input){
        double omega_adjustment = 0.0;
        
        // Time management
        Time dt = input.timestamp - timestamp_prev;
        dt /= timescale;
        
        //integrate
        epsilon_integrated += input.epsilon * dt;
        
        //solve for adjustment
        omega_adjustment = Kp * (epsilon) + Ki * (epsilon_integrated);
        
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
Gyro_plant gyro_plant;
Error_computer error_computer;
PI_controller pi_controller(1.0, 1.0);  //(float Kp, float Ki)

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
    
    Wire.begin();
    
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
    //start from error signal and work clockwise around model
    
    //run the error computer
    theta_accel = callibrated_accel.pitch_angle();
    error_signal = error_computer.run(theta_gyro, theta_accel);
    
    //run the pi_controller
    omega_adjustment = pi_controller.run(error_signal);
    
    //run the drift adjuster
    omega_raw = calibrated_gyro.read();
    omega_compensated = drift_adjuster.run(omega_raw, omega_adjustment)
    
    //run the gyro plant
    theta_gyro = gyro_plant.run(omega_compensated);
    
    Serial.print("Theta is: ");
    Serial.println(theta_gyro,3);
}



