// Compatible with test branch of LSM303

/*
	This dev sketch connects directly to the Pololu IMU
	over I2C.  It tests the bottom PCB and allow me to configure
	the sensor for offset.

    Input data from the LSM303DLHC 3-axis accel populated on the
	Pololu IMU v2.

    (c) Zac Staples zacsketches (at) github.

    You can use this software as you see fit provided you
    acknowledge the source.

    This software builds on the LSM303 library provided by Pololu
    and forked by me at https://github.com/zacsketches/lsm303-arduino.
    My fork of this library adds several templated vector math
    functions in the header file, rearranges the file structure so
    it can be included easily in Arduino sketches if you clone the
    whole repo into your Arduino/library folder.  I also change the
    vector names to capital letters so the nomenclature in the program
    reflects mathematics texts a little better.

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
      All axes enabled (X, Y, Z)
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
typedef LSM303::vector<int16_t> vector;

/*
   For MaryLou, I have the main IMU connected to the
   I2C port closest to the AREF pin.  So I need to
   construct the LSM303 object to use the Due's
   secondary I2C port and to use the correct .begin()
   method in the setup() function.
*/
LSM303 accel(LSM303::I2C_port::secondary);
//LSM303 accel2(LSM303::I2C_port::primary);

/*
	This sketch will zero the bias of the accelerometer
	by taking the average over a series of samples and then
	subtracting these tare values when the device is in motion.
	The sample_num value determines how many samples are
	averaged to calculate the bias.
*/
const int    sample_num  = 100;      //scalar


// Global variables
const double sensitivity = 1.0;     //mG / LSB, datasheet page 9
const int    sample_time = 20;      //ms...try to run at 50hz
vector             raw_g;
double         raw_angle = 0.0;
double compensated_angle = 0.0;
double  pitch_angle_bias = 0.0;
Time                 now = 0;

int16_t& pitch_axis = accel.A.x;
int16_t& roll_axis = accel.A.y;
int16_t& yaw_axis = accel.A.z;

void setup() {
  Serial.begin(115200);
  Serial.println();

  /* I use the Arduino Due for many of my projects which
     has two I2C ports.  The default I2C port is accessed
     on pin 20 (SDA) and 21 (SCL).  The secondary port is
     close to the AREF pin.  The command
  		Wire.begin()
     initializes the default port.  Whereas the command
  		Wire1.begin()
     initializes the secondary port.
  */
  Wire1.begin();

  while (!accel.init());
  Serial.println("Accel initialized");

  accel.enableDefault();

  //Measure DC Offset and/or mounting position errors
  Serial.println("Computing bias");
  for (int i = 1; i <= sample_num; ++i) {
    accel.read();
    accel.shift_accel();
    float Gx = pitch_axis;
    float Gy = roll_axis;
    float Gz = -1 * yaw_axis;
    double phi = atan2(Gy, Gz);

    /*
       recursive average on phi
       see example at:
       http://people.revoledu.com/kardi/tutorial/RecursiveStatistic/Time-Average.htm
    */
    float n = (float)i;
    pitch_angle_bias = (n - 1) / n * pitch_angle_bias + 1 / n * phi;

    if (i % 10 == 0) {
      Serial.print("    Taking sample: ");
      Serial.println(i);
    }
  }

  print_config();

  accel.read();
  accel.shift_accel();
  float Gx = pitch_axis;
  float Gy = roll_axis;
  float Gz = -1 * yaw_axis;
  double new_phi = atan2(Gy, Gz);
  Serial.print("instant phi is: ");
  Serial.println(new_phi * rad_to_deg);

  delay(5000);
}

void loop() {
  //measure gravity and pitch angle.
  if (millis() - now > sample_time) {
    now = millis();
    accel.read();
    accel.shift_accel();
    float Gx = pitch_axis;
    float Gy = roll_axis;
    float Gz =  -1 * yaw_axis;
    double phi = atan2(Gy, Gz);

    raw_angle = phi;
    compensated_angle = raw_angle - pitch_angle_bias;
    raw_angle *= rad_to_deg;
    compensated_angle *= rad_to_deg;
    print_angle();
  }
}

void print_config() {
  Serial.print("Raw gravity vector:\n");
  vector_print(raw_g);

  Serial.print("Pitch angle bias: ");
  Serial.println(pitch_angle_bias * rad_to_deg, 5);

  Serial.println("-------------------------");
}

void print_angle() {
  Serial.print("Raw pitch is: ");
  Serial.print(raw_angle, 3);
  Serial.print("\tCompensated pitch is: ");
  Serial.println(compensated_angle, 3);
}
