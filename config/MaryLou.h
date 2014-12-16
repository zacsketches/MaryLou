#ifndef MARYLOU_CONFIG
#define MARYLOU_CONFIG DEC_2014

//Physical connections
//The left motor is B on the motor shield
#define LT_DIR_PIN  13
#define LT_PWM_PIN  11
#define LT_SENSE_PIN  A1
#define LT_ENCODER_A_PIN  7
#define LT_ENCODER_B_PIN  6

//The right motor is A on the motor shield
#define RT_DIR_PIN  12
#define RT_PWM_PIN  3
#define RT_SENSE_PIN  A0
#define RT_ENCODER_A_PIN  5
#define RT_ENCODER_B_PIN  4

// 2A / 1023 ADC resolution = .00196
// This value is used in converting values from the current sensing into
// Amps running into the motor 
#define CONV_I 0.00196;

// Max effort taken from model of controller in MATLAB simulation
// to disturbances...for now I'm using 255 until I run the MATLAB model
// some more.
#define MAX_EFFORT 255;

// Deadband data taken on 12/14/14.
// Deadband_solver with step size 10, 1500 delay, 30 starting effort and 
// five loops.  MaryLou was suspended in the vice and connected 
// to 12V benchtop power supply.  I probably need to run the same test on
// the battery, and consider whether I want dead band for suspended ops
// or with the robot weight on the wheels....but this should work for now.
#define RT_FWD_DEAD 50
#define RT_BCK_DEAD -72
#define LT_FWD_DEAD 46
#define LT_BCK_DEAD -66

// Conversion factors used moved from radians to degrees and vice versa.
#define RAD_TO_DEG 57.2958  		// 180 / pi
#define DEG_TO_RAD .0174533 		// pi / 180

// Bias constants in the accelerometer and gyro
#define THETA_BIAS -0.43329    //FROM EXPERIMENTAL DATA WITH ACCEL_TEST.INO
#define DC_OFFSET -223         //From experimental data with gyro_test.ino
#define PITCH_AXIS x    //Which axis are the Euler angles measured around.  Changes dependent
#define ROLL_AXIS  y    //upon the physical installation of the MEMS sensor on the bot.
#define YAW_AXIS   z
#endif