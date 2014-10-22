/*
	Began on Oct 9 2014
*/

//data structures
#include <Vector.h>
#include <Pair.h>

//Glow Worm core
#include <clearinghouse.h>

//Glow Worm messages
#include <messages/plant_status.h>
#include <messages/control_effort.h>
#include <messages/pitch.h>
#include <messages/state_vec.h>

//Glow worm components
#include <quadrature.h>
#include <balance_plant.h>
#include <L3G.h>
#include <LSM303.h>
#include <attitude_computer.h>
//#include <State_observer.h>
//#include <Balance_regulator.h>

//Supporting libraries
#include <Wire.h>

//Logging Macros
#define LOG_UART Serial
#define log(x) LOG_UART.println(x)
#define Log_p(x, y) LOG_UART.println(x, y);

//Physical connections
const int lt_dir_pin = 12;
const int lt_pwm_pin = 3;
const int lt_sense_pin = A0;
const int lt_encoder_A_pin = 7;
const int lt_encoder_B_pin = 6;

const int rt_dir_pin = 13;
const int rt_pwm_pin = 11;
const int rt_sense_pin = A1;
const int rt_encoder_A_pin = 5;
const int rt_encoder_B_pin = 4;

//const int imu_address;

/*------------Required to initiate the Glow Worm Framework---------------*/
gw::Clearinghouse ch;
int gw::Message::msg_count = 0;
int gw::Node::node_count = 0;

/*------------Create a set of global messsages---------------------------*/
Control_effort_msg control_effort_msg;
Plant_status_msg plant_status_msg;
Pitch_msg pitch_msg;
State_vec_msg state_vec_msg;

/*------------Construct the system components----------------------------*/
//Motor(name, dir_pin, pwm_pin, current_sense_pin, Position::position)
gw::Motor* lt_motor = 
  new gw::Motor("lt_mtr", lt_dir_pin, lt_pwm_pin, lt_sense_pin, Position::lt);  
gw::Motor* rt_motor = 
  new gw::Motor("rt_mtr", rt_dir_pin, rt_pwm_pin, rt_sense_pin, Position::rt);  
//Quadrature_encoder<int_A_pin, int_B_pin>(Position::position)
Quadrature_encoder<lt_encoder_A_pin,lt_encoder_B_pin>* lt_encoder = 
  new Quadrature_encoder<lt_encoder_A_pin,lt_encoder_B_pin>(Position::lt,"lt_enc");
Quadrature_encoder<rt_encoder_A_pin,rt_encoder_B_pin>* rt_encoder = 
  new Quadrature_encoder<rt_encoder_A_pin,rt_encoder_B_pin>(Position::rt,"rt_enc");
//Blance_plant()
Balance_plant plant;

//todo...change this so the gyro and accel are attached...not required by the
//constructor.  This will alow more flexibility in attaching other kinds of gyros
//and/or accelerometers.
//Attitude_computer(L3G&, LSM303&)
L3G gyro;
LSM303 accel;
Attitude_computer computer(gyro, accel);

//State_observer()
//State_observer observer;
//Balance_regulator()
//Balance_regulator regulator;

void setup() {
  Serial.begin(115200);
  Serial.println();
  
  Wire.begin();
  gw::wire_begun = true;
  
  log("test of log macro");

//Once messages are registered with the clearinghouse by the publisher
//it's not necessary to manually register them with the clearinghouse
//  ch.register_msg(&plant_status_msg);
//  ch.register_msg(&pitch_msg);
  ch.register_msg(&control_effort_msg);
  ch.register_msg(&state_vec_msg);
  
  ch.list();

  //initialize the balance plant
  rt_motor->reverse();        //right motor is installed opposite left
  rt_encoder->reverse();      //right motor encoder also installed opposite
  plant.attach(lt_encoder);
  plant.attach(rt_encoder);
  plant.attach(lt_motor);
  plant.attach(rt_motor);
  plant.begin();
  plant.print();
  
  //create a static control effort msg for debugging
  control_effort_msg.u = 400;
  
  //initialize the attitude computer
  computer.begin();
  computer.set_ki(.0006);
  computer.print();
  
  //initialize the state observer
//  observer.begin();
//  observer.print();
  
  //initialize the balance regulator
//  regulator.begin();
//  regulator.print();
  
  delay(2000);
}

void loop() {
  plant.run();
//  plant_status_msg.print();  

  computer.run();
  pitch_msg.print();


//  delay(1000);
}
