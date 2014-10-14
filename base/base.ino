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
//#include <Imu.h>
//#include <Attitude_computer.h>
//#include <State_observer.h>
//#include <Balance_regulator.h>

//Supporting libraries
//#include <Wire.h>

//Macros
#define LOG_UART Serial
#define log(x) LOG_UART.println(x)

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
//Imu(address)
//Imu* imu(imu_address);
//Attitude_computer()
//Attitude_computer computer;
//State_observer()
//State_observer observer;
//Balance_regulator()
//Balance_regulator regulator;

void setup() {
  Serial.begin(115200);
  Serial.println();
  log("test of log macro");
  
  ch.register_msg(&control_effort_msg);
//  ch.register_msg(&plant_status_msg);
  ch.register_msg(&pitch_msg);
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
  control_effort_msg.u = -300;
  
  //initialize the attitude computer
//  imu.begin();
//  computer.attach(imu);
//  computer.begin();
//  computer.print();
  
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
  plant_status_msg.print();  
  delay(1000);
}
