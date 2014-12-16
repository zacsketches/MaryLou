/*
	Began on Oct 9 2014
*/

//data structures
#include <Vector.h>
#include <Pair.h>

//Glow Worm core
#include <clearinghouse.h>

//Physical connections for MaryLou
#include <MaryLou.h>

//Glow Worm messages
#include <messages/plant_status.h>
#include <messages/control_effort.h>
#include <messages/pitch.h>
#include <messages/state_vec.h>

//Glow worm components
#include <blocks/balance_plant.h>
#include <blocks/attitude_computer.h>
#include <blocks/state_observer.h>
//#include <Balance_regulator.h>

//Supporting libraries
#include <Wire.h>
#include <tools/quadrature.h>
#include <L3G.h>
#include <LSM303.h>

//Logging Macros
#define LOG_UART Serial
#define LOG(x) LOG_UART.println(x)
#define LOG_P(x_float, prec) LOG_UART.println(x_float, prec); 

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
/*-----Motors-----*/
//Motor(name, dir_pin, pwm_pin, current_sense_pin, Position::position)
gw::Motor* lt_motor = 
  new gw::Motor("lt_mtr", lt_dir_pin, lt_pwm_pin, lt_sense_pin, Position::lt);  
gw::Motor* rt_motor = 
  new gw::Motor("rt_mtr", rt_dir_pin, rt_pwm_pin, rt_sense_pin, Position::rt);  

/*-----Encoders-----*/
//Quadrature_encoder<int_A_pin, int_B_pin>(Position::position)
Quadrature_encoder<RT_ENCODER_A_PIN, RT_ENCODER_B_PIN> rt_encoder(Position::rt, "rt_enc");
Quadrature_encoder<RT_ENCODER_A_PIN, RT_ENCODER_B_PIN>* rt_ptr = &rt_encoder;
Quadrature_encoder<LT_ENCODER_A_PIN, LT_ENCODER_B_PIN> lt_encoder(Position::lt, "lt_enc");
Quadrature_encoder<LT_ENCODER_A_PIN, LT_ENCODER_B_PIN>* lt_ptr = &lt_encoder;

/*-----Blocks-----*/
//Balance_plant()
Balance_plant plant;
//todo...change this so the gyro and accel are attached...not required by the
//constructor.  This will alow more flexibility in attaching other kinds of gyros
//and/or accelerometers.
//Attitude_computer(L3G&, LSM303&)
L3G gyro;
LSM303 accel;
Attitude_computer computer(gyro, accel);

//State_observer()
State_observer observer;

//Balance_regulator()
//Balance_regulator regulator;

void setup() {
  Serial.begin(115200);
  Serial.println();
  
  Wire.begin();
  gw::wire_begun = true;
  
  LOG("test of log macro");

//Once messages are registered with the clearinghouse by the publisher
//it's not necessary to manually register them with the clearinghouse
//  ch.register_msg(&plant_status_msg);
//  ch.register_msg(&pitch_msg);
//  ch.register_msg(&state_vec_msg);

  ch.register_msg(&control_effort_msg);
  
  ch.list();

    //initialize the balance plant
    rt_motor->reverse();        
    lt_motor->reverse();        
    plant.attach(lt_ptr);
    plant.attach(rt_ptr);
    plant.attach(lt_motor);
    plant.attach(rt_motor);
    plant.begin();
    plant.print();
  
  //create a static control effort msg for debugging
  control_effort_msg.u = 50;
  
  //initialize the attitude computer
  computer.begin();
  computer.set_ki(.0006);
  computer.print();
  
  //initialize the state observer
  observer.begin();
  observer.print();
  
  //initialize the balance regulator
//  regulator.begin();
//  regulator.print();
  
  delay(2000);
}

void loop() {
  plant.run();
//  plant_status_msg.print();  

  computer.run();
//  pitch_msg.print();

  observer.run();
//  state_vec_msg.print();

//  delay(1000);
}
