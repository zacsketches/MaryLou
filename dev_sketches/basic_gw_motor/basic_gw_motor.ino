/* 
    Basic gw example to get the motors running
*/

//Data structures
#include <Vector.h>
#include <Pair.h>

//Glow Worm core
#include <clearinghouse.h>

//Glow Worm Messages
#include <messages/control_effort.h>
#include <messages/plant_status.h>

//Glow Worm Blocks
#include <blocks/balance_plant.h>

//other libraries (Servo, Wire, etc)
#include <tools/quadrature.h>
#include <MaryLou.h>

//***********************************************************************
//                               Main Sketch
//***********************************************************************

/*------------Required to initialize GW framework------------------------*/
gw::Clearinghouse ch;
int gw::Message::msg_count = 0;
int gw::Node::node_count = 0;

/*------------Create a set of global messsages---------------------------*/
Control_effort_msg control_effort_msg;
Plant_status_msg plant_status_msg;

/*------------Physical Connections--------------------------------------*/
// This data comes from the MaryLou.h library.  I did this to consolidate
// that info in case I have to change something it won't require 
// editing every dev_sketch.

/*------------Construct the system blocks-------------------------------*/
//Motor(name, dir_pin, pwm_pin, current_sense_pin, Position::position)
gw::Motor* lt_motor = 
  new gw::Motor("lt_mtr", LT_DIR_PIN, LT_PWM_PIN, LT_SENSE_PIN, Position::lt);  
gw::Motor* rt_motor = 
  new gw::Motor("rt_mtr", RT_DIR_PIN, RT_PWM_PIN, RT_SENSE_PIN, Position::rt);  

//Quadrature_encoder<int_A_pin, int_B_pin>(Position::position)
Quadrature_encoder<LT_ENCODER_A_PIN,LT_ENCODER_B_PIN>* lt_encoder = 
  new Quadrature_encoder<LT_ENCODER_A_PIN,LT_ENCODER_B_PIN>(Position::lt,"lt_enc");
Quadrature_encoder<RT_ENCODER_A_PIN,RT_ENCODER_B_PIN>* rt_encoder = 
  new Quadrature_encoder<RT_ENCODER_A_PIN,RT_ENCODER_B_PIN>(Position::rt,"rt_enc");

//Blance_plant()
Balance_plant plant;

/*------------Setup-----------------------------------------------------*/
void setup() {
    Serial.begin(115200);
    Serial.println();

    ch.register_msg(&control_effort_msg);

    ch.list();
    
    plant.print();

    //initialize the balance plant
    rt_motor->reverse();        //right motor is installed opposite left
    lt_motor->reverse();        //right motor is installed opposite left
//    plant.attach(lt_encoder);
//    plant.attach(rt_encoder);
    plant.attach(lt_motor);
    plant.attach(rt_motor);
    plant.begin();
    plant.print();
    
    //set initial conditions..this will update the plant_status_message to
    //the current values for the two encoders..should be zero, but in some
    //cases it may be a small int close to zero.
    control_effort_msg.u = 100;
    plant.run();
    
    delay(2000);
}

/*------------Loop------------------------------------------------------*/
void loop() {
    plant.run();
    plant_status_msg.print(); 
}







