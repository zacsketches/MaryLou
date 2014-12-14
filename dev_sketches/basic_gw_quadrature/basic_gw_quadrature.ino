/* 
    Basic gw example to get quadrature working
*/

//Data structures
#include <Vector.h>
#include <Pair.h>

//Glow Worm core
#include <clearinghouse.h>

//other libraries (Servo, Wire, etc)
#include <tools/quadrature.h>
#include <MaryLou.h>

//***********************************************************************
//                               Main Sketch
//**********************************************************************

/*------------Required to initialize GW framework------------------------*/
gw::Clearinghouse ch;
int gw::Message::msg_count = 0;
int gw::Node::node_count = 0;

//Quadrature_encoder<int_A_pin, int_B_pin>(Position::position)
//Quadrature_encoder<RT_ENCODER_A_PIN,RT_ENCODER_B_PIN>* rt_encoder = 
//  new Quadrature_encoder<RT_ENCODER_A_PIN,RT_ENCODER_B_PIN>(Position::rt,"rt_enc");
Quadrature_encoder<LT_ENCODER_A_PIN,LT_ENCODER_B_PIN>* lt_encoder = 
  new Quadrature_encoder<LT_ENCODER_A_PIN,LT_ENCODER_B_PIN>(Position::lt,"lt_enc");

/*------------Setup-----------------------------------------------------*/
void setup() {
    Serial.begin(115200);
    Serial.println();

    lt_encoder->begin();
//    lt_encoder->reverse();
}

/*------------Loop------------------------------------------------------*/
void loop() {
    Serial.println(lt_encoder->count());
}







