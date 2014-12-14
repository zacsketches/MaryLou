/*
    Basic gw example to get quadrature working.  This extends the raw sketch to wrap
    the encoders in the GW framework.
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
Quadrature_encoder<RT_ENCODER_A_PIN, RT_ENCODER_B_PIN> rt_encoder(Position::rt, "rt_enc");
Quadrature_encoder<RT_ENCODER_A_PIN, RT_ENCODER_B_PIN>* rt_ptr = &rt_encoder;

Quadrature_encoder<LT_ENCODER_A_PIN, LT_ENCODER_B_PIN> lt_encoder(Position::lt, "lt_enc");
Quadrature_encoder<LT_ENCODER_A_PIN, LT_ENCODER_B_PIN>* lt_ptr = &lt_encoder;

/*------------Setup-----------------------------------------------------*/
void setup() {
  Serial.begin(115200);
  Serial.println();
  
  Vector<Encoder*> encoders;
  encoders.push_back(rt_ptr);
  encoders.push_back(lt_ptr);
  
  for(int i=0; i<encoders.size(); i++) {
      encoders[i]->begin();   
  }

//  rt_ptr->begin();
//  lt_ptr->begin();

}

/*------------Loop------------------------------------------------------*/
void loop() {
  long rt_ct = rt_ptr->count();
  long lt_ct = lt_ptr->count();

  //format the output for printing
  char buf[50];
  sprintf(buf, "rt count is: %d\tlt count is:%d", rt_ct, lt_ct);
  Serial.println(buf);
  
  //delay to slow down the print loop, but not the encoders since they are interrupt
  //driven.
  delay(100);
}







