/*
    Script to test if the motors encoders are connected correctly.
*/

#include <MaryLou.h>
#include <quadrature.h>

/*
	Because the attachInterrupt(int, function_ptr, const) requires
    a function pointer as its final argument the encoder class
    cannot use normal construction.  Each instance of 
    the Quadrature_encoder must be a singleton class constructed
    from the template with the two integer arguments of the 
    interrupt pins used for the encoder.  For example to build
    a quadrature encoder to use int0 and int1 on the Arduino Uno
    you would construct it like this:
        Quadrature_encoder<0, 1> encoder;

	The following enums are provided in the header file:
    Motion::motion {frwd, back, stop}
	Board::board {uno, due}
	
	The constructor for a Quadrature_encoder requires a board type
	so it an configure the interrupt pins properly.  The default is
	an Arduino Uno.
	Quadrature_encoder<PIN_A, PIN_B> (Board::board b = Board::uno)

    The public methods of Quadrature_encoder are:
		begin() - must be called in setup
		count() - returns a long of the encoder count
		motion() - returns a value from the Motion::motion enum
			corresponding to what direction the shaft has turned
			since the last time motion() was called.
                reverse() - changes the polarity of the decoder in case
                        you need to change the sign of the output.  Run
                        the example below with the .reverse() line commented
                        out and see the difference in output.
   
    So to use the encoder you would construct an object with
    the template parameters and a board definition.  Then you would call 
    encoder.begin() in the setup() of the sketch.  Then you
    can call encoder.count() or encoder.motion() anywhere else
    in the sketch to read the encoder.
*/

Quadrature_encoder<RT_ENCODER_A_PIN, RT_ENCODER_B_PIN> rt_encoder(Board::due);
Quadrature_encoder<RT_ENCODER_A_PIN, RT_ENCODER_B_PIN>* rt_ptr = &rt_encoder;

Quadrature_encoder<LT_ENCODER_A_PIN, LT_ENCODER_B_PIN> lt_encoder(Board::due);
Quadrature_encoder<LT_ENCODER_A_PIN, LT_ENCODER_B_PIN>* lt_ptr = &lt_encoder;

void setup() {
  Serial.begin(115200);
  Serial.println();

//    rt_encoder.begin();
  rt_ptr->begin();
  lt_ptr->begin();
}

void loop() {
  //read the encoder
//  long rt_ct = rt_encoder.count();
    long rt_ct = rt_ptr->count();
    long lt_ct = lt_ptr->count();
  
  //format the output for printing 
  char buf[50];
  sprintf(buf, "rt count is: %d\tlt count is:%d", rt_ct, lt_ct);
  Serial.println(buf);

  //format the output for printing 
//  char buf[50];
//  sprintf(buf, "rt count is: %d", rt_ct);
//  Serial.println(buf);

  
  //check the motion
//  Motion::motion m = rt_encoder.motion();
//  Serial.println(text(m));

  //delay the loop for a moment.
  //Note that the encoder continues to count even during this
  //delay because it is interrupt driven!
  delay(100);
}
