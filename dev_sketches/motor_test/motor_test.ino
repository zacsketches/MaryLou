/*
	Motor test
	
	Test the physcial connections in conjunction with the Arduino
	Motor Shield to ensure we get movent in all directions.
	
	Use the physical pin connections from the MaryLou library
*/

#include <MaryLou.h>

void setup() {  
  pinMode(RT_DIR_PIN, OUTPUT); //Initiates Motor Channel A pin
  pinMode(LT_DIR_PIN, OUTPUT); //Initiates Motor Channel A pin
}

void loop(){
  //left forward @ half speed
  analogWrite(RT_PWM_PIN, 0);   //Spins the motor on Channel A at half speed
  digitalWrite(LT_DIR_PIN, HIGH); //Establishes forward direction of Channel A
  analogWrite(LT_PWM_PIN, 100);   //Spins the motor on Channel A at 100 speed
  delay(3000);

  //right forward @ half speed
  analogWrite(LT_PWM_PIN, 0);   //Spins the motor on Channel A at half speed
  digitalWrite(RT_DIR_PIN, HIGH); //Establishes forward direction of Channel A
  analogWrite(RT_PWM_PIN, 100);   //Spins the motor on Channel A at 100 speed
  delay(3000);

  
  //left backward @ half speed
  analogWrite(RT_PWM_PIN, 0);
  digitalWrite(LT_DIR_PIN, LOW); //Establishes backward direction of Channel A
  analogWrite(LT_PWM_PIN, 123);   //Spins the motor on Channel A at half speed
  delay(3000);
  
  //right backward @ half speed
  analogWrite(LT_PWM_PIN, 0);   //Spins the motor on Channel A at half speed
  digitalWrite(RT_DIR_PIN, LOW); //Establishes forward direction of Channel A
  analogWrite(RT_PWM_PIN, 100);   //Spins the motor on Channel A at 100 speed
  delay(3000);
  
  //coast both motors to a stop
  analogWrite(RT_PWM_PIN, 0);   //Spins the motor on Channel A at half speed
  analogWrite(LT_PWM_PIN, 0);   //Spins the motor on Channel A at half speed

  delay(3000);
}

