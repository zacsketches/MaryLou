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

void setup() {  
  pinMode(rt_dir_pin, OUTPUT); //Initiates Motor Channel A pin
  pinMode(lt_dir_pin, OUTPUT); //Initiates Motor Channel A pin
}

void loop(){
    //forward @ half speed
  digitalWrite(rt_dir_pin, HIGH); //Establishes forward direction of Channel A
  analogWrite(rt_pwm_pin, 100);   //Spins the motor on Channel A at full speed
  digitalWrite(lt_dir_pin, HIGH); //Establishes forward direction of Channel A
  analogWrite(lt_pwm_pin, 100);   //Spins the motor on Channel A at full speed
  
  delay(3000);
  
  //backward @ half speed
  digitalWrite(rt_dir_pin, LOW); //Establishes backward direction of Channel A
  analogWrite(rt_pwm_pin, 123);   //Spins the motor on Channel A at half speed
  digitalWrite(lt_dir_pin, LOW); //Establishes backward direction of Channel A
  analogWrite(lt_pwm_pin, 123);   //Spins the motor on Channel A at half speed
  
  delay(3000);
  
  digitalWrite(9, HIGH); //Eengage the Brake for Channel A
  
  delay(1000);
  
}

