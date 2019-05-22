#include <SimpleServo.h>

SimpleServo flag_servo;
const int jetson_i = 2;

void setup() {
  // put your setup code here, to run once:
  flag_servo.attach(1);
  pinMode(jetson_i, INPUT);
  flag_servo.write(120);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  int val = digitalRead(jetson_i);

  if (val == HIGH){
    flag_servo.write(0); //on
  }
  else{
    flag_servo.write(120); //off
  }
    
  delay(100);
  
}
