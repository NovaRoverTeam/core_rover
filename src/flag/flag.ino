#include <Servo.h>

Servo flag_servo;
const int jetson_i = 7;

void setup() {
  // put your setup code here, to run once:
  flag_servo.attach(6);
  pinMode(jetson_i, INPUT);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  int val = digitalRead(jetson_i);

  if (val == HIGH){
    flag_servo.write(20); //on
   }
  else{
    flag_servo.write(120); //off
    }
    
  delay(100);
  
}
