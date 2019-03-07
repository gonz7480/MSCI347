#include <Servo.h>

#define MotorPin 5

Servo motor;

void setup() {
  motor.attach(MotorPin);

  //motors need a stop command before it can move
  motor.writeMicroseconds(1500); 
  delay(3000);
}

void loop() {
  motor.writeMicroseconds(1650);
  delay(3000);
}
