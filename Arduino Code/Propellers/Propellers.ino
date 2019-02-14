#include <Servo.h>

#define MotorPin 8

Servo motor;

void setup() {
  motor.attach(MotorPin);

}

void loop() {
  motor.writeMicroseconds(1550);
  delay(2000);

  motor.writeMicroseconds(1450);
  delay(2000);

}
