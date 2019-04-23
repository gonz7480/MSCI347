#include <Servo.h>

#define MotorRPin 5
#define MotorLPin 3

Servo motorR;
Servo motorL;

void setup() {
  motorR.attach(MotorRPin);
  motorL.attach(MotorLPin);

  //motors need a stop command before it can move
  motorR.writeMicroseconds(1500); 
  motorL.writeMicroseconds(1500);
  delay(3000);
}

void loop() {
  motorR.writeMicroseconds(1600);
  
//  motorL.writeMicroseconds(1500);
//  motorR.writeMicroseconds(1600);
//  delay(2000);
//
//  motorL.writeMicroseconds(1400);
//  motorR.writeMicroseconds(1500);
//  delay(2000);
//
//  motorL.writeMicroseconds(1400);
//  motorR.writeMicroseconds(1600);
//  delay(2000);
}
