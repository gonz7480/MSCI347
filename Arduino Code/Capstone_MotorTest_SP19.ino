#include <Servo.h>
Servo mtrwnch;

void setup() {
  mtrwnch.attach(5);                //attached to pin 5
}

void loop() {
up();                               //indefinite movement on loop; need to add delay function
}

void pause(){
  mtrwnch.writeMicroseconds(1500);  //command to stop moving the motor
}

void up (){
  mtrwnch.writeMicroseconds(1200);  //command for motor to reel in
}

void down(){
  mtrwnch.writeMicroseconds(1700);  //command for motor to reel back down
}
