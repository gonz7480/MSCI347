#include <Servo.h>
Servo mtrwnch;

void setup() {
  mtrwnch.attach(5);                //attached to pin 5
  
  up();                             //time in milliseconds
  down();
  pause();
}

void loop() {
}

void pause(int time){
  mtrwnch.writeMicroseconds(1500);  //command to stop moving the motor
  delay(time);
}

void up (int time){
  mtrwnch.writeMicroseconds(1200);  //command for motor to reel in
  delay(time);
}

void down(int time){
  mtrwnch.writeMicroseconds(1700);  //command for motor to reel back down
  delay(time);
}
