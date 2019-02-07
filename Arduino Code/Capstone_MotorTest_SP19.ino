#include <Servo.h>
Servo mtrwnch;

void setup() {
  mtrwnch.attach(5); 
}

void loop() {
up();
}

void stop(){
mtrwnch.writeMicroseconds(1500);
delay(500);
}

void up (){
mtrwnch.writeMicroseconds(1200);
}
