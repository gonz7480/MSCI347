//------------------------------------//
//             Created by             //
//         Cristian Arguera 2019      //
//           MSCI 437 Capstone       //
//------------------------------------//
#include <Servo.h>
Servo WenchMtr; // Names the motor for later use in the program 
const int pinSwitch = 12;  // Pin Reed set for pin 12 on Arduino 
const int pinLed    = 9;  //Pin LED
int StatoSwitch = 0;
void up (){
   WenchMtr.writeMicroseconds(1200);
  ;
}
void down (int time){
  WenchMtr.writeMicroseconds(1700);
  delay(time);
}
void pause (int time){
  WenchMtr.writeMicroseconds(1500);
  delay(time);
}
void setup()
{
  WenchMtr.attach(9);
  pinMode(pinSwitch, INPUT);
}
void loop()
{
  StatoSwitch = digitalRead(pinSwitch);  // Read the value given by the reed sensor
  if (StatoSwitch == HIGH) 
  {
    WenchMtr.detach();
  }
  else
  {
    up();
  }
}
