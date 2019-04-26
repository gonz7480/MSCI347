//------------------------------------//
//             Created by             //
//         Cristian Arguera 2019      //
//           MSCI 437 Capstone       //
//------------------------------------//
#include <Servo.h>
#include <SoftwareSerial.h>
Servo WenchMtr; // Names the motor for later use in the program
const int pinSwitch = 12;  // Pin Reed set for pin 12 on Arduino
const int pinLed    = 9;  //Pin LED
int StatoSwitch = 0;

//KEEP THIS PINS CONSISTENT WITH NAVIGATION
const byte RXNav = 7;
const byte TXNav = 6;

SoftwareSerial navigation (RXNav, TXNav);

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
  navigation.begin(9600);
  WenchMtr.attach(9);
  pinMode(pinSwitch, INPUT);
}

void loop()
{
  navigation.listen(); //listen for signal from GPS

  //while no signal has been sent, stay in setup
  while(!navigation.available()){ break; };

  StatoSwitch = digitalRead(pinSwitch);  // Read the value given by the reed sensor
  if (StatoSwitch == HIGH)
  {
    WenchMtr.detach();

    //Once observatory is up, send "home" signal to navigation
    navigation.println('o');
  }
  else
  {
    up();
  }
}
