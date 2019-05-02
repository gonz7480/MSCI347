/** This code just drops the Benthic Observatory, the propellors and GPS coordinates are not used
 *  Written by Kaitlyn Beardshear
 */


//Variables to change depth and time spent on bottom
int depth = 1; //measured in meters
int bottomTime = 2; //measured in seconds

#include <Servo.h>  //Servo library

int switchState; //Variable to hold if the switch is closed or not
int reelSwitch = 41; //Pin connected to the reel switch

const int motorController = 30; //Winch controller
int dropTime; //Variable to hold the droptime in ms
int winchWait; //Variable to hold the bottom time in ms

//Declare servos
Servo winchMtr; //winch controller

//function to pull  up the Benthic Observatory
void winchUp() {
  switchState = digitalRead(reelSwitch);
  while (switchState == 1) { //if the switch is not connected, keep pulling up
    switchState = digitalRead(reelSwitch);
    Serial.println(switchState);
    winchMtr.writeMicroseconds(1200);
  }
  if (switchState == 0) {
    winchStop(); //once the switch is connected, stop
  }
}

//Function to drop the Benthic Observatory
void winchDown(int dropTime) {
  winchMtr.writeMicroseconds(1700);
  delay(dropTime);
}

//Function to stop the winch
void winchStop() {
  winchMtr.writeMicroseconds(1500);
}

void winchPause(int winchWait) {
  winchMtr.writeMicroseconds(1500);
  delay(winchWait);
}

void setup() {
  Serial.begin(9600);
  winchMtr.attach(30);
  pinMode(reelSwitch, INPUT_PULLUP);

  dropTime = ((depth) * 7000); //The winch takes approx 7 seconds to unspool 1 meter of string, this calculates
  // how long the winch needs to unspool based on the given depth.
  winchWait = ((bottomTime) * 1000); //This changes the bottom time from seconds to milliseconds

  Serial.println("dropping");
  winchDown(dropTime);
  Serial.println("waiting");
  winchPause(winchWait);
  Serial.println("up");
  winchUp();
}

void loop() {
}
