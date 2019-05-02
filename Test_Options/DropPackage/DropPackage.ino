/** This code just drops the Benthic Observatory, the propellors and GPS coordinates are not used
    Written by Kaitlyn Beardshear
*/

//Variables to change based on the launch
int depth = 3; //How deep the Benthic Observatory is lowered (measured in meters)
int bottomTime = 30; //How long the Benthic Observatory sits on the seafloor (measured in seconds)
int delayStart = 5; //How long to wait before dropping the Benthic Observatory (measured in minutes)

#include <Servo.h>  //Servo library

//Defined pins and global variables
//The reel switch is used to stop the winch
int reelState; //Variable to hold if the switch is closed or not
#define reelSwitch 41 //Pin connected to the reel switch

//Winch variables
#define motorController 30 //Winch controller
int delayTime; //Variable to hold the start delay in ms
int dropTime; //Variable to hold the droptime in ms
int winchWait; //Variable to hold the bottom time in ms
int dropLoops; //The number of times the winchDown loop cycles

Servo winchMtr; //Declare Winch controller

//Function to pull  up the Benthic Observatory
void winchUp () {
  reelState = digitalRead(reelSwitch);
  while (reelState == 1) { //If the switch is not connected, keep pulling up
    reelState = digitalRead(reelSwitch);
    winchMtr.writeMicroseconds(1200);
  }
  if (reelState == 0) {
    winchStop(); //Once the switch is connected, stop
  }
}

//Function to drop the Benthic Observatory
void winchDown() {
  for (int i = 0; i < dropLoops; i++) { //Checks the reelSwitch every 200ms just in case the line becomes tangled while dropping
    reelState = digitalRead(reelSwitch);
    if (reelState == 0) { //If the switch is connected, stop
      winchStop(); 
    } else {
      winchMtr.writeMicroseconds(1700);
    }
    delay(200);
  }
}

//Function to stop the winch
void winchStop() {
  winchMtr.writeMicroseconds(1500);
}

//Function for the winch to hold the Benthic Observatory at the desired depth
void winchPause(int winchWait) {
  winchMtr.writeMicroseconds(1500);
  delay(winchWait);
}

void startDelay(int delayTime) {
  delay(delayTime);
}

void setup() {
  Serial.begin(9600);  //Start the serial monitor
  winchMtr.attach(30);  //Attach the motor 
  pinMode(reelSwitch, INPUT_PULLUP);  //Set the reelSwitch as an input

  delayTime = ((delayStart) * 60000); //This changes the start delay from minutes to ms
  dropTime = ((depth) * 7000); //The winch takes approx 7 seconds to unspool 1 meter of string, this calculates
  // how long the winch needs to unspool based on the given depth.
  dropLoops = ((dropTime) / 200); //The loop needs to cycle every 200 ms for the total amount of time
  winchWait = ((bottomTime) * 1000); //This changes the bottom time from seconds to milliseconds

//Current test
startDelay(delayTime);
winchDown();
winchPause(winchWait);
winchUp();
}

void loop() {
}
