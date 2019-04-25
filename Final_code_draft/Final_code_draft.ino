//Libraries
#include <SoftwareSerial.h> //TX RX library
#include <Servo.h>  //Servo library
#include <TinyGPS++.h>  //GPS library
#include <Adafruit_Sensor.h>  //Sensor library
#include <Adafruit_LSM303_U.h>  //Compass library

//Toggle this boolean to turn on print statements for debuggings
bool debug = false;

//destination lat and lon
float des_LAT = 36.60337222;
float des_LNG = -121.88472222;

//GPS connection pins
const byte RXGPS 5
const byte TXGPS 4

//Winch connection pins
const byte RXWinch = 6;
const byte TXWinch = 7;

//Reed switch pin (master shut-off)
const int REED_PIN = 2;

//Baud rates
#define SerialBaud 9600
#define DebugBaud 115200

#define Pi 3.14159

//Declare servos
Servo RTmtr;
Servo LTmtr;

SoftwareSerial ss(RXGPS, TXGPS);  //The serial connection to the GPS device
SoftwareSerial winch(RXWinch, TXWinch); //Serial conncetion with winch Arduino

TinyGPSPlus gps;  //The TinyGPS++ object
unsigned long lastUpdateTime = 0;  //Set last updated time to zero

//Compass object
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);

//lat and lon of launching point
float home_LAT;
float home_LNG;

//current lat, lon, and heading
float curr_LAT;
float curr_LNG;
float heading;

float distanceToDestination;
float courseToDestination;
int courseChangeNeeded;

//boolean used with getHome()
bool launch = true;

// Function to read the magnetometer and convert output to degrees
float compass() { //Get a new sensor event
  sensors_event_t event;
  mag.getEvent(&event);
  float heading = (atan2(event.magnetic.y, event.magnetic.x) * 180) / Pi;  // Calculate the angle of the vector y,x
  if (heading < 0) {  // Normalize to 0-360
    heading = 360 + heading;
  }
  return heading;
}

/* Function to determine which direction RoboBuoy needs to turn to correct its heading
 * Returns a character defining which direction to turn.
 * 'R' for right, 'L' for left, 'N' for none
 */
char courseChange(){
  if(courseChangeNeeded > -10 && courseChangeNeeded < 10){
    return 'N';
  }
  else if(courseChangeNeeded < -180){
    return 'L';
  }
  else if(courseChangeNeeded < 180 && courseChangeNeeded > 0){
    return 'L';
  }
  else if(courseChangeNeeded >= 180){
    return 'R';
  }
  else if(courseChangeNeeded >= -180  && courseChangeNeeded < 0){
    return 'R';
  }
  else{return 'N';}
}

// Function to stop the motors
void stop(){
   RTmtr.writeMicroseconds(1500);
   LTmtr.writeMicroseconds(1500);
   delay(1500);
}


/* Function to define how the thrusters should move/turn
 * int mod: an integer that modifies the speed of the thrusters.
 *          Must be greater than 25 if using BlueRobotics T100 Thrusters
 * char dir: a character indicating the direction to turn
 *           'L' for left, 'R' for right
 * int wait: an integer to set the delay when not turning
 */
void moveMotor(int mod, char dir, int wait = 1000){
  switch(dir){
    case 'L':
        RTmtr.writeMicroseconds(1500 - mod);
        LTmtr.writeMicroseconds(1500);
        delay(500);
        break;
    case 'R':
        //while(mod > 0){
        RTmtr.writeMicroseconds(1500);
        LTmtr.writeMicroseconds(1500 + mod);
        delay(500);
        break;
    default:
        RTmtr.writeMicroseconds(1500 - mod);
        LTmtr.writeMicroseconds(1500 + mod);
        delay(wait);
  }

}

//Function to save the GPS coordinates of where RoboBuoy is launched
void getHome(){
  if(launch){
    home_LAT = gps.location.lat();
    home_LNG = gps.location.lng();
  }
  launch = false;
}

//Function to set destination to launching point for retrieval
void goHome(){
  des_LAT = home_LAT;
  des_LNG = home_LNG;
}

float LipoRatio = 4.144670051;

void setup() {
  Serial.begin(DebugBaud);  //Begin connection with the serial monitor
  ss.begin(SerialBaud);  //Begin software serial connection with GPS
  RTmtr.attach(5);  //Attach the servos to the pins
  LTmtr.attach(3);

  pinMode(REED_PIN, INPUT_PULLUP);

  // Initialise the Compass
  if (!mag.begin()) { //Compass failed to initialize, check the connections
    Serial.println("Oops, no Compass detected. Check your wiring!");
    while (1);
  }

  //T100 trusters need a stop command to initialize
  RTmtr.writeMicroseconds(1500);
  LTmtr.writeMicroseconds(1500);
  delay(2000);
}


void loop() {
  if(digitalRead(REED_PIN) == LOW){
    stop();
  }

  int sensorvalue = analogRead(A8); //read battery

  float voltage = (sensorvalue*.00488759);
  float LipoVoltage = (voltage * LipoRatio);
  Serial.println(voltage);
  Serial.println(sensorvalue);
  Serial.println(LipoVoltage);
  delay(500);

  if (LipoVoltage < 14.3){
    stop();
  }
/* Compass code based on the example in the Adafuit_LSM303_U library.
 * Select GPS lines are from the example in the TinyGPS++ library.
 * All movement functions were written primarily by:
 *            Sophia Rose, Andrew Beardshear, Andrew Reyna
 */

  //Check if manual override has been initiated
  //If yes, use manual override
  //break;

  //Check if signal from Winch has been received
  //If yes, goHome();
  winch.listen();
  if(winch.available()){
    goHome();
  }

  //If any characters have arrived from the GPS,
  //send them to the TinyGPS++ object
  while (ss.available() > 0) {
    if (gps.encode(ss.read()));
  }

  //Save initial GPS location
  if(gps.location.isValid() && launch == true){getHome();}

  //Save current lat, lon, and heading
  curr_LAT = gps.location.lat();
  curr_LNG = gps.location.lng();
  heading = compass();


  //Establish our current status
  //These two lines are from the TinyGPS++ example
  distanceToDestination = TinyGPSPlus::distanceBetween(curr_LAT, curr_LNG, des_LAT, des_LNG);
  courseToDestination = TinyGPSPlus::courseTo(curr_LAT, curr_LNG, des_LAT, des_LNG);

  if(debug){
    Serial.println();
    Serial.println(gps.location.isValid());
    Serial.print("LAT: "); Serial.print(curr_LAT, 6); Serial.print("  LON: "); Serial.println(curr_LNG,6);
    Serial.print("CURRENT ANGLE: "); Serial.println(heading);
    Serial.print("COURSE TO DEST: "); Serial.println(courseToDestination);
    Serial.print("DISTANCE: "); Serial.print(distanceToDestination);
    Serial.println(" meters to go."); Serial.print("INSTRUCTION: ");
  }

  //calculate the difference in angle between current heading
  //and a heading that would lead RoboBuoy straight to the destination
  courseChangeNeeded = heading - courseToDestination;

  //If less than 1 meter away from destination, stay put
  if (distanceToDestination <= 1) {
    //When initially true, send signal to winch to lower benthic observatory
    winch.println('x');

    moveMotor(25, courseChange());

    if(debug){
      Serial.print("crawl");
      Serial.println(courseChange());
    }
  }//If less than 2 meters away, go slow
  else if(distanceToDestination <= 2){
    moveMotor(50, courseChange());
    moveMotor(50, 'N');

    if(debug){
      Serial.print("slow ");
      Serial.println(courseChange());
    }
  }//If less than 10 meters away, go fairly fast
  else if(distanceToDestination <= 10){
    moveMotor(150, courseChange());
    moveMotor(150, 'N');

    if(debug){
      Serial.print("medium ");
      Serial.println(courseChange());
    }
  }else{ //Else, go fast
    moveMotor(200, courseChange());
    moveMotor(200, 'N', 5000);

    if(debug){
      Serial.print("fast ");
      Serial.println(courseChange());
    }

  }

}
