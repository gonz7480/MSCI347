/* GIVE CREDIT TO ORIGINAL SOURCE
*/

//Libraries
#include <SoftwareSerial.h> //TX RX library
#include <Servo.h>  //Servo library
#include <TinyGPS++.h>  //GPS library
#include <Wire.h>  //I2C library
#include <Adafruit_Sensor.h>  //Sensor library
#include <Adafruit_LSM303_U.h>  //Compass library

//Defined pins
#define RXPin 13
#define TXPin 12

//Baud rates
#define GPSBaud 9600
#define ConsoleBaud 115200

#define Pi 3.14159

//Declare servos
Servo RTmtr;
Servo LTmtr;

SoftwareSerial ss(RXPin, TXPin);  //The serial connection to the GPS device

TinyGPSPlus gps;  //The TinyGPS++ object
unsigned long lastUpdateTime = 0;  //Set last updated time to zero

Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);  //Compass object

#define des_LAT 36.6541352 //Destination latitude
#define des_LNG -121.7980495  //Destination longitude

float curr_LAT;
float curr_LNG;
float heading;

float distanceToDestination;
float courseToDestination;
const char *directionToDestination;
int courseChangeNeeded;

float compass() { //Get a new sensor event
  sensors_event_t event;
  mag.getEvent(&event);
  float heading = (atan2(event.magnetic.y, event.magnetic.x) * 180) / Pi;  // Calculate the angle of the vector y,x
  if (heading < 0) {  // Normalize to 0-360
    heading = 360 + heading;
  }
  return heading;
}

void setup() {
  Serial.begin(ConsoleBaud);  //Begin connection with the serial monitor
  ss.begin(GPSBaud);  //Begin software serial connection with GPS
  Serial.print("Testing");
  RTmtr.attach(6);  //Attach the servos to the pins
  LTmtr.attach(7);

  // Initialise the Compass
  if (!mag.begin()) { //Compass failed to initialize, check the connections
    Serial.println("Oops, no Compass detected. Check your wiring!");
    while (1);
  }
}

void loop() {
  //If any characters have arrived from the GPS, send them to the TinyGPS++ object
  while (ss.available() > 0) {
    if (gps.encode(ss.read()));
    //Serial.println("STUCK");
  }
  lastUpdateTime = millis();
  Serial.println();

  curr_LAT = gps.location.lat();
  curr_LNG = gps.location.lng();
  heading = compass();


  //Establish our current status
  distanceToDestination = TinyGPSPlus::distanceBetween(curr_LAT, curr_LNG, des_LAT, des_LNG);
  courseToDestination = TinyGPSPlus::courseTo(curr_LAT, curr_LNG, des_LAT, des_LNG);
  const char *directionToDestination = TinyGPSPlus::cardinal(courseToDestination);

  Serial.println();
  Serial.print("LAT: "); Serial.print(curr_LAT, 6); Serial.print("  LON: "); Serial.println(curr_LNG,6);
  Serial.print("CURRENT ANGLE: "); Serial.println(compass());
  Serial.print("COURSE TO DEST: "); Serial.println(courseToDestination);

  //calculate the difference in angle between current heading and a heading that would lead RoboBuoy straight to the destination
  courseChangeNeeded = heading - courseToDestination;

  if (distanceToDestination <= 1) { //If less than 1 meter away from destination, stay put
    moveMotor(25);
  }else if(distanceToDestination <= 2){ //If less than 2 meters away, go slow
    moveMotor(50);
  }else if(distanceToDestination <= 10){ //If less than 10 meters away, go fairly fast
    moveMotor(150);
  }else{ //Else, go fast
    moveMotor(300);
  }

  Serial.print("DISTANCE: "); Serial.print(distanceToDestination);
  Serial.println(" meters to go."); Serial.print("INSTRUCTION: ");

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

/* Function to define how the thrusters should move/turn
* int mod: an integer that modifies the speed of the thrusters. Must be greater than 25 if using BlueRobotics T100 Thrusters
*/
void moveMotor(int mod){
  switch(courseChange()){
    case 'L':
        RTmtr.writeMicroseconds(1500 - mod);
        LTmtr.writeMicroseconds(1500);
        break;
    case 'R':
        RTmtr.writeMicroseconds(1500);
        LTmtr.writeMicroseconds(1500 + mod);
        break;
    default:
        RTmtr.writeMicroseconds(1500 - mod);
        LTmtr.writeMicroseconds(1500 + mod);
  }
}
/* //deprecated movement functions
void stopRobot() {  // function to stop moving for a period of time
  LTmtr.writeMicroseconds(1500);  // both wheels stop
  RTmtr.writeMicroseconds(1500);
}

void forward(int duration) {  //function to go straight full speed for designated time
  LTmtr.writeMicroseconds(1700);  // left wheel counterclockwise
  RTmtr.writeMicroseconds(1300);  // right wheel clockwise
  delay(duration);
}

void reverse(int duration) {  // function to go backwards full speed for designated time
  LTmtr.writeMicroseconds(1300);  // left wheel clockwise
  RTmtr.writeMicroseconds(1700);  // right wheel counterclockwise
  delay(duration);
}*/
