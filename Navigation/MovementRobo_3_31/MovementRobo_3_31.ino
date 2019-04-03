/* GIVE CREDIT TO ORIGINAL SOURCE
    This is currently the best working code
    Walking to the coordinates works
    Servos do not work
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

float compass() { //Get a new sensor event
  sensors_event_t event;
  mag.getEvent(&event);
  //float Pi = 3.14159;
  float heading = (atan2(event.magnetic.y, event.magnetic.x) * 180) / Pi;  // Calculate the angle of the vector y,x
  if (heading < 0) {  // Normalize to 0-360
    heading = 360 + heading;
  }
  return heading;
}

void loop() {
  //If any characters have arrived from the GPS, send them to the TinyGPS++ object
  while (ss.available() > 0) {
    if (gps.encode(ss.read()));
    //Serial.println("STUCK");
  }
  //Every 5 seconds, do an update.
  //if (millis() - lastUpdateTime >= 1000) {
    lastUpdateTime = millis();
    Serial.println();

    curr_LAT = gps.location.lat();
    curr_LNG = gps.location.lng();
    heading = compass();


    //Establish our current status
    distanceToDestination = TinyGPSPlus::distanceBetween(curr_LAT, curr_LNG, des_LAT, des_LNG);
    courseToDestination = TinyGPSPlus::courseTo(curr_LAT, curr_LNG, des_LAT, des_LNG);
    const char *directionToDestination = TinyGPSPlus::cardinal(courseToDestination);
    //int courseChangeNeeded = (int)(360 + courseToDestination - heading) % 360;

    //courseChangeNeeded is a value between 0 and 360 where 0/360 indicates that the destination is straight ahead from current location
    //The original code calculated current course by calculating the angle between current GPS location and the
    //previous location (gps.course.deg() from TinyGPSPlus). People have noticed inaccuracies in course calculation when speed is less than 5kph.
    //For speeds less than 5kph, I've replaced the TinyGPS function with the angle from the compass.


//    float l = gps.location.lat();
//    float m = gps.location.lng();
    Serial.println();
    Serial.print("LAT: "); Serial.print(curr_LAT, 6); Serial.print("  LON: "); Serial.println(curr_LNG,6);
    Serial.print("CURRENT ANGLE: "); Serial.println(compass());
    Serial.print("COURSE TO DEST: "); Serial.println(courseToDestination);


    courseChangeNeeded = heading - courseToDestination;

    if (distanceToDestination <= 2) {
      moveMotor(50);
    }else if(distanceToDestination <= 10){
      moveMotor(150);
    }else{
      moveMotor(300);
    }
    /*else if(courseChangeNeeded > -10 && courseChangeNeeded < 10){
      Serial.print("Turn forward. "); Serial.println(courseChangeNeeded);
    }
    else if(courseChangeNeeded < -180){
      turnLeft(5);
      Serial.print("Turn to the left: "); Serial.println(360 - (-1*courseChangeNeeded));
    }
    else if(courseChangeNeeded < 180 && courseChangeNeeded > 0){
      turnLeft(5);
      Serial.print("Turn to the left: "); Serial.println(courseChangeNeeded);
    }
    else if(courseChangeNeeded >= 180){
      turnRight(5);
      Serial.print("Turn to the right: "); Serial.println(360 - courseChangeNeeded);
    }
    else if(courseChangeNeeded >= -180  && courseChangeNeeded < 0){
      turnRight(5);
      Serial.print("Turn to the right: "); Serial.println(-1 * courseChangeNeeded);
    }*/

//    Serial.println(curr_LAT);
//    Serial.println(curr_LNG);
//    
    //forward(1000);

    Serial.print("DISTANCE: "); Serial.print(distanceToDestination);
    Serial.println(" meters to go."); Serial.print("INSTRUCTION: ");

}



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
//newer robot movement stuff
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
}

//void turnLeft() {
//  LTmtr.writeMicroseconds(1300);  // left wheel clockwise
//  RTmtr.writeMicroseconds(1300);  // right wheel clockwise
//  delay(553);  // milliseconds to make a 90 degree left turn
//}

//void turnRight() {
//  LTmtr.writeMicroseconds(1700);  // left wheel counterclockwise
//  RTmtr.writeMicroseconds(1700);  // right wheel counterclockwise
//  delay(600);  // milliseconds to make a 90 degree right turn
//}

void turnLeft(int degree) { //turns my robot left
  //This equation came from calculating a linear relationship between the previous delay values for 45 and 90 degrees
  int turnTime = 6.14*degree;
  if(turnTime > 0){
    LTmtr.writeMicroseconds(1300);
    RTmtr.writeMicroseconds(1300);
    delay((int)turnTime);
  }
}

void turnRight(int degree) { //turns my robot 90 degrees right
  //This equation came from calculating a linear relationship between the previous delay values for 45 and 90 degrees
  int turnTime = 6.67*degree;
  if(turnTime > 0){
    LTmtr.writeMicroseconds(1700);
    RTmtr.writeMicroseconds(1700);
    delay((int)turnTime);
  }
}



/** Robot previous movement stuff

  void forward(int wait){ //moves robot forward
  LTmtr.writeMicroseconds(1600); //gives the wheels their inital postion
  RTmtr.writeMicroseconds(1350); //gives wheels inital their postion
  delay(wait);
  }

  void reverse(int wait) { //moves robot backwards
  LTmtr.writeMicroseconds(1350);
  RTmtr.writeMicroseconds(1600);
  delay(wait);
  }

  void turnRight(int degree) { //turns my robot 90 degrees right
  if(degree == 90){
    LTmtr.write(160);
    RTmtr.write(160);
    delay(950);
  }
  else if(degree == 45){
    LTmtr.write(160);
    RTmtr.write(160);
    delay(425);
  }
  else{
    LTmtr.write(160);
    RTmtr.write(160);
  }
  }
  void turnLeft(int degree) { //turns my robot left
  if(degree == 90){
    LTmtr.write(240);
    RTmtr.write(240);
    delay(770);
  }
  else if(degree == 45){
    LTmtr.write(240);
    RTmtr.write(240);
    delay(385);
  }else{
    LTmtr.write(240);
    RTmtr.write(240);
  }
  }

  void stopRobot() { //stops my robot
  LTmtr.writeMicroseconds(1500);
  RTmtr.writeMicroseconds(1500);
  }
**/
