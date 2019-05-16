#include <SoftwareSerial.h>
#include <Servo.h> 
#include <TinyGPS++.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
 

Servo RTmtr;
Servo LTmtr;

#define RXPin 10
#define TXPin 8
#define GPSBaud 9600
#define ConsoleBaud 115200

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

// The TinyGPS++ object
TinyGPSPlus gps;
unsigned long lastUpdateTime = 0;

// Assign a unique ID to this sensor at the same time 
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);

#define des_LAT 36.652874
#define des_LNG -121.794330

/* This example shows a basic framework for how you might
   use course and distance to guide a person (or a drone)
   to a destination.  This destination is the Eiffel Tower.
   Change it as required.

   The easiest way to get the lat/long coordinate is to
   right-click the destination in Google Maps (maps.google.com),
   and choose "What's here?".  This puts the exact values in the
   search box.
*/

void setup()
{
  Serial.begin(ConsoleBaud);
  ss.begin(GPSBaud);
  RTmtr.attach(7);
  LTmtr.attach(6);

  // Initialise the sensor 
  if(!mag.begin())
  {
    // There was a problem detecting the LSM303 ... check your connections 
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
}

float compass(){
  // Get a new sensor event 
  sensors_event_t event; 
  mag.getEvent(&event);
  
  float Pi = 3.14159;
  
  // Calculate the angle of the vector y,x
  float heading = (atan2(event.magnetic.y,event.magnetic.x) * 180) / Pi;
  
  // Normalize to 0-360
  if (heading < 0)
  {
    heading = 360 + heading;
  }
//  Serial.print("Compass Heading: ");
//  Serial.println(heading);
//  delay(500);

  return heading;
}

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


void loop()
{
  // If any characters have arrived from the GPS,
  // send them to the TinyGPS++ object
  while (ss.available() > 0)
    gps.encode(ss.read());

  // Every 5 seconds, do an update.
  if (millis() - lastUpdateTime >= 5000)
  {
    lastUpdateTime = millis();
    Serial.println();

    // Establish our current status
    double distanceToDestination = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), des_LAT, des_LNG);
    double courseToDestination = TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), des_LAT, des_LNG);
    const char *directionToDestination = TinyGPSPlus::cardinal(courseToDestination);
    int courseChangeNeeded = (int)(360 + courseToDestination - gps.course.deg()) % 360;

    // debug
    Serial.print("DEBUG: Course2Dest: ");
    Serial.print(courseToDestination);
    Serial.print("  CurCourse: ");
    Serial.print(gps.course.deg());
    Serial.print("  Dir2Dest: ");
    Serial.print(directionToDestination);
    Serial.print("  RelCourse: ");
    Serial.print(courseChangeNeeded);
    Serial.print("  CurSpd: ");
    Serial.println(gps.speed.kmph());

    float l = gps.location.lat();
    Serial.println();
    Serial.print("Lat: "); Serial.print(l,6); Serial.print("  Lon: "); Serial.println(gps.location.lng());
    Serial.print("current Angle: "); Serial.println(compass());

    // Within 20 meters of destination?  We're here!
    if (distanceToDestination <= 2)
    {
      Serial.println("CONGRATULATIONS: You've arrived!");
      exit(1);
    }

    Serial.print("DISTANCE: ");
    Serial.print(distanceToDestination);
    Serial.println(" meters to go.");
    Serial.print("INSTRUCTION: ");

    // Standing still? Just indicate which direction to go.
    if (gps.speed.kmph() < 0.5)
    {
      Serial.print("Head ");
      Serial.print(directionToDestination);
      Serial.println(".");
      //return;
    }

    if (courseChangeNeeded >= 345 || courseChangeNeeded < 15){
      Serial.println("Keep on straight ahead!");
      forward(10000);
    }
    else if (courseChangeNeeded >= 315 && courseChangeNeeded < 345){
      Serial.println("Veer slightly to the left.");
      turnLeft(45);
      forward(6000);
    }
    else if (courseChangeNeeded >= 15 && courseChangeNeeded < 45){
      Serial.println("Veer slightly to the right.");
      turnRight(45);
      forward(6000);
    }
    else if (courseChangeNeeded >= 255 && courseChangeNeeded < 315){
      Serial.println("Turn to the left.");
      turnLeft(90);
      forward(6000);
    }
    else if (courseChangeNeeded >= 45 && courseChangeNeeded < 105){
      Serial.println("Turn to the right.");
      turnRight(90);
      forward(6000);
    }
    else{
      Serial.println("Turn completely around.");
      reverse(1000);
    }
  }
}
