#include <Servo.h>
#include <NeoSWSerial.h>
#include <NMEAGPS.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>

Servo RTmtr;
Servo LTmtr;

#define GPSBaud 9600
#define ConsoleBaud 115200

// The serial connection to the GPS device
//AltSoftSerial gpsPort; // pins 8 & 9 would be better!
#define RXPin 9
#define TXPin 8
NeoSWSerial gpsPort(RXPin, TXPin);

NMEAGPS gps;
uint8_t updates = 0; // used to count elapsed seconds

NeoGPS::Location_t des( 36.652874, -121.794330 ); // Orono, ME


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


void setup()
{
  Serial.begin(ConsoleBaud);
  gpsPort.begin(GPSBaud);

  RTmtr.attach(7);
  LTmtr.attach(6);
}

void loop()
{
  // Process characters from the GPS

  while (gps.available( gpsPort )) {
    gps_fix fix = gps.read();

    // Every 5 seconds, do an update.
    if (++updates >= 3) {
      updates = 0;
      Serial.println();

      // If we have a location, give instructions
      if (fix.valid.location) {

        // Establish our current status
        double distanceToDestination = fix.location.DistanceKm( des );
        double courseToDestination   = fix.location.BearingToDegrees( des );
        const __FlashStringHelper *directionToDestination =
                                        compassDir(courseToDestination);
        int courseChangeNeeded = (int)(360 + courseToDestination - compass()) % 360;

        // debug
//        Serial.print( F("DEBUG: Course2Dest: ") );
//        Serial.print(courseToDestination);
//        Serial.print( F("  CurCourse: ") );
//        if (fix.valid.heading)
//          Serial.print( fix.heading() );
//        Serial.print( F("  Dir2Dest: ") );
//        Serial.print(directionToDestination);
//        Serial.print( F("  RelCourse: ") );
//        Serial.print(courseChangeNeeded);
//        Serial.print( F("Lat: ") );
//        Serial.print( fix.latitude(), 6 );
//        Serial.print( F("  Lon: ") );
//        Serial.println( fix.longitude(), 14 );
//
//        Serial.print( F("  CurSpd: ") );
//        if (fix.valid.speed)
//          Serial.print( fix.speed_kph() );
//        Serial.println('\n');

        // ???  THIS ISN'T A THING
        //Serial.print( F("current Angle: ") );
        //Serial.println(atan2(gps.location.lat(), gps.location.lng())*180/M_PI);

        // Within 20 meters of destination?  We're here!
        if (distanceToDestination <= 3.0)
        {
          Serial.println( F("CONGRATULATIONS: You've arrived!") );
          exit(1);
        }

        Serial.print( F("DISTANCE: ") );
        Serial.print(distanceToDestination);
        Serial.println( F(" meters to go.") );
        Serial.print( F("INSTRUCTION: ") );

        // Standing still? Just indicate which direction to go.
        if (fix.speed_kph() < .5)
        {
          Serial.print( F("Head ") );
          Serial.print(directionToDestination);
          Serial.println( '.' );

        } else // suggest a course change
        if ((courseChangeNeeded >= 345) || (courseChangeNeeded < 15)){
          Serial.println( F("Keep on straight ahead!") );
          forward(6000);
        }
        else if ((courseChangeNeeded >= 315) && (courseChangeNeeded < 345)){
          Serial.println( F("Veer slightly to the left.") );
          turnLeft(45);
        }
        else if ((courseChangeNeeded >= 15) && (courseChangeNeeded < 45)){
          Serial.println( F("Veer slightly to the right.") );
          turnRight(45);
        }
        else if ((courseChangeNeeded >= 255) && (courseChangeNeeded < 315)){
          Serial.println( F("Turn to the left.") );
          turnLeft(90);
        }
        else if ((courseChangeNeeded >= 45) && (courseChangeNeeded < 105)){
          Serial.println( F("Turn to the right.") );
          turnRight(90);
        }
        else{
          Serial.println( F("Turn completely around.") );
          turnRight(90);
          turnRight(90);
        }
      } else {
        Serial.println( F("Waiting for GPS fix...") );
      }

      forward(6000);
    }
  }
}

//------------------------------------------------------------
//  This snippet is from NMEAaverage.  It keeps all the
//    compass direction strings in FLASH memory, saving RAM.

const char nCD  [] PROGMEM = "N";
const char nneCD[] PROGMEM = "NNE";
const char neCD [] PROGMEM = "NE";
const char eneCD[] PROGMEM = "ENE";
const char eCD  [] PROGMEM = "E";
const char eseCD[] PROGMEM = "ESE";
const char seCD [] PROGMEM = "SE";
const char sseCD[] PROGMEM = "SSE";
const char sCD  [] PROGMEM = "S";
const char sswCD[] PROGMEM = "SSW";
const char swCD [] PROGMEM = "SW";
const char wswCD[] PROGMEM = "WSW";
const char wCD  [] PROGMEM = "W";
const char wnwCD[] PROGMEM = "WNW";
const char nwCD [] PROGMEM = "NW";
const char nnwCD[] PROGMEM = "NNW";

const char * const dirStrings[] PROGMEM =
  { nCD, nneCD, neCD, eneCD, eCD, eseCD, seCD, sseCD,
    sCD, sswCD, swCD, wswCD, wCD, wnwCD, nwCD, nnwCD };

const __FlashStringHelper *compassDir( uint16_t bearing ) // degrees CW from N
{
  const int16_t directions    = sizeof(dirStrings)/sizeof(dirStrings[0]);
  const int16_t degreesPerDir = 360 / directions;
        int8_t  dir           = (bearing + degreesPerDir/2) / degreesPerDir;

  while (dir < 0)
    dir += directions;
  while (dir >= directions)
    dir -= directions;

  return (const __FlashStringHelper *) pgm_read_ptr( &dirStrings[ dir ] );

} // compassDir
