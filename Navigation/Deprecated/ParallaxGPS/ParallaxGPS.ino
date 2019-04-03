#include <SoftwareSerial.h>

//define receiver/transmitter pins
#define rxPin 3
#define txPin 4

SoftwareSerial mySerial(rxPin, txPin); //declare SoftwareSerial object

/* Function to convert degree decimal minute format to decimal degree
 * String coord: string containing lat/lon numbers
 * bool negative: boolean of whether the value should be negative
 */
double toDecimalDegrees(String coord, bool negative = false){
  if(negative){
    return -(coord.toInt()/100) + (double)(coord.toDouble() - 100*(coord.toInt()/100))/60.0;
  }
  else{
    return (coord.toInt()/100) + (double)(coord.toDouble() - 100*(coord.toInt()/100))/60.0;
  }
}

bool valid = false; //bool for navigation receiver warning
String lat = ""; //Store latitude as string
String lon = ""; //Store longitude as string
double lat_dd = 0.0; //Store latitude as decimal degrees
double lon_dd = 0.0; //Store longitude as decimal degrees

/* Function to parse NMEA sentence to get nav receiver warning, lat, and lon
 * String sentence: the NMEA sentence to be used
 */
void NMEAparser(String sentence){
  int commas = 0; //counting commas in NMEA sentence

  //If 2nd letter in NMEA code is 'R'...
  if(sentence[3] == 'R'){
    for(int i = 5; commas <= 5; i++){ //only search sentence up to the 5th comma
      if(sentence[i] == ','){ //count the commas as they are found
        commas++;
      }
      else{
        //Store navigation receiver warning
        if(commas == 2){
          if(sentence[i] == 'A'){
            valid = true;
          }
        }
        //Store latitude
        if(commas == 3){
          lat += sentence[i];
        }
        if(commas == 4){
          if(sentence[i] == 'S'){ //If in Southern Hemisphere, set latitude to negative
            lat_dd = toDecimalDegrees(lat, true);  //Convert to decimal degrees
          }
          else{lat_dd = toDecimalDegrees(lat);}
        }
        //Store longitude
        if(commas == 5){
          lon += sentence[i];
        }
        if(commas == 6){
          if(sentence[i] == 'W'){ //If in Western Hemisphere, set longitude to negative
            lon_dd = toDecimalDegrees(lon, true);
          }
          else{lon_dd = toDecimalDegrees(lon);}
        }
      }
    }
  }
}

/* Function to calculate the angle RoboBuoy needs to turn to so that it is
 * directly facing its destination.
 * float angle: the current angle RoboBuoy is pointed at, read from the magnetometer
 * double currentLat, currentLon: the current GPS coordinates of RoboBuoy
 * double endLat, endLon: the GPS coordinates of the destination
 */
float getAngle(float angle, double currentLat, double currentLon, double endLat, double endLon){
    int deltaLat = currentLat - endLat;
    int deltaLon = currentLon - endLon;
    int hypotenuse = sqrt((deltaLat^2) - (deltaLon^2));
    float theta = sin(deltaLon/hypotenuse);

    if(deltaLat > 0){
      if(deltaLon < 0){ //If the difference between longitudes is negative...
        return angle + 180.0 - theta;
      }
      else if(deltaLon > 0){ //If the difference between longitudes is positive...
        return angle + 180.0 + theta;
      }
      else{ //If current position and destination are at the same longitude...
        return angle + 180.0;
      }
    }
    else if(deltaLat < 0){
      if(deltaLon < 0){ //If the difference between longitudes is negative...
        return angle - theta;
      }
      else if(deltaLon > 0){ //If the difference between longitudes is positive...
        return angle + theta;
      }
      else{ //If current position and destination are at the same longitude...
        return angle;
      }
    }
}



void setup() {
  pinMode(rxPin, INPUT); //set receiver pin to input
  Serial.begin(4800); //set serials to correct baud rate
  mySerial.begin(4800);
}



String sentence = ""; //Store entire NMEA sentence



void loop() {
//  if(mySerial.available()){ //if data is available
//    sentence = mySerial.read();  //print to Serial Monitor
//  }

  String sentence = "$GPRMC,002105,A,3640.5557,N,12147.0016,W,000.2,082.1,010219,,,A*63";

  NMEAparser(sentence);

  getAngle(90, 36.65694, -121.79561, 0.0, 0.0);
}
