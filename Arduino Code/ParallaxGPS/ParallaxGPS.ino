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

void setup() {
  pinMode(rxPin, INPUT); //set receiver pin to input
  Serial.begin(4800); //set serials to correct baud rate
  mySerial.begin(4800);
}

int commas = 0; //counting commas in NMEA sentence
bool valid = false; //bool for navigation receiver warning

String sentence = ""; //Store entire NMEA sentence

String lat = ""; //Store latitude as string
String lon = ""; //Store longitude as string

double lat_dd = 0.0; //Store latitude as decimal degrees
double lon_dd = 0.0; //Store longitude as decimal degrees


void loop() {
//  if(mySerial.available()){ //if data is available
//    sentence = mySerial.read();  //print to Serial Monitor
//  }

  String sentence = "$GPRMC,002105,A,3640.5557,N,12147.0016,W,000.2,082.1,010219,,,A*63";

  //Parses the NMEA sentence for nav receiver warning, lat, and lon
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
