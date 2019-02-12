#include <SoftwareSerial.h>

//define receiver/transmitter pins
#define rxPin 3
#define txPin 4

SoftwareSerial mySerial(rxPin, txPin); //declare SoftwareSerial object

void setup() {
  pinMode(rxPin, INPUT); //set receiver pin to input
  Serial.begin(4800); //set serials to correct baud rate
  mySerial.begin(4800);
}

int commas = 0; //counting commas in NMEA sentence
bool valid = false; //bool for navigation receiver warning

String sentence = "";
String lat = "";
String lon = "";
//int lat_degrees = 0;
//float lat_minutes = 0;
//int lon_degrees = 0;
//float lon_degrees = 0;

double lat_dd = 0.0;
double lon_dd = 0.0;


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
        //Store nav receiver warning
        if(commas == 2){
          if(sentence[i] == 'A'){
            valid = true;  
          }
        }
        //Store latitude
        if(commas == 3){
          lat += sentence[i];
        }
        //Store longitude
        if(commas == 5){
          lon += sentence[i];
        }
      }  
    }  
  }
  
  //Convert degree and decimal minute to decimal degree
  lat_dd = (lat.toInt()/100) + (double)(lat.toDouble() - 100*(lat.toInt()/100))/60.0;
  lon_dd = (lon.toInt()/100) + (double)(lon.toDouble() - 100*(lon.toInt()/100))/60.0;
}
