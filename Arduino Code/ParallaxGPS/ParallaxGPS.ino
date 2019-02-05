#include <SoftwareSerial.h>

//define receiver/transmitter pins
#define rxPin 2
#define txPin 3

SoftwareSerial mySerial(rxPin, txPin); //declare SoftwareSerial object

void setup() {
  pinMode(rxPin, INPUT); //set receiver pin to input
  Serial.begin(4800); //set serials to correct baud rate
  mySerial.begin(4800);
}

int commas = 0; //counting commas in NMEA sentence
bool valid = false; //bool for navigation receiver warning
int lat_ind[2] = {0,0}; //storing start/end indices for lat
int lon_ind[2] = {0,0}; //storing start/end indices for lon

void loop() {
//  if(mySerial.available()){ //if data is available
//    sentence = mySerial.read();  //print to Serial Monitor
//  }

  char sentence[] = "$GPRMC,002105,A,3640.5557,N,12147.0016,W,000.2,082.1,010219,,,A*63";

  //Parses the NMEA sentence for nav receiver warning, lat, and lon
  //If 2nd letter in NMEA code is 'R'...
  if(sentence[3] == 'R'){
    for(int i = 5; commas <= 5; i++){ //only search sentence up to the 5th comma
      if(sentence[i] == ','){ //count the commas as they are found
        commas++;  
      }
      else{
        //Save nav receiver warning
        if(commas == 2){
          if(sentence[i] == 'A'){
//            Serial.println(sentence[i]);
            valid = true;  
//            Serial.println(valid);
          }
        }
        //Save the starting index of latitude
        if(commas == 3){
//          Serial.println(i);
          if(lat[0] == 0){
            lat[0] = i; 
//            Serial.println(lat[0]); 
          }
        }
        //Save ending index of latitude and starting index of longitude
        if(commas == 4){
          if(lat[1] == 0){
            lat[1] = i-2;  
          }
          if(lon[0] == 0){
            lon[0] = i; 
          }
        }
        //Save ending index of longitude
        if(commas == 5){
          if(lon[1] == 0){
            lon[1] = i-2;  
          }  
        }
      }  
    }  
  }

  

  //Convert degree and decimal minute to decimal degree

}
