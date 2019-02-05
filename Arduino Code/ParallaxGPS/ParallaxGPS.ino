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

int commas = 0;
bool valid = false;
int lat[2] = {0,0};
int lon[2] = {0,0};

void loop() {
//  if(mySerial.available()){ //if data is available
//    sentence = mySerial.read();  //print to Serial Monitor
//  }

  char sentence[] = "$GPRMC,002105,A,3640.5557,N,12147.0016,W,000.2,082.1,010219,,,A*63";
  
  if(sentence[3] == 'R'){
    for(int i = 5; i < sizeof(sentence); i++){
      if(sentence[i] == ','){
        commas++;  
      }
      else{
        if(commas == 2){
          if(sentence[i] == 'A'){
//            Serial.println(sentence[i]);
            valid = true;  
//            Serial.println(valid);
          }
        }
        if(commas == 3){
//          Serial.println(i);
          if(lat[0] == 0){
            lat[0] = i; 
//            Serial.println(lat[0]); 
          }
        }
        if(commas == 4){
          if(lat[1] == '0'){
            lat[1] = (char)i;  
          }
//          lon += sentence[i];  
//          Serial.println(sentence[i]);
        }
      }  
    }  
  }

  

  //Convert degree and decimal minute to decimal degree

}
