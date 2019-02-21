///DO NOT USE THIS. NOT UP TO DATE.

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

void loop() {
  if(mySerial.available()){ //if data is available
    Serial.write(mySerial.read());  //print to Serial Monitor
  }

}
