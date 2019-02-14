//Voltage checker
//I am missing pins, and exact values for slope. We need
//to test that when given the chance
//I started to write the whole thing stopped, because 
//I didnt know why I even started that.
unsigned long time;
#include <ArduinoSort.h>
int opto = 18;
  int WiBat = 1;
  int WiBatSlope = 1;
  int WiBatInt = 1;
void setup() {
  //Serial.begin(9600);
  pinMode(opto, OUTPUT);

  

//Threshold 
  int TurnAround = 13.5;
  int Stop = 13.0;

}

float Charge(){
  //Discharges
  pinMode(1, INPUT);
  pinMode(opto, OUTPUT);
  pinMode(opto, LOW);
  delay(100);
}

float median(){
  int ChargeArray[5];
  ChargeArray[0] = Charge();
  ChargeArray[1] = Charge();
  ChargeArray[2] = Charge();
  ChargeArray[3] = Charge();
  ChargeArray[4] = Charge();
  sortArray(ChargeArray, 5);
  Serial.print(ChargeArray[2], "v");
  float voltage = ChargeArray[2] * WiBatSlope + WiBatInt;

  //return float;
}

void loop() {
//   int period = 60000;
//  int currentMillis = millis();
//  int startMillis = millis();
//  if (currentMillis - startMillis >= period){
//    Charge();
//    if (Charge <= Stop){
//    //Insert code about stopping thruster
//      if{(Charge <= TurnAround){
//      //insert code about stopping in general
//      }
//    }
//    
//  }
//}else{
}
