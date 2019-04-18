#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303.h>

Adafruit_LSM303 lsm;

void setup() 
{
#ifndef ESP8266
  while (!Serial);     // will pause Zero, Leonardo, etc until serial console opens
#endif
  Serial.begin(9600);
  
  // Try to initialise and warn if we couldn't detect the chip
  if (!lsm.begin())
  {
    Serial.println("Oops ... unable to initialize the LSM303. Check your wiring!");
    while (1);
  }
}

void loop() 
{
  float Pi = 3.14159;
  lsm.read();
  Serial.print("Accel X: "); Serial.print((int)lsm.accelData.x); Serial.print(" ");
  Serial.print("Y: "); Serial.print((int)lsm.accelData.y);       Serial.print(" ");
  Serial.print("Z: "); Serial.println((int)lsm.accelData.z);     Serial.print(" ");
  Serial.print("Mag X: "); Serial.print((int)lsm.magData.x);     Serial.print(" ");
  Serial.print("Y: "); Serial.print((int)lsm.magData.y);         Serial.print(" ");
  Serial.print("Z: "); Serial.println((int)lsm.magData.z);       Serial.print(" ");
  delay(1000);

  float xbearing = lsm.magData.x*.48828125;
  float ybearing = lsm.magData.y*.48828125;
  float D = atan(ybearing/xbearing)*(180/3.14);
  
  float heading = (atan2(lsm.magData.y,lsm.magData.x) * 180) / Pi;
 if (heading <360){
  heading = 360 + heading;
  
 }
 
//  Serial.print("x"); Serial.print(xbearing);
//  Serial.println("y"); Serial.print( ybearing);
  Serial.print("D="); Serial.println(D);
  Serial.print("Heading: "); Serial.println(heading);
}
