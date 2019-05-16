#include <Servo.h>

#define ConsoleBaud 9600

Servo RTmtr;
Servo LTmtr;

void turnLeft(int degree) { //turns my robot left
  //This equation came from calculating a linear relationship between the previous delay values for 45 and 90 degrees
  int turnTime = 6.14*degree;
  if(turnTime > 0){
    LTmtr.writeMicroseconds(1300);
    RTmtr.writeMicroseconds(1300);
    delay(turnTime);
  }
}

void turnRight(int degree) { //turns my robot 90 degrees right
  //This equation came from calculating a linear relationship between the previous delay values for 45 and 90 degrees
  int turnTime = 6.67*degree; 
  if(turnTime > 0){
    LTmtr.writeMicroseconds(1700);
    RTmtr.writeMicroseconds(1700);
    delay(turnTime);
  }
}

void forward(int duration) {  //function to go straight full speed for designated time
  LTmtr.writeMicroseconds(1700);  // left wheel counterclockwise
  RTmtr.writeMicroseconds(1300);  // right wheel clockwise
  delay(duration);
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(ConsoleBaud);
  RTmtr.attach(6);  //Attach the servos to the pins
  LTmtr.attach(7);
}

void loop() {
  // put your main code here, to run repeatedly:
  int courseChangeNeeded = -200;

  if(courseChangeNeeded > -10 && courseChangeNeeded < 10){ //turned right
    Serial.print("Turn forward. "); Serial.println(courseChangeNeeded);
    //forward(1000); 
  }
  else if(courseChangeNeeded < -180){ //turns correctly, turns right between turns
    turnLeft(5);
    Serial.print("Turn to the left: "); Serial.println(360 - (-1*courseChangeNeeded));
  }
  else if(courseChangeNeeded < 180 && courseChangeNeeded > 0){ //turns correctly, turns right between turns
    turnLeft(5);
    Serial.print("Turn to the left: "); Serial.println(courseChangeNeeded);
  }
  else if(courseChangeNeeded >= 180){ //turns correctly, turns right between turns
    turnRight(5);
    Serial.print("Turn to the right: "); Serial.println(360 - courseChangeNeeded);
  }
  else if(courseChangeNeeded >= -180  && courseChangeNeeded < 0){
    turnRight(5);
    Serial.print("Turn to the right: "); Serial.println(-1 * courseChangeNeeded);
  } 
  
}
