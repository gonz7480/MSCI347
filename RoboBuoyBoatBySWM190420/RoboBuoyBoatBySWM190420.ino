// This is one of two paired Arduino Uno sketches developed by Professor Steve Moore to give students
// in his Spring 2019 MSCI 437 (Ocean Instrumentation Projects) class at CSUMB a working example of 2-way
// UDP communction between Ethernet shields on a LAN, so the students can use this approach to help them
// monitor and (if needed) control their ocean-going "RoboBuoy III" robot, which will have a SHORE and
// BOAT station linked by a WiFi client bridge.

// This sketch (and it's partner, RoboBuoyBoatBySWM190420) were tested and working on 21 April 2019 (Easter Sunday).
// Based on UDPSendReceiveString example code in Ardunio Ethernet library documentation accessed on 18 Apr 2019

// Background: RoboBuoy III is a semi-autonomous robot being designed/built by students in CSUMB MSCI 437.
// It consists of a GPS-equipped boat that can pilot itself to a specified waypoint, then lower a camera via
// a winch to the seafloor. Users on shore can monitor the craft and override the autopilot, if needed.
// This program is one of two developed by Steve Moore (instructor) to create a framework the class can use to
// monitor and control the vessel via a WiFi client bridge LAN.

// This version is for the BOAT station.
// There's a corresponding version for the SHORE station.

// include libraries to make it easier to use Ethernet and UDP routines.
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <TinyGPS++.h>  //GPS library
#include <Adafruit_Sensor.h>  //Sensor library
#include <Adafruit_LSM303_U.h>  //Compass library
#include <SoftwareSerial.h> //TX RX library
#include <Servo.h>  //Servo library

//destination lat and lon
float des_LAT = 36.60337222;
float des_LNG = -121.88472222;

// Set up network info for the Arduino Ethernet Shield in the SHORE station
byte macShore[] = { 0x90, 0xA2, 0xDA, 0x0D, 0xB2, 0xC8 };  // mac address of shore Ethernet shield
IPAddress ipShore(192, 168, 1, 177);  // static IP address for shore Ethernet shield
unsigned int localPortShore = 8124;  // used as UDP port for shore

// Set up network info for the Arduino Ethernet Shield in the BOAT station
byte macBoat[] = { 0x90, 0xA2, 0xDA, 0x0D, 0xB2, 0xC9 };  // sets mac address of boat Ethernet shield
IPAddress ipBoat(192, 168, 1, 178);  // sets static IP address for shore Ethernet shield
unsigned int localPortBoat = 8125;  // used as UDP port for boat

// packet buffers for sending and receiving data
char statusReport[UDP_TX_PACKET_MAX_SIZE];
char commandBuffer[UDP_TX_PACKET_MAX_SIZE];

// Create an Ethernet instance to let us send and receive packets over UDP
EthernetUDP Udp;

// constants to control timing
const long int statusInterval = 5000;  // interval between boat status reports in milliseconds
unsigned long previousMillis = 0;  // stores previous value of millis() reading; used for timing control

// pin assignments
const int battVoltagePin = A1;  // use this analog pin to measure (divided!) battery voltage
const int leakDetectPin = 7;  // use this digital I/O pin to monitor the leak detect circuit

// status variables
char driveMode = 'S'; // Stores the propulsion control mode code (see below); Initialize to 'S'.
                      // S = Stop. All thrusters set to zero speed.
                      // M = Manual control via joystick
                      // A = Navigating to destination under autopilot
char winchMode = 'H'; // Stores status of benthic obseratory winch. Codes below:
                      // D = Descending. Einch is paying out line.
                      // H = Stop
                      // U = Ascending

int voltageRaw = 1024;  // raw A/D counts; initialize to invalid 1024 to detect problems
char voltageArray[5];  // stores the ascii characters representing voltage
int leakStatus = 'N';  // warns human operator of leak; 'N' = no leak, 'L' = leak
int range = 65000;  // in meters. Initialize to improbable value
int bearing = 361;  // direction to destination in degrees. Initialize to invalid value
int heading = 361;  // direction in which vessel is pointing. Initialize to invalid value


//GPS connection pins
const byte RXGPS = 5;
const byte TXGPS = 4;

//Winch connection pins
const byte RXWinch = 6;
const byte TXWinch = 7;

//Reed switch pin (master shut-off)
const int REED_PIN = 2;

//Baud rates
#define SerialBaud 9600
//#define DebugBaud 115200

#define Pi 3.14159

//Declare servos
Servo RTmtr;
Servo LTmtr;

SoftwareSerial ss(RXGPS, TXGPS);  //The serial connection to the GPS device
SoftwareSerial winch(RXWinch, TXWinch); //Serial conncetion with winch Arduino

TinyGPSPlus gps;  //The TinyGPS++ object
unsigned long lastUpdateTime = 0;  //Set last updated time to zero

//Compass object
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);

//lat and lon of launching point
float home_LAT;
float home_LNG;

//current lat, lon, and heading
float curr_LAT;
float curr_LNG;

float distanceToDestination;
float courseToDestination;
int courseChangeNeeded;

//boolean used with getHome()
bool launch = true;

//constant for checking LiPo batter life
float LipoRatio = 4.144670051;

// Function to read the magnetometer and convert output to degrees
float compass() { //Get a new sensor event
  sensors_event_t event;
  mag.getEvent(&event);
  float heading = (atan2(event.magnetic.y, event.magnetic.x) * 180) / Pi;  // Calculate the angle of the vector y,x
  if (heading < 0) {  // Normalize to 0-360
    heading = 360 + heading;
  }
  return heading;
}

void left(int mod, int wait){
  RTmtr.writeMicroseconds(1500 - mod);
  LTmtr.writeMicroseconds(1500);
  delay(wait);
}

void right(int mod, int wait){
  RTmtr.writeMicroseconds(1500);
  LTmtr.writeMicroseconds(1500 + mod);
  delay(wait);
}

void forward(int mod, int wait){
  RTmtr.writeMicroseconds(1500 - mod);
  LTmtr.writeMicroseconds(1500 + mod);
  delay(wait);
}

void backward(int mod, int wait){
  RTmtr.writeMicroseconds(1500 + mod);
  LTmtr.writeMicroseconds(1500 - mod);
  delay(wait);
}

/* Function to determine which direction RoboBuoy needs to turn to correct its heading
 * Returns a character defining which direction to turn.
 * 'R' for right, 'L' for left, 'N' for none
 */
char courseChange(){
  if(courseChangeNeeded > -10 && courseChangeNeeded < 10){
    return 'N';
  }
  else if(courseChangeNeeded < -180){
    return 'L';
  }
  else if(courseChangeNeeded < 180 && courseChangeNeeded > 0){
    return 'L';
  }
  else if(courseChangeNeeded >= 180){
    return 'R';
  }
  else if(courseChangeNeeded >= -180  && courseChangeNeeded < 0){
    return 'R';
  }
  else{return 'N';}
}

// Function to stop the motors
void stop(){
   RTmtr.writeMicroseconds(1500);
   LTmtr.writeMicroseconds(1500);
   delay(1500);
}

/* Function to define how the thrusters should move/turn
 * int mod: an integer that modifies the speed of the thrusters.
 *          Must be greater than 25 if using BlueRobotics T100 Thrusters
 * char dir: a character indicating the direction to turn
 *           'L' for left, 'R' for right
 * int wait: an integer to set the delay when not turning
 */
void moveMotor(int mod, char dir, int wait = 1000){
  switch(dir){
    case 'L':
        left(mod, wait/2);
        break;
    case 'R':
        right(mod, wait/2);
        break;
    default:
        forward(mod, wait);
  }
}

//Function for autopilot navigation
//Changes the speed of the motors based on distance from destination
void autopilot(){
  //If less than 1 meter away from destination, stay put
  if (distanceToDestination <= 1) {
    //When initially true, send signal to winch to lower benthic observatory
    winch.println('x');

    moveMotor(25, courseChange());
  }//If less than 2 meters away, go slow
  else if(distanceToDestination <= 2){
    moveMotor(50, courseChange());
    moveMotor(50, 'N');
  }//If less than 10 meters away, go fairly fast
  else if(distanceToDestination <= 10){
    moveMotor(150, courseChange());
    moveMotor(150, 'N');
  }else{ //Else, go fast
    moveMotor(200, courseChange());
    moveMotor(200, 'N', 5000);
  }
}

//Function for manual navigation
void manual(){
  if(driveMode == 'R'){
    right(50, 300);
  }
  if(driveMode == 'L'){
    left(50, 300);
  }
  if(driveMode == 'F'){
    forward(50);
  }
  if(driveMode == 'B'){
    backward(50);
  }
  if(driveMode == 'S'){
    stop();
  }
}

//Function to save the GPS coordinates of where RoboBuoy is launched
void getHome(){
  if(launch){
    home_LAT = gps.location.lat();
    home_LNG = gps.location.lng();
  }
  launch = false;
}

//Function to set destination to launching point for retrieval
void goHome(){
  des_LAT = home_LAT;
  des_LNG = home_LNG;
}

void setup() {

  // set I/O pin directions and modes
  pinMode(A1, INPUT);  // make sure battery voltage measuring pin is set to be an input.
  digitalWrite(A1, LOW); // make sure pullup resistor on the analog input pin is disabled.

  pinMode(REED_PIN, INPUT_PULLUP);

  // open serial communication with computer and wait for port to open
  Serial.begin(115200);
  while(!Serial);  // Wait for serial port to connect. Needed for native USB port only.

  // configure for the correct chip select (slave select) pin
  Ethernet.init(10);

  // start the Ethernet
  Ethernet.begin(macBoat, ipBoat);

  // Start UDP
  Udp.begin(localPortBoat);

  //Serial.begin(DebugBaud);  //Begin connection with the serial monitor
  ss.begin(SerialBaud);  //Begin software serial connection with GPS
  RTmtr.attach(5);  //Attach the servos to the pins
  LTmtr.attach(3);

  // Initialise the Compass
  if (!mag.begin()) { //Compass failed to initialize, check the connections
    //Serial.println("Oops, no Compass detected. Check your wiring!");
    while (1);
  }

  //T100 trusters need a stop command to initialize
  RTmtr.writeMicroseconds(1500);
  LTmtr.writeMicroseconds(1500);
  delay(2000);
}

void loop() {

  if(digitalRead(REED_PIN) == LOW){
    stop();
  }

  // snapshot the current time
  unsigned long currentMillis = millis();

  // if time to send status report, do so
  if ((unsigned long)(currentMillis - previousMillis) >= statusInterval) {

    // update previousMillis
    previousMillis = currentMillis;

    // debug
    Serial.println();
    Serial.println("Time to send status report.");

    // Read battery voltage
    voltageRaw = analogRead(battVoltagePin);
    voltageRaw = analogRead(battVoltagePin);  // read again to give MUX time to settle.
    //voltage = 1234;  // for debug only (temporarily override measured value with known constant)
    float voltage = (voltageRaw*.00488759);
    float LipoVoltage = (voltage * LipoRatio);

    if (LipoVoltage < 14.3){
      //send signal to shore
      stop();
    }

    //Check if signal from Winch has been received
    //If yes, goHome();
    winch.listen();
    if(winch.available()){
      goHome();
    }

    //If any characters have arrived from the GPS,
    //send them to the TinyGPS++ object
    while (ss.available() > 0) {
      if (gps.encode(ss.read()));
    }

    //Save initial GPS location
    if(gps.location.isValid() && gps.satellites.value() > 3 && launch == true){
      getHome();
    }else if(launch == true){
      break;
    }

    //Save current lat, lon, and heading
    curr_LAT = gps.location.lat();
    curr_LNG = gps.location.lng();
    heading = compass();

    //Establish our current status
    //These two lines are from the TinyGPS++ example
    distanceToDestination = TinyGPSPlus::distanceBetween(curr_LAT, curr_LNG, des_LAT, des_LNG);
    courseToDestination = TinyGPSPlus::courseTo(curr_LAT, curr_LNG, des_LAT, des_LNG);

    //calculate the difference in angle between current heading
    //and a heading that would lead RoboBuoy straight to the destination
    courseChangeNeeded = heading - courseToDestination;

    // getLeakStatus();
    leakStatus = 'L';  // Placeholder for sensor data: N = no leak, L = leak!
    // getPosition();
    // getRange();
    // getBearing();

    // clear statusReport buffer to avoid leftovers in updated status report
    for(int i=0;i<UDP_TX_PACKET_MAX_SIZE;i++) {
      statusReport[i] = 0;
    }

    // assemble status report package
    statusReport[0] = driveMode;
    statusReport[1] = winchMode;
    statusReport[2] = leakStatus;
    String voltageString = String(voltage, DEC);
    voltageString.toCharArray(voltageArray, 5);
    for (int v = 0; v<4; v++) {
      statusReport[v+3] = voltageArray[v];  // array location offset by 3 to leave room for driveMode, etc.
    }

    // send status report package to shore
    Udp.beginPacket(ipShore, localPortShore);
    Udp.write(statusReport);
    Udp.endPacket();

  }  // closes: if time for status report

  // If there's a command packet available, read it and respond accordingly
  int packetSize = Udp.parsePacket();
  if(packetSize != 0) {
    Udp.read(commandBuffer, UDP_TX_PACKET_MAX_SIZE);  // Read the packet into commandBuffer

    // parse command buffer
    mode = commandBuffer[0];
    driveMode = commandBuffer[1];
    winchMode = commandBuffer[2];

    // act on commands. Could use case/switch here to streamline program a bit.
    if (mode == 'M') {
      manual();
      // manual flight mode; follow joystick commands
      //Serial.print("M");
    } else if (mode == 'A') {
      autopilot();
      // navigate by autopilot to target waypoint
      //Serial.println("A");
    } else {
      mode == 'S'; // stop all motors if receive S command or don't receive any valid command.
      stop();
      //Serial.print("S");
    }


  }  // closes: if commmand packet available
}  // closes loop()
