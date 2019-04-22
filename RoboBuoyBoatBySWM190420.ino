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
                      // N = Navigating to destination under autopilot
                      // K = Keeping station over the dive site
char winchMode = 'T'; // Stores status of benthic obseratory winch. Codes below:
                      // T = Topside.Observatory is out of the water ,and winch is stopped.
                      // D = Descending. Einch is paying out line.
                      // B = On Bottom, and winch is stopped
                      // A = Ascending
                        
int voltage = 1024;  // raw A/D counts; initialize to invalid 1024 to detect problems
char voltageArray[5];  // stores the ascii characters representing voltage
int leakStatus = 'N';  // warns human operator of leak; 'N' = no leak, 'L' = leak
int range = 65000;  // in meters. Initialize to improbable value
int bearing = 361;  // direction to destination in degrees. Initialize to invalid value
int heading = 361;  // direction in which vessel is pointing. Initialize to invalid value

void setup() {

  // set I/O pin directions and modes
  pinMode(A1, INPUT);  // make sure battery voltage measuring pin is set to be an input.
  digitalWrite(A1, LOW); // make sure pullup resistor on the analog input pin is disabled.

  // open serial communication with computer and wait for port to open
  Serial.begin(115200);
  while(!Serial);  // Wait for serial port to connect. Needed for native USB port only.
 
  // configure for the correct chip select (slave select) pin
  Ethernet.init(10);

  // start the Ethernet
  Ethernet.begin(macBoat, ipBoat);

  // Start UDP
  Udp.begin(localPortBoat); 
}

void loop() {

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
    voltage = analogRead(battVoltagePin);
    voltage = analogRead(battVoltagePin);  // read again to give MUX time to settle.
    voltage = 1234;  // for debug only (temporarily override measured value with known constant)

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
    driveMode = commandBuffer[0];
    winchMode = commandBuffer[1];

    // act on commands. Could use case/switch here to streamline program a bit.
    if (driveMode == 'M') {
      // manual flight mode; follow joystick commands
      Serial.print("M");
    } else if (driveMode == 'N') {
      // navigate by autopilot to target waypoint
      Serial.println("N");    
    } else {
      driveMode == 'S'; // stop all motors if receive S command or don't receive any valid command.
      Serial.print("S");
    }      
  }  // closes: if commmand packet available
}  // closes loop()


    

