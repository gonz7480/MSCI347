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

// This version is for the SHORE station.
// There's a corresponding version for the BOAT station.

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
char commandPacket[UDP_TX_PACKET_MAX_SIZE];

// Create an Ethernet instance to let us send and receive packets over UDP
EthernetUDP Udp;

// constants to control timing
const long int commandInterval = 100;  // interval between boat status reports in milliseconds
unsigned long previousMillis = 0;  // stores previous value of millis() reading; used for timing control

// pin assignments
  // pin assignments for joystick, etc. go here.

// status variables
int voltage = 0;  // used to measure and send battery voltage (raw A/D counts)
int leak = 0;  // used to warn human operator that water has been detected inside vehicle (if leak = 1);
int range = 65000;
int bearing = 360;

// command variables
char driveCommand = 'S';  // still need function to update this based on joystick commands
char winchCommand = 'S';  // still need function to update this based on joystick commands

void setup() {

  // open serial communication and wait for port to open
  Serial.begin(115200);
  while(!Serial);  // Wait for serial port to connect. Needed for native USB port only.

  // configure for the correct chip select (slave select) pin
  Ethernet.init(10);

  // start the Ethernet
  Ethernet.begin(macShore, ipShore);

  // Start UDP
  Udp.begin(localPortShore); 
}

void loop() {

    // snapshot the current time
  unsigned long currentMillis = millis();
  
  // if time to send command packet to boat, do so
  if ((unsigned long)(currentMillis - previousMillis) >= commandInterval) {

    // update previousMillis
    previousMillis = currentMillis;

    // clear commandBuffer to avoid leftovers in updated commands
    for(int i=0;i<UDP_TX_PACKET_MAX_SIZE;i++) {
      commandPacket[i] = 0;
    }

    // Get pilot commands from joystick
    // Note: this is just a placeholder; ultimately need to encode and send joystick commands    
    driveCommand = 'M';
    winchCommand = 'D';

    // assemble command packet
    commandPacket[0] = driveCommand; 
    commandPacket[1] = winchCommand; 

    // send command packet to boat
    Udp.beginPacket(ipBoat, localPortBoat);
    Udp.write(commandPacket);
    Udp.endPacket();
  }
  
  // If there's a status report available from the boat, read a the report
  int packetSize = Udp.parsePacket();
  if(packetSize != 0) { 
    Udp.read(commandPacket, UDP_TX_PACKET_MAX_SIZE);  // Read the packet into commandPacket
    Serial.print("Packet Size = ");
    Serial.println(packetSize);
    // decode the character array packet back into an integer
//    int volts = ((packetBuffer[0] * 256) + packetBuffer[1]);
//    Serial.println(volts, DEC);
//    Serial.println(packetBuffer);
    for(int i=0; i<24; i++) {
//      Serial.print(i);
//      Serial.print(":");
      if(commandPacket[i] != 0) {
        Serial.print(commandPacket[i]);
        Serial.print(" ");
      }
    }
  Serial.println();
  }
}
