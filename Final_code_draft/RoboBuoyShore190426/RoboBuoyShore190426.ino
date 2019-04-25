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
char modeCommand = 'S';  // still need function to update this based on joystick commands
char driveCommand = 'S';  // still need function to update this based on joystick commands
char winchCommand = 'S';  // still need function to update this based on joystick commands
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Includes de libraries for the joystick control and defining the joystick parameters.
#include <PS2X_lib.h>  //for v1.6

/******************************************************************
 * set pins connected to PS2 controller:
 *   - 1st column: original 
 * replace pin numbers by the ones you use
 ******************************************************************/
#define PS2_DAT        5  // BROWN   
#define PS2_CMD        6  // ORANGE
#define PS2_SEL        7  // YELLOW
#define PS2_CLK        4  // BLUE
//#define pressures   true
#define pressures   false
//#define rumble      true
#define rumble      false
PS2X ps2x; // create PS2 Controller Class

int error = 0;
byte type = 0;
byte vibrate = 0;
int mode=0;
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Joystick  setup
  //setup pins and settings: GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error
  error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
  Serial.println(error);
  if(error == 0){
    Serial.print("Found Controller, configured successful ");
    Serial.print("pressures = ");
  }
  else{ if(error == 1)
    Serial.println("No controller found, check wiring, see readme.txt to enable debug. visit www.billporter.info for troubleshooting tips");
   
  else if(error == 2)
    Serial.println("Controller found but not accepting commands. see readme.txt to enable debug. Visit www.billporter.info for troubleshooting tips");

  else if(error == 3)
    Serial.println("Controller refusing to enter Pressures mode, may not support it. ");
  }
//  Serial.print(ps2x.Analog(1), HEX);
  
  type = ps2x.readType(); 
  switch(type) {
    case 0:
      Serial.print("Unknown Controller type found ");
      break;
    case 1:
      Serial.print("DualShock Controller found ");
      break;
   case 2:
      Serial.print("Wireless Sony DualShock Controller found ");
      break;
   }
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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
////////////////////////////////////////////////////////////////////////////////////////
  //  if(error == 1){ //skip loop if no controller found
  //  return; 
  //  }else{
      ps2x.read_gamepad(false, vibrate); //read controller and set large motor to spin at 'vibrate' speed
    
      if(ps2x.ButtonPressed(PSB_START) || Serial.read == 'A'){       //will be TRUE as long as button is pressed
         Serial.println("Start just presed");
         mode=1;     //Autonomous mode
      }else if(ps2x.ButtonPressed(PSB_SELECT) || Serial.read == 'M'){
          Serial.println("Select just pressed");  
          mode=0;     //Manual mode
      }else if(ps2x.ButtonPressed(PSB_L2) || Serial.read == 'S'){
          Serial.println("Emergency Stop");  
          modeCommand = 'S';     //Manual mode
      }
      if(mode==1){    //Turn it to autonomous mode
          modeCommand = 'A';
      }else if(mode==0){    //Turn it to manual mode
        modeCommand = 'M';
        
        if(ps2x.ButtonPressed(PSB_PAD_UP) || Serial.read() == 'F') {      //will be TRUE as long as button is pressed
          Serial.print("Up pressed");
          driveCommand='F';                                //Go Forward
        }else if(ps2x.ButtonPressed(PSB_PAD_RIGHT) || Serial.read() == 'R'){
          Serial.print("Right pressed");
          driveCommand='R';                                //Go to the Right
       }else if(ps2x.ButtonPressed(PSB_PAD_LEFT) || Serial.read() == 'L'){
          Serial.print("LEFT pressed");
          driveCommand='L';                                //Go to the Left
       }else if(ps2x.ButtonPressed(PSB_PAD_DOWN) || Serial.read() == 'B'){
          Serial.print("DOWN pressed");
          driveCommand='B';                                //Go Back
       }else if(ps2x.ButtonPressed(PSB_CIRCLE) || Serial.read() == 'D'){               //will be TRUE if button was JUST pressed
          Serial.println("Circle pressed");
          winchCommand = 'D';                            //Winch Down
       }else if(ps2x.ButtonPressed(PSB_TRIANGLE) || Serial.read() == 'U'){
          Serial.println("Triangle pressed");
          winchCommand = 'U';                             //Winch UP
       }
       if(ps2x.ButtonPressed(PSB_L2) || Serial.read() == 'S'){
          Serial.println("L2 pressed");
          driveCommand = 'S';                              //Stop the thrusters
       }
       if(ps2x.ButtonPressed(PSB_R2) || Serial.read() == 'H'){
          Serial.println("R2 pressed");
          winchCommand = 'H';                              //Stop the Winch
       }
     }
    

           




////////////////////////////////////////////////////////////////////////////////////////
    // Note: this is just a placeholder; ultimately need to encode and send joystick commands    
    //driveCommand = 'M';
    //winchCommand = 'D';

    // assemble command packet
    commandPacket[0] = modeCommand; 
    commandPacket[1] = driveCommand; 
    commandPacket[2] = winchCommand; 

    // send command packet to boat
    Udp.beginPacket(ipBoat, localPortBoat);
    Udp.write(commandPacket);
    Serial.print(commandPacket);
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
}
