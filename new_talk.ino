#include <SPI.h>
#include <Ethernet.h>
#include <Servo.h>
#include <SoftwareSerial.h>

int depth = 1; //How deep the Benthic Observatory is lowered (measured in meters)

//Defined pins and global variables
//The reel switch is used to stop the winch
int reelState; //Variable to hold if the switch is closed or not
#define reelSwitch 3 //Pin connected to the reel switch

//Winch variables
#define motorController 7 //Winch controller
int delayTime; //Variable to hold the start delay in ms
int dropTime; //Variable to hold the droptime in ms
int winchWait; //Variable to hold the bottom time in ms
int dropLoops; //The number of times the winchDown loop cycles

Servo winchMtr;
int pos = 0;

#define RXfromMega 5
#define TXfromMega 6
SoftwareSerial megaSerial(RXfromMega, TXfromMega);


byte mac[] = { 0x90, 0xA2, 0xDA, 0x0D, 0xB2, 0xC9 };   //physical mac address
byte ip[] = { 192, 168, 1, 178 };                      // ip in lan (that's what you need to use in your browser. ("192.168.1.178")
byte gateway[] = { 192, 168, 1, 1 };                   // internet access via router
byte subnet[] = { 255, 255, 255, 0 };                  //subnet mask
EthernetServer server(80);                             //server port
String readString;

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }
  //microservo.attach(7);
  megaSerial.begin(9600);
  
  // start the Ethernet connection and the server:
  Ethernet.begin(mac, ip, gateway, subnet);
  server.begin();
  Serial.print("server is at ");
  Serial.println(Ethernet.localIP());
  digitalWrite(7, HIGH);
  while(1);
}


void loop() {
  winchMtr.writeMicroseconds(1700);
  delay(1000);
             reelState = digitalRead(reelSwitch);
            while (reelState == 1) { //If the switch is not connected, keep pulling up
              reelState = digitalRead(reelSwitch);
              winchMtr.writeMicroseconds(1200);
            }
            if (reelState == 0) {
              winchMtr.writeMicroseconds(1500);
              megaSerial.print('h');
            }
  // Create a client connection
  EthernetClient client = server.available();
  if (client) {
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();

        //read char by char HTTP request
        if (readString.length() < 100) {
          //store characters to string
          readString += c;
          //Serial.print(c);
        }

        //if HTTP request has ended
        if (c == '\n') {
          Serial.println(readString); //print to serial monitor for debuging

          client.println("HTTP/1.1 200 OK"); //send new page
          client.println("Content-Type: text/html");
          client.println();
          client.println("<HTML>");
          client.println("<HEAD>");
          client.println("<meta name='apple-mobile-web-app-capable' content='yes' />");
          client.println("<meta name='apple-mobile-web-app-status-bar-style' content='black-translucent' />");
          client.println("<link rel='stylesheet' type='text/css' href='https://randomnerdtutorials.com/ethernetcss.css' />");
          client.println("<TITLE>HELP Please</TITLE>");
          client.println("</HEAD>");
          client.println("<BODY>");
          client.println("<H1>RoboBuoy</H1>");
          client.println("<hr />");
          client.println("<br />");
          client.println("<H2>Manual Control</H2>");
          client.println("<br />");
          client.println("<a href=\"/?button1on\"\">Drop Package</a>");
          client.println("<a href=\"/?button1off\"\">NOTHING</a><br />");
          client.println("<br />");
          client.println("<br />");
          client.println("<a href=\"/?button2on\"\">Raise Package</a>");
          client.println("<a href=\"/?button2off\"\">Rotate Right</a><br />");
          client.println("<br />");
          client.println("</BODY>");
          client.println("</HTML>");

          delay(1);
          //stopping client
          client.stop();
          //controls the Arduino if you press the buttons
          if (readString.indexOf("?button1on") > 0) {
            dropTime = ((depth) * 7000); //The winch takes approx 7 seconds to unspool 1 meter of string, this calculates
            // how long the winch needs to unspool based on the given depth.
            dropLoops = ((dropTime) / 200); //The loop needs to cycle every 200 ms for the total amount of time
            //for (int i = 0; i < dropLoops; i++) { //Checks the reelSwitch every 200ms just in case the line becomes tangled while dropping
              //reelState = digitalRead(reelSwitch);
              //if (reelState == 0) { //If the switch is connected, stop
                //winchMtr.writeMicroseconds(1500);
              //} //else {
                //winchMtr.writeMicroseconds(1700);
             // }
              //delay(200);
           // }
           winchMtr.writeMicroseconds(1700);

          }
          if (readString.indexOf("?button1off") > 0) {
            //nothing
          }
          if (readString.indexOf("?button2on") > 0) {
            //insert the raise package function include the switch control

            reelState = digitalRead(reelSwitch);
            while (reelState == 1) { //If the switch is not connected, keep pulling up
              reelState = digitalRead(reelSwitch);
              winchMtr.writeMicroseconds(1200);
            }
            if (reelState == 0) {
              winchMtr.writeMicroseconds(1500);
              megaSerial.print('h');
            }

          }
          //          if (readString.indexOf("?button2off") > 0) {
          //            for (pos = 180; pos >= 1; pos -= 3) // goes from 180 degrees to 0 degrees
          //            {
          //              microservo.write(pos);              // tell servo to go to position in variable 'pos'
          //              delay(15);                       // waits 15ms for the servo to reach the position
          //            }
          //          }
          //clearing string for next read
          readString = "";

        }
      }
    }
  }
}
