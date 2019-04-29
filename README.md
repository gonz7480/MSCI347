# <RoboBuoy>
## Ocean Instrumentation Capstone Project
## CSUMB's Spring 2019 MSCI 437 Class

This repository holds all the software for **RoboBuoy**, an autonomous surface-swimming datalogger.

### Folder Hierarchy
Arduino Examples:
  - Arduino_GPS_Example: Example code for connecting a Parallax GPS to an Arduino.
                         Copied from Arduino Playground. https://playground.arduino.cc/Tutorials/GPS
  - Capstone_MotorTest_SP19: Testing code for the winch motor.
  - PS2Controlv0: Example code for using a PlayStation 2 controller to control servos
                  and an Electronic Speed Controller.
                  Copied from TechMonkeyBusiness. http://www.techmonkeybusiness.com/using-a-playstation-2-controller-with-your-arduino-project.html
  - Movement: <Was this an example or fully ours??>

Navigation:
  - Compass_test: Example code for reading an Adafruit LSM303 Magnetometer using an Arduino.
                  Copied from Adafruit_LSM303 Repo by microbuilder. https://github.com/adafruit/Adafruit_LSM303
  - Libraries: A folder containing all the libraries we either use or have tested in the process
               of developing our software. All libraries include their respective ReadMe files
               except for the ones listed below.
               NeoGPS-master: https://github.com/SlashDevin/NeoGPS
               TinyGPSPlus-master: http://arduiniana.org/libraries/tinygpsplus/
  - Movement: Prototype code using TinyGPS++ library for the autonomous navigation of RoboBuoy.
              **Not correctly working as of 4 Mar 2019 when tested on a BoeBot with an Arduino Uno.** https://www.parallax.com/product/boe-bot-robot
  - Movement_2.0: Second prototype code also using TinyGPS++ library, but uses precise heading changes.
                  **Not correctly working as of 4 Mar 2019 when tested on a BoeBot with an Arduino Uno.**
  - Neo_gps_code: Prototype code using NeoGPS library for the autonomous navigation of RoboBuoy.
                  **Not correctly working as of 4 Mar 2019 when tested on a BoeBot with an Arduino Uno.**
  - ParallaxGPS: Back-up software for navigation using a Parallax GPS unit. The final version of <RoboBuoy>
                 uses an Adafruit Ultimate GPS Breakout v3.
  - Propellers: Initial script for testing that our trusters worked.
                Used T100 trusters from Blue Robotics https://www.bluerobotics.com/store/thrusters/t100-t200-thrusters/t100-thruster/
                and an Afro ESC.
  
Safety:
  - Leak Detector: Code for testing wether or not there is water and what height the water is at
  - voltage_check_final: Code for checking the voltage of the LiPo batteries
  - ReedSwitch: code used to manually turn off the motors

Test_Options:
  - AutoNavigationwReed: This allows for a basic GPS navigation launch with the reed interrupt included to stop the motors at any point
  - AutoNavigationwWinch: This allows for GPS movement and dropping the Benthic Observatory
  - DropPackage: This code just drops the Benthic Observatory without the use of GPS or propellors
