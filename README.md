# soccer2020
This repository contains the various design files and programs for our robot this year. 
- - - -
They can be accessed via the different branches: 
* Layer1 controls the solenoid, light sensors, mouse sensor, temp sensor and current sensor (both for motor) 
* Layer3 controls the IMU (FXOS8700 + FXAS21002 from Adafruit) 
* Layer3Teensy is the main controller; it receives and processes data via UART from the other layers, bluetooth module, and the RasPi (for camera). It then controls the motors directly. 
* Layer4 controls the light ring, ESC for dribbler and TOF distance sensors 
* Mechanical contains all the 3D models for the robot
* PCB contains all the PCB design files for the robot
- - - -
Do follow us and contact us at [@bozotics](https://www.instagram.com/bozotics/) for any queries! Also do check out our **new** website at <https://bozo.infocommsociety.com/>
