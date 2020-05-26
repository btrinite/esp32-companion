# Intro
Arduino code for ESP32 part of my DIY Robocars project (Not tested on others Arduino board).

ESP32 has the following responsabilities :
- Receive SBUS channels messages from radio rx, and forward them to the host through ROS Serial link
- Receive Steering and Throttling command from host and generate PWM accordingly to drive car's Servo and ESC.
- Display some status (LED+Display) about the whole car, including IP address 

# (unusual) Dependencies :
- SBUS forked by myself to fix esp32 isse : https://github.com/btrinite/SBUS-for-esp32
- Adafruit_SSD1306, pay attention to configure your display nodel in Adafruit_SSD1306.h
- ROS : Ros Arduino library has to be generated from my forked version of rosserial package https://github.com/btrinite/rosserial, please check ROS, rosserial and mode specifically http://wiki.ros.org/rosserial/Tutorials/Adding%20Other%20Messages

