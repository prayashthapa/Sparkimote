# Sparkimote!
###### CSCI 3302: Intro to Robotics (Final Project)

![Sparkimote]
(sparki.jpg?raw=true "Sparkimote")

**Sparkimote** is a web-interface that communicates with Sparki. It can communicate with Sparki either via USB Serial or Bluetooth Serial. Follow the instructions below to get started with controlling Sparki through your browser!

- clone the repo and cd to it inside of Terminal
- run ```npm install``` to install dependencies
- upload sparki/sparki.ino to Sparki
- run ```node server/index.js``` (ensure that the serial ports match depending on comm. type):
	- /dev/cu.ArcBotics-DevB for Bluetooth (pair via Bluetooth Settings in System Preferences)
	- /dev/tty.usbmodem... for USB
- visit http://localhost:8080
- fire away!

###### Note: If you want to use USB Serial Communication instead, replace all instances of ```Serial1``` inside of sparki/sparki.ino with ```Serial```!