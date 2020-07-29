# BagTest

Ventilators In Peoria (VIP)

Test the VIP fixture

This is written and tested on the Arduino Mega 2560

Serial Port Info:

stty -F /dev/ttyACM0 115200
cu -l /dev/ttyACM0 -s 115200

 ~/arduino-1.8.12/arduino --upload BagTest.ino


#To compile
make verify

#To compile and upload
make verify

#To compile and upload
make upload

#Start data collection
make test 

In another terminal 

tail -f out.csv


