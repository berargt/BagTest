upload:
	~/arduino-1.8.12/arduino --upload BagTest.ino
verify:
	~/arduino-1.8.12/arduino --verify BagTest.ino
test:
	stty -F /dev/ttyUSB0 115200
	cat /dev/ttyUSB0 > out.csv
