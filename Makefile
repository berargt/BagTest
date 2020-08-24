upload:
	~/arduino-1.8.12/arduino --upload BagTest.ino
verify:
	~/arduino-1.8.12/arduino --verify BagTest.ino
test:
	stty -F /dev/ttyUSB0 115200
	cat /dev/ttyUSB0 > out.csv
#	stty -F /dev/serial/by-id/usb-FTDI_USB__-__Serial-if00-port0 115200
#	cat /dev/serial/by-id/usb-FTDI_USB__-__Serial-if00-port0 > out.csv
