#include "fs6122.h"
#include <Wire.h>

void fs6122_init(){
  Wire.begin();

  //Configure filter depth
  Wire.beginTransmission(FS6122_ADDRESS);
  Wire.write(FS6122_CMD_WRITE_FILTER_DEPTH);
  Wire.write(byte(FS6122_FILTER_DEPTH));
  Wire.endTransmission();
}

void fs6122_zeroFlowCal(){
  // Must be done when there is guarnteed zero flow in the pipe
  // This function blocks for up to a second, and should not be done in periodic tasking
  Wire.beginTransmission(FS6122_ADDRESS);
  Wire.write(FS6122_CMD_CAL_FLOW_RATE);
  Wire.write(byte(0xFF)); //arbitrary byte
  Wire.endTransmission();
  delay(300);

  Wire.beginTransmission(FS6122_ADDRESS);
  Wire.write(FS6122_CMD_CAL_PRESSURE);
  Wire.write(byte(0xFF)); //arbitrary byte
  Wire.endTransmission();
  delay(300);
}

float fs6122_readPressure_SmlpM(){
  long reading = 0;

  Wire.beginTransmission(FS6122_ADDRESS);
  Wire.write(FS6122_CMD_READ_FLOW_RATE);
  Wire.endTransmission();

  Wire.requestFrom(FS6122_ADDRESS, 4); 
  
  if (4 <= Wire.available()) { // if two bytes were received
    for(int byteIdx = 0; byteIdx < 4; byteIdx++){
      reading = reading << 8;    // shift high byte to be high 8 bits
      reading |= Wire.read(); // receive low byte as lower 8 bits
    }
    return (float)reading;
  } else {
    //ERROR - sensor did not return data. TODO - return something meaningful
    return -42;
  }
}
