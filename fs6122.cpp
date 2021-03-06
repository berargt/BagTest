#include "fs6122.h"
#include <Wire.h>
#include <Streaming.h>  // cout <iostream> functionality using Serial << endl;

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

void fs6122_readSmlpM_umH2O(pressure_flow_type *pf){
   long reading = 0;

   Wire.beginTransmission(FS6122_ADDRESS);
   Wire.write(FS6122_CMD_READ_FLOW_RATE_AND_PRESSURE);
   Wire.endTransmission();

   Wire.requestFrom(FS6122_ADDRESS, 9); 

   if (8 <= Wire.available()) { // if two bytes were received
      for(int byteIdx = 0; byteIdx < 4; byteIdx++){
         pf->mSLPM = pf->mSLPM << 8;    // shift high byte to be high 8 bits
         pf->mSLPM |= Wire.read(); // receive low byte as lower 8 bits
      }

      if (4 <= Wire.available()) {
         for(int byteIdx = 0; byteIdx < 4; byteIdx++){
            pf->umH2O = pf->umH2O << 8;    // shift high byte to be high 8 bits
            pf->umH2O |= Wire.read(); // receive low byte as lower 8 bits
         }
      }

   } else {
      //ERROR - sensor did not return data. TODO - return something meaningful
       pf->mSLPM = -42;
       pf->umH2O = -42;
   }
}


float fs6122_readSmlpM(){
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
