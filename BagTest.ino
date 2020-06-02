/******************************************************************************/
/* BagTest.ino                                                                */
/* Author Gregory Berardi                                                     */
/* 04/22/2020                                                                 */
/* Code to test the Ambu Bag and Ventilator Fixture                           */
/******************************************************************************/
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Encoder.h>
#include <stdio.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Streaming.h>  // cout <iosstream> functionality using Serial << endl;
#include <avr/wdt.h>    // add the dog
#include "fs6122.h"
#include "wdog.h"

// defines
#define SPEED         200
#define OPEN_POS_ADD  100
#define CLOSE_POS_ADD 400
#define OPEN_DWELL    1250
#define TGT_CYC_MS    750
#define TGT_HST       10

#define OPEN_SW       9
#define CLOSED_SW     8
#define PAUSE_SW      4
#define PWM_PIN       3
#define DIR_PIN       2
#define CLOSE         HIGH
#define OPEN          LOW

#define MAIN_LOOP_TS_MS 50

// Globals
uint8_t dir;
//long minPosition, maxPosition;
long newPosition;
long lastPosition;
long maxOpenPos;
long closePos;
int pwmSpeed;
uint8_t openSwState;
uint8_t closeSwState;
long cycleCount;
unsigned long lastTime;
unsigned long cycleTime;
long loopStartTime_us = 0;
double cpuLoad = 0;
long loopOverruns = 0;
unsigned long loopCounter = 0;

// Classes
Adafruit_BME280 bme; 
Encoder myEnc(18, 19);  // use interrupt pins
LiquidCrystal_I2C lcd(0x27, 20, 4); // LCD address 0x27 20 chars 4 line display

/******************************************************************************/
/* Setup                                                                      */
/******************************************************************************/
void setup()
{
  lcd.init();
  lcd.backlight();
  lcd.home();

  lcd.print("Initializing...");
  lcd.setCursor(0, 1);
  pinMode(PAUSE_SW, INPUT_PULLUP);
  lcd.print("Ambu Bag Fixture!");
  delay(3000);
  Serial.begin(115200);
  while(!Serial);    // time to get serial running

  //Init BME280 sensor
  bme.begin(BME280_ADDRESS);

  //Init Flowmeter
  fs6122_init();
  fs6122_zeroFlowCal();

//  Serial << "Ambu Bag Fixture Starting!" << endl;
  pinMode(PWM_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(OPEN_SW, INPUT_PULLUP);
  pinMode(CLOSED_SW, INPUT_PULLUP);

  doHome();
  dir = CLOSE;
  lastTime = millis();
  pwmSpeed = SPEED;
  cycleCount = 0;
  
  digitalWrite(DIR_PIN, dir);
  analogWrite(PWM_PIN, pwmSpeed);
  wdog_start();
}

/******************************************************************************/
/* Main Loop                                                                  */
/******************************************************************************/
void loop()
{
  markLoopStart();

  float ch1Val = fs6122_readPressure_SmlpM();
  float ch2Val = bme.readTemperature();
  float ch3Val = cpuLoad;

  newPosition = myEnc.read();

  sendPiData(ch1Val, ch2Val, ch3Val);

  if (newPosition > closePos && dir == CLOSE) {
    dir = OPEN;
    doTransition();
  } else if (newPosition < maxOpenPos && dir == OPEN) {
    dir = CLOSE;
    cycleCount++;
    cycleTime = millis()-lastTime;
    // let's try to hit a target speed
    if (cycleTime > (TGT_CYC_MS + TGT_HST)) {
      closePos-=10;
    } else if (cycleTime < (TGT_CYC_MS - TGT_HST)) {
      closePos++;
    }
    doTransition();
//    Serial << cycleCount << "," << cycleTime << "," << closePos << endl;
    lastTime = millis();
  }

  if(newPosition == lastPosition) {
//    lcd.setCursor(7, 0);lcd.print("      ");
    lcd.setCursor(7, 0);lcd.print(newPosition);
  }

  lastPosition=newPosition;

  openSwState = digitalRead(OPEN_SW);
  closeSwState = digitalRead(CLOSED_SW);

  if ((openSwState==0 || closeSwState==0) && pwmSpeed != 0) {
    pwmSpeed = 0; // stop the fixture
//    Serial << (openSwState==0?"Failed Open":"Failed Closed") << endl;
  }
  
  digitalWrite(DIR_PIN, dir);
  analogWrite(PWM_PIN, pwmSpeed);
  wdog_reset();
  markLoopEnd();
}


/******************************************************************************/
/* doHome                                                                     */
/*                                                                            */
/* home the jaws by opening fully and calculate the min/max jaw position      */
/******************************************************************************/
void doHome(void) {
  uint8_t homeSpeed = 25; // go slow

  lcd.clear(); //display status of motor on LCD
  lcd.setCursor(0, 0);
  lcd.print("Homing...");

  analogWrite(PWM_PIN, homeSpeed);
  openSwState = digitalRead(OPEN_SW);

  while (openSwState == 0) {
    digitalWrite(DIR_PIN, CLOSE);
    openSwState = digitalRead(OPEN_SW);
    wdog_reset();
  }

  dir = OPEN;
  digitalWrite(DIR_PIN, OPEN);
  analogWrite(PWM_PIN, homeSpeed);

  
  do {
    openSwState = digitalRead(OPEN_SW);
    lcd.setCursor(0, 1); lcd.print("SW:");lcd.print(openSwState);
    maxOpenPos = myEnc.read();
    lcd.setCursor(0, 2); lcd.print("POS:"); lcd.print(maxOpenPos);
    wdog_reset();
  } while (openSwState == 1);

  lcd.setCursor(0, 3); lcd.print("Done!");
  
  // Start Closing to avoid switch closure detection
  digitalWrite(DIR_PIN, CLOSE);
  analogWrite(PWM_PIN, SPEED);

  delay(100);

  maxOpenPos = myEnc.read() + OPEN_POS_ADD;
  closePos = maxOpenPos + CLOSE_POS_ADD;
//  Serial << "maxOpenPos:" << maxOpenPos << endl << "closePos:" << closePos << endl;
//  Serial << "pwmSpeed:" << SPEED << endl;
}

/******************************************************************************/
/* doTransition                                                               */
/*                                                                            */
/* When transitioning from open to close and visa versa these are redundant   */
/******************************************************************************/
void doTransition(void) {
  digitalWrite(DIR_PIN, dir);
  analogWrite(PWM_PIN, 0);
  if (dir == CLOSE) {
    delay(OPEN_DWELL); // dwell for open but we are transitioning to closed
  }
  analogWrite(PWM_PIN, pwmSpeed);
  displayLCD();
}

/******************************************************************************/
/* displayLCD                                                                 */
/*                                                                            */
/* display data to the LCD                                                    */
/******************************************************************************/
void displayLCD(void) {
  lcd.clear(); //display status of motor on LCD
  lcd.setCursor(0, 0);
  lcd.print("CURPOS:");
  lcd.print(newPosition);
  lcd.setCursor(0, 1);
  lcd.print("OPEN POS:");
  lcd.print(maxOpenPos);
  lcd.setCursor(0, 2);
  lcd.print("POS:");
  lcd.print(newPosition);
  lcd.setCursor(0, 3);
  lcd.print("CLOSE POS:"); lcd.print(closePos);
//  lcd.print("MIN:"); lcd.print(minPosition);
//  lcd.print(" MAX:"); lcd.print(maxPosition);
}

//////////////////////////////////////////////////////
// Loop Timing
void markLoopStart(){
  loopStartTime_us = micros();
}

void markLoopEnd(){
  loopCounter++;
  long delayDur = (long)(MAIN_LOOP_TS_MS * 1000L) - (micros() - loopStartTime_us);
  if(delayDur > 0){
    if(delayDur > 65529){
      delay(delayDur/1000);
    } else {
       delayMicroseconds((unsigned int)delayDur);
    }
    cpuLoad = 100.0 * ( 1.0 - ((double)delayDur/1000.0)/(double)MAIN_LOOP_TS_MS);
  } else {
    //Error! Loop time overrun
    loopOverruns++;
    cpuLoad = 100.0;
  }
}

//////////////////////////////////////////////////////
// Communication with RaspberryPi

void sendPiData(float val1, float val2, float val3){
  char sendPacket[75];
  sprintf(sendPacket, "%d.%02d,%d.%02d,%d.%02d", //sprintf doesn't support floats, so do some tricks
  (int)val1, (int)(val1*100)%100,
  (int)val2, (int)(val2*100)%100,
  (int)val3, (int)(val3*100)%100);
  Serial.println(sendPacket);
}
