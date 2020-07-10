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
#include <Streaming.h>  // cout <iostream> functionality using Serial << endl;
#include <avr/wdt.h>    // add the dog
#include "fs6122.h"
#include "wdog.h"

// defines

// comment this for normal operation
#define TIMESTAMP_DATA_OUTPUT

#define START_PWM         100
#define DWELLMS           100
#define PER_STEP_MS       10
#define OFF_MS            2200
#define RAMP_UP_PWM_INC   2
#define RAMP_DOWN_PWN_INC 4

#define OPEN_POS_ADD  250
#define CLOSE_POS_ADD 600

#define OPEN_SW       9
#define CLOSED_SW     8
#define PAUSE_SW      4
#define PWM_PIN       3
#define DIR_PIN       2

#define CLOSE         HIGH
#define OPEN          LOW

#define MAIN_LOOP_TS_MS 0

enum sequence {
   INIT_RAMP_UP_SEQ,
   RAMP_UP_SEQ,
   DWELL_SEQ,
   INIT_RAMP_DOWN_SEQ,
   RAMP_DOWN_SEQ,
   OFF_TIME_SEQ
};

// Globals
uint8_t dir;
long position;
long maxOpenPos;
long closePos;
int pwmSpeed = START_PWM;
uint8_t openSwState;
uint8_t closeSwState;
long cycleCount;
unsigned long lastTime;
unsigned long cycleTime;
long loopStartTime_us = 0;
double cpuLoad = 0;
long loopOverruns = 0;
unsigned long loopCounter = 0;
int count;

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
  // This delay is enough time to program if there are issues with the code/programming
  delay(3000);
  Serial3.begin(115200);
  while(!Serial3);    // time to get serial running
  Serial3 << "Starting BagTest Fixture!" << endl;

  //Init BME280 sensor
  bme.begin(BME280_ADDRESS_ALTERNATE);

  //Init Flowmeter
  fs6122_init();
  fs6122_zeroFlowCal();

  pinMode(PWM_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(OPEN_SW, INPUT_PULLUP);
  pinMode(CLOSED_SW, INPUT_PULLUP);

  Home();
  dir = CLOSE;
  lastTime = millis();
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

   pressure_flow_type fs6122;
   fs6122_readSmlpM_umH2O(&fs6122);
#ifdef TIMESTAMP_DATA_OUTPUT
   float temperature;
   temperature = bme.readTemperature();
#endif
   position = myEnc.read();

   // Communication with RaspberryPi
#ifdef TIMESTAMP_DATA_OUTPUT
   Serial3 << millis() << "," << fs6122.mSLPM << "," << fs6122.umH2O << ","
      << cpuLoad << "," << position << "," << temperature << "," 
      << dir << "," << pwmSpeed << "," << analogRead(A0) << "\r";
#else
   Serial3 << fs6122.flow_rate << "," << fs6122.pressure << "," << cpuLoad << endl;
#endif

   Sequence();

   digitalWrite(DIR_PIN, dir);
   analogWrite(PWM_PIN, pwmSpeed);

   //  displayLCD();
   wdog_reset();
   markLoopEnd();
}

/******************************************************************************/
/* Home                                                                       */
/*                                                                            */
/* Blocking because it doesn't matter when we home                            */
/* home the jaws by opening fully and calculate the min/max jaw position      */
/******************************************************************************/
void Home(void) {
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
  digitalWrite(DIR_PIN, dir);
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
  analogWrite(PWM_PIN, pwmSpeed);

  delay(100);

  maxOpenPos = myEnc.read() + OPEN_POS_ADD;
  closePos = maxOpenPos + CLOSE_POS_ADD;
  Serial3 << "********************************************************" << "\r";
  Serial3 << "maxOpenPos: " << maxOpenPos << "\r";
  Serial3 << "closePos: " << closePos << "\r";
  Serial3 << "********************************************************" << "\r";
}

/******************************************************************************/
/* Sequence                                                                   */
/*                                                                            */
/******************************************************************************/
void Sequence(void) {

  static char state = INIT_RAMP_UP_SEQ;
  static unsigned long next_inc_ms = 0;

  switch (state) {

    case INIT_RAMP_UP_SEQ:
      pwmSpeed = START_PWM;
      state = RAMP_UP_SEQ;
      next_inc_ms = millis() + PER_STEP_MS;
      dir = CLOSE;
      break;

    case RAMP_UP_SEQ:
      if (millis() > next_inc_ms)  {
        if (pwmSpeed >= 200) {
          next_inc_ms = millis() + DWELLMS;
          state = DWELL_SEQ;
        } 
        else {
          pwmSpeed += RAMP_UP_PWM_INC;
        }
        next_inc_ms = millis() + PER_STEP_MS;
      }
      dir = CLOSE;
      break;

    case DWELL_SEQ:
      if (millis() > next_inc_ms) state = INIT_RAMP_DOWN_SEQ;
      break;

    case INIT_RAMP_DOWN_SEQ:
      state = RAMP_DOWN_SEQ;
      next_inc_ms = millis() + PER_STEP_MS;
      pwmSpeed -= RAMP_DOWN_PWN_INC;
      dir = OPEN;
      break;

    case RAMP_DOWN_SEQ:
      if (millis() > next_inc_ms)  {
        if (pwmSpeed <= 100) {
          next_inc_ms = millis() + OFF_MS;
          pwmSpeed = 0;
          state = OFF_TIME_SEQ;
        } 
        else {
          pwmSpeed -= RAMP_DOWN_PWN_INC;
          next_inc_ms = millis() + PER_STEP_MS;
        }
      }
      dir = OPEN;
      break;

    case OFF_TIME_SEQ:
      if (millis() >= next_inc_ms) state = INIT_RAMP_UP_SEQ;
      break;
  }
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
  lcd.print(position);
  lcd.setCursor(0, 1);
  lcd.print("OPEN POS:");
  lcd.print(maxOpenPos);
  lcd.setCursor(0, 2);
  lcd.print("CLOSE POS:"); lcd.print(closePos);
  lcd.setCursor(0, 3);
//  lcd.print("POS:");
//  lcd.print(position); // DUPE TBD TODO
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
