/******************************************************************************/
/* BagTest.ino                                                                */
/* Author Gregory Berardi                                                     */
/* 04/22/2020                                                                 */
/* Code to test the Ambu Bag and Ventilator Fixture                           */
/******************************************************************************/
#include <Wire.h>
#include <Encoder.h>
#include <stdio.h>
#include <Adafruit_Sensor.h>  // Adafruit_Unified_Sensor
#include <Streaming.h>        // cout <iostream> functionality using Serial << endl;
#include <avr/wdt.h>          // add the dog
#include "fs6122.h"
#include "wdog.h"

// defines

// comment this for normal operation
//#define TIMESTAMP_DATA_OUTPUT

#define START_PWM_SPEED   120
#define CLOSE_DWELLMS     100
#define OPEN_SPEED        150
#define REQ_VOLUME        500
#define MAX_CLOSE_MS      1400

// position definitions
#define OPEN_POS_ADD  250
#define CLOSE_POS_ADD 400
#define MAX_POS       700

// pin definitions
#define OPEN_SW       9
#define CLOSED_SW     8
#define PAUSE_SW      4
#define PWM_PIN       3
#define DIR_PIN       2

#define CLOSE         HIGH
#define OPEN          LOW

#define MAIN_LOOP_TS_MS 0

enum sequence {
  INIT_CLOSE_SEQ,
  CLOSE_SEQ,
  CLOSE_DWELL_SEQ,
  OPEN_SEQ,
  OPEN_DWELL_SEQ
};

// Globals
bool run = false;
uint8_t dir;
long position;
long maxOpenPos;
long closePos;
int pwmSpeed = START_PWM_SPEED;
int desiredPwmSpeed = START_PWM_SPEED;
uint8_t openSwState;
uint8_t closeSwState;
long cycleCount;
unsigned long lastms;
unsigned long cycleTime;
long loopStartTime_us = 0;
double cpuLoad = 0;
long loopOverruns = 0;
unsigned long loopCounter = 0;
int count;
float accumVolml; 
  pressure_flow_type fs6122;

// Classes
Encoder myEnc(18, 19);  // use interrupt pins

/******************************************************************************/
/* Setup                                                                      */
/******************************************************************************/
void setup()
{

  pinMode(PAUSE_SW, INPUT_PULLUP);
  // This delay is enough time to program if there are issues with the code/programming
  delay(3000);
  Serial3.begin(115200);
  while(!Serial3);    // time to get serial running
  Serial3 << "Starting BagTest Fixture!" << "\r";

  //Init Flowmeter
  fs6122_init();
  fs6122_zeroFlowCal();

  pinMode(PWM_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(OPEN_SW, INPUT_PULLUP);
  pinMode(CLOSED_SW, INPUT_PULLUP);

  Home();
  dir = CLOSE;
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

//  getCommand();

  // disable the getCommand so it will run
  run = true;

  if (run == true) {

    fs6122_readSmlpM_umH2O(&fs6122);
    position = myEnc.read();

    // Communication with RaspberryPi
#ifdef TIMESTAMP_DATA_OUTPUT
    Serial3 << millis() << "," << fs6122.mSLPM << "," << fs6122.umH2O << ","
      << position << "," << dir << "," << pwmSpeed << "," 
      << analogRead(A0) << "," << accumVolml << "\r";
#else
    Serial3 << fs6122.mSLPM << "," << fs6122.umH2O << "," << accumVolml << "\r";
#endif

    Sequence();

    digitalWrite(DIR_PIN, dir);
    analogWrite(PWM_PIN, pwmSpeed);

  }
  else {
    // Stop
    analogWrite(PWM_PIN, 0);
  }

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
    maxOpenPos = myEnc.read();
    wdog_reset();
  } while (openSwState == 1);

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

  static char state = INIT_CLOSE_SEQ;
  static unsigned long nextms = 0;
  static unsigned long seqStartms;

  switch (state) {

    case INIT_CLOSE_SEQ:
      dir = CLOSE;
      pwmSpeed = desiredPwmSpeed;
      // get current time
      lastms = millis();
      seqStartms = lastms;
      accumVolml = 0;
      state = CLOSE_SEQ;
      break;

    case CLOSE_SEQ:
      if (abs(accumVolume())>= (float)REQ_VOLUME || (millis() - seqStartms) > MAX_CLOSE_MS) {
        nextms = millis() + CLOSE_DWELLMS;
        state = CLOSE_DWELL_SEQ;
        pwmSpeed = 0;
      }
      dir = CLOSE;
      break;

    case CLOSE_DWELL_SEQ:
      accumVolume();
      if (millis() > nextms) {

        state = OPEN_SEQ;
      }
      break;

    case OPEN_SEQ:
      accumVolume();
      if (position <= maxOpenPos) { 
        pwmSpeed = 0;
        nextms = millis() + 2*(millis() - seqStartms);
        state = OPEN_DWELL_SEQ;
      }
      else {
        dir = OPEN;
        pwmSpeed = OPEN_SPEED;
      }

      break;

    case OPEN_DWELL_SEQ:
      accumVolume();
      if (millis() > nextms)  {
        state = INIT_CLOSE_SEQ;
      }
      break;

    default:
      state=INIT_CLOSE_SEQ;
      break;
  }
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

float accumVolume(void) {
  float currentMS = millis();
  float timeSliceMS = (currentMS - lastms);
  float volumeml = ((float)fs6122.mSLPM/60000.0)*timeSliceMS; 

  lastms = currentMS;
  accumVolml += volumeml;

  return accumVolml;
}

/******************************************************************************/
/* getCommand                                                                 */
/*                                                                            */
/******************************************************************************/
void getCommand() {

  String command = Serial3.readString();

  if (command.equals("run")) {
    run = true;
  }

  if (command.equals("stop")) {
    run = false;
  }
}
