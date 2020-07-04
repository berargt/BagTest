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

#define SPEED         128
#define OPEN_POS_ADD  200
#define CLOSE_POS_ADD 600
#define OPEN_DWELL    1250
#define TGT_CYC_MS    2000
#define TGT_HST       10

#define OPEN_SW       9
#define CLOSED_SW     8
#define PAUSE_SW      4
#define PWM_PIN       3
#define DIR_PIN       2
#define CLOSE         HIGH
#define OPEN          LOW

#define MAIN_LOOP_TS_MS 0

#define  OPENING 0
#define  DWELL   1
#define  TRANS   2

// Globals
uint8_t dir;
long position;
long maxOpenPos;
long closePos;
int pwmSpeed;
int speed = SPEED;
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
  pwmSpeed = speed;
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
  Serial3 << "Start Loop" << endl;
  int i=SPEED;
  for(;i<256; i+=8) {
    for (count=0;count<5;) {
      speed = i;

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
      Serial3 << fs6122.flow_rate << "," << fs6122.pressure << "," << ch3Val << endl;
#endif

      Sequence();

      digitalWrite(DIR_PIN, dir);
      analogWrite(PWM_PIN, pwmSpeed);

      //  displayLCD();
      wdog_reset();
      markLoopEnd();
    }
  }
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
  analogWrite(PWM_PIN, speed);

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

	static char state = OPENING;
	static unsigned long start_time = 0;

	if (dir == CLOSE) {
		pwmSpeed = speed;

		if (position > closePos) {
			dir = OPEN;
		}
	} 
	else { // dir == OPEN or something else
		switch (state)
		{
			case OPENING:
				if (position < maxOpenPos) {
					state = DWELL; // advance to next state
					start_time = millis();
				}
				pwmSpeed = speed;
				break;

			case DWELL:
				if ((millis() - start_time)  > OPEN_DWELL) {
					state = TRANS; // advance to next state
				}
				else {
					// stop motor
					pwmSpeed = 0;
				}
				break;

			case TRANS:
				dir = CLOSE;
				cycleCount++;
				cycleTime = millis()-lastTime;
				state = OPENING;
				pwmSpeed = speed;
				lastTime = millis();
            count++;

				break;
		}
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
