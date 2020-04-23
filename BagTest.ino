// Author Gregory Berardi
// 04/22/2020
// BagTest.ino
// Code to test the Ambu Bag and Ventilator Fixture
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Encoder.h>
#include <stdio.h>
#include <Streaming.h> // cout <iosstream> functionality using Serial << endl;

#define INPIN       4
#define OPEN_SW     9
#define CLOSED_SW   8
#define PWM_PIN     3
#define DIR_PIN     2
#define CLOSE       HIGH
#define OPEN        LOW

uint8_t dir;
//long minPosition, maxPosition;
long newPosition;
long maxOpenPos;
long closePos;
int pwmSpeed;
uint8_t openSwState;
uint8_t closeSwState;
long cycleCount;
unsigned long lastTime;

Encoder myEnc(18, 19);  // use interrupt pins
LiquidCrystal_I2C lcd(0x27, 20, 4); // set the LCD address to 0x27 for a 16 chars and 2 line display

/******************************************************************************/
/* Setup                                                                      */
/******************************************************************************/
void setup()
{
  lcd.init();                      // initialize the lcd
  lcd.backlight();
  lcd.home();

  lcd.print("Initializing...");
  lcd.setCursor(0, 1);
  pinMode(INPIN, INPUT);
  lcd.print("Ambu Bag Fixture!");
  delay(3000);
  Serial.begin(115200);
  Serial << "Ambu Bag Fixture Starting!" << endl;
  pinMode(PWM_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(OPEN_SW, INPUT_PULLUP);
  pinMode(CLOSED_SW, INPUT_PULLUP);

  dir = CLOSE;
  pwmSpeed = 87;
  
//  minPosition = maxPosition = 0;

  doHome();
  lastTime = millis();
  cycleCount = 0;
}

/******************************************************************************/
/* Main Loop                                                                  */
/******************************************************************************/
void loop()
{
  newPosition = myEnc.read();

  digitalWrite(DIR_PIN, dir);
  analogWrite(PWM_PIN, pwmSpeed);

  if (newPosition > closePos && dir == CLOSE) {
    dir = OPEN;
    doTransition();
  } else if (newPosition < maxOpenPos && dir == OPEN) {
    dir = CLOSE;
    cycleCount++;
    doTransition();
    Serial << "CYC:" << cycleCount << " Time:" << millis()-lastTime << endl;
    lastTime = millis();
  }
  openSwState = digitalRead(OPEN_SW);
  closeSwState = digitalRead(CLOSED_SW);

  if ((openSwState==0 || closeSwState==0) && pwmSpeed != 0) {
    pwmSpeed = 0; // stop the fixture
    Serial << (openSwState==0?"Failed Open":"Failed Closed") << endl;
  }
  
//  newPosition > maxPosition ? maxPosition = newPosition : maxPosition;
//  newPosition < minPosition ? minPosition = newPosition : minPosition;
}


/******************************************************************************/
/* Home                                                                       */
/*                                                                            */
/* home the jaws by opening fully and calculate the min/max jaw position      */
/******************************************************************************/

void doHome(void) {
  uint8_t homeSpeed = 25; // go slow

  lcd.clear(); //display status of motor on LCD
  lcd.setCursor(0, 0);
  lcd.print("Homing...");

  dir = OPEN;
  digitalWrite(DIR_PIN, OPEN);
  analogWrite(PWM_PIN, homeSpeed);
  
  do {
    openSwState = digitalRead(OPEN_SW);
    lcd.setCursor(0, 1); lcd.print("SW:");lcd.print(openSwState);
    maxOpenPos = myEnc.read();
    lcd.setCursor(0, 2); lcd.print("POS:"); lcd.print(maxOpenPos);
  } while (openSwState == 1);

  analogWrite(PWM_PIN, 0);      // Stop the motor
  maxOpenPos = myEnc.read() + 50;
  closePos = maxOpenPos + 500;
  Serial << "maxOpenPos:" << maxOpenPos << endl << "closePos:" << closePos << endl;
  Serial << "pwmSpeed:" << pwmSpeed << endl;
}

/******************************************************************************/
/* Transition from open to close                                              */
/*                                                                            */
/* When transitioning from open to close and visa versa these are redundant   */
/******************************************************************************/
void doTransition(void) {
  digitalWrite(DIR_PIN, dir);
  analogWrite(PWM_PIN, pwmSpeed);
  displayLCD();
}

/******************************************************************************/
/* Transition from open to close                                              */
/*                                                                            */
/* display data to the LCD                                                    */
/******************************************************************************/
void displayLCD(void) {
  lcd.clear(); //display status of motor on LCD
  lcd.setCursor(0, 0);
  lcd.print("PWM:");
  lcd.print(pwmSpeed);
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
