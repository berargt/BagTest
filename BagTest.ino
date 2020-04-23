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

Encoder myEnc(18, 19);  // use interrupt pins
uint8_t oldSW, newSW;
bool updateDisplay = true;
uint8_t dir = CLOSE;
long minPosition, maxPosition;
long newPosition;
long maxOpenPos;
long closePos;
int pwmSpeed = 87;
uint8_t openSwState;
uint8_t closeSwState;
long cycleCount = 0;
unsigned long lastTime;

LiquidCrystal_I2C lcd(0x27, 20, 4); // set the LCD address to 0x27 for a 16 chars and 2 line display

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
  oldSW = newSW = digitalRead(INPIN);
  pinMode(PWM_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(OPEN_SW, INPUT_PULLUP);
  pinMode(CLOSED_SW, INPUT_PULLUP);
  
  minPosition = maxPosition = 0;
  doHome();
  lastTime = millis();
}

// The main loop
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
  
  newPosition > maxPosition ? maxPosition = newPosition : maxPosition;
  newPosition < minPosition ? minPosition = newPosition : minPosition;
}

// Home the fixture
// get open and close counts
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

// When transitioning from open to close and visa versa these are redundent
void doTransition(void) {
  digitalWrite(DIR_PIN, dir);
  analogWrite(PWM_PIN, pwmSpeed);
  displayLCD();
}

// display data to the LCD
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
