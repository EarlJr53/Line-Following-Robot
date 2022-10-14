#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motorL = AFMS.getMotor(1);
Adafruit_DCMotor *motorR = AFMS.getMotor(2);

void setup() {

  AFMS.begin();
  // put your setup code here, to run once:
  motorL->setSpeed(100);
  motorR->setSpeed(100);
  motorL->run(RELEASE);
  motorR->run(RELEASE);

  long baudRate = 9600;   // NOTE1: The baudRate for sending & receiving programs must match
  Serial.begin(baudRate); // NOTE2: Set the baudRate to 115200 for faster communication
  Serial.setTimeout(1);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  motorL->run(FORWARD);
  motorR->run(FORWARD);
  Serial.println("hi");
}
