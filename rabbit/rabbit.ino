#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

int request = 0;

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor motorL = AFMS.getMotor(1);
Adafruit_DCMotor motorR = AFMS.getMotor(2);

int sensorL;
int sensorR;

void setup()
{
    AFMS.begin();
    //
    // start the serial port
    //
    long baudRate = 9600;   // NOTE1: The baudRate for sending & receiving programs must match
    Serial.begin(baudRate); // NOTE2: Set the baudRate to 115200 for faster communication
    Serial.setTimeout(1);

    // Only continue while serial connection is available
    // This means nothing happens until the Python program is run
    while (!Serial.available())
    {
    }

    while (request == 0)
    {
        request = Serial.readString().toInt();
    }

    motorL->setSpeed(100*request)
    motorR->setSpeed(100*request)
}


void loop()
{
    if (Serial.readString().toInt() == 0)
    {
      exit;
    }
    
    if (!sensorL && !sensorR)
    {
      motorL->run(FORWARD);
      motorR->run(FORWARD);
    }
    else if (!sensorL && sensorR)
    {
      motorL->run(FORWARD);
      motorR->run(BACKWARD);
    }
    else if (sensorL && !sensorR)
    {
      motorL->run(BACKWARD);
      motorR->run(FORWARD);
    }

}
