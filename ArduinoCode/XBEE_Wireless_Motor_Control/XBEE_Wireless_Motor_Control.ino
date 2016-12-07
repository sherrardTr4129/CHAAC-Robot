/************************************************
 ** Author: Trevor Sherrard
 ** Partner: Rodney Sanchez
 ** Class: Robotics Systems
 ** Project: CHAAC unit robot
 ** Date: 11/18/2016
 ** File: XBEE_Wireless_Motor_Control.ino
 *************************************************/
#include "QuadDecoder.h"
#include <ADC.h>
#define mod 2249  // Number of ticks per wheel revolution
#define XBEE Serial1
int readQD(int LineSensePin);

QuadBase *encoderA = new QuadDecoder<1>();
IntervalTimer velTimerM2;


QuadBase *encoderB = new QuadDecoder<1>();
IntervalTimer velTimerM1;


// variables to be used in the timer interrupt
volatile  int oldCntA = 0; // used to store the count the last time you calculated
volatile int newCntA = 0; //  new value to use in calculating difference
volatile int diffA = 0; // difference between newCntA and oldCntA
volatile  int oldCntB = 0; // used to store the count the last time you calculated
volatile int newCntB = 0; //  new value to use in calculating difference
volatile int diffB = 0; // difference between newCntA and oldCntA

const double rpmConvert =  2.667; // rev/min = cnt/us * us/sec * sec/min * rev/cnt
const int period = 10000; // in microseconds



//motor pin constants
const int MLPWMPin = 5;
const int MRPWMPin = 6;
const int MLDirPin = 4;
const int MRDirPin = 7;
const int D2pin = 3;
const int SFpin = 2;

//Line Sensor Pin declaration
const int LineSensorPin = 8;

//Serial read variables
int RMotorSpeed, LMotorSpeed = 0;
char DirChar;
float LSerFloat = 0;
float RSerFloat = 0;


void setup() {
  Serial.begin(9600);
  XBEE.begin(9600);
  //motor pin assignments
  pinMode(SFpin, INPUT);
  pinMode(D2pin, OUTPUT);
  pinMode(MLDirPin, OUTPUT);
  pinMode(MLPWMPin, OUTPUT);
  pinMode(MRDirPin, OUTPUT);
  pinMode(MRPWMPin, OUTPUT);
}

void loop() {
  //set motor speed state back to 0 for next update
  analogWrite(MRPWMPin, 0);
  analogWrite(MLPWMPin, 0);
  digitalWrite(D2pin, HIGH);
  //see if there is anything in serial buffer
  if (XBEE.available() > 0)
  {
    DirChar = XBEE.read();
    if (DirChar == 'l')
    {
      LSerFloat = XBEE.parseFloat();
      if (LSerFloat < 120)
      {
        digitalWrite(MLDirPin, HIGH);
        LMotorSpeed = map(int(LSerFloat), 0, 400, 255, 0);

      }
      else if (LSerFloat > 210)
      {
        digitalWrite(MLDirPin, LOW);
        LMotorSpeed = map(int(LSerFloat), 0, 400, 0, 255);

      }
      else if (LSerFloat < 220 && LSerFloat > 130)
      {
        LMotorSpeed = 0;
      }
    }
    else if (DirChar == 'r') {
      RSerFloat = XBEE.parseFloat();
      if (RSerFloat < 150)
      {
        //right forward
        digitalWrite(MRDirPin, LOW);
        RMotorSpeed = map(int(RSerFloat), 0, 400, 255, 0);
      }
      else if (RSerFloat > 225)
      {
        //right reverse
        digitalWrite(MRDirPin, HIGH);
        RMotorSpeed = map(int(RSerFloat), 0, 400, 0, 255);

      }
      else if (RSerFloat < 220 && RSerFloat > 140)
      {
        RMotorSpeed = 0;
      }
    }
  }
  analogWrite(MRPWMPin, RMotorSpeed);
  analogWrite(MLPWMPin, LMotorSpeed);

  Serial.print("Speed L, MotorDir: " );
  Serial.print(LMotorSpeed);
  Serial.print(",");
  Serial.println(digitalRead(MLDirPin));
  Serial.print("Speed R, MotorDir: " );
  Serial.print(RMotorSpeed);
  Serial.print(",");
  Serial.println(digitalRead(MRDirPin));
  Serial.print("LineSensor Read value: ");
  Serial.println(readQD(LineSensorPin));
  Serial.println();
  delay(5);
}

int readQD(int LineSensePin) {
  pinMode(LineSensePin, OUTPUT);
  digitalWrite(LineSensePin, HIGH);
  delayMicroseconds(10);
  pinMode(LineSensePin, INPUT);

  long curTime = micros();

  while (digitalRead(LineSensePin) && (micros() - curTime) < 3000);
  return micros() - curTime;
}
