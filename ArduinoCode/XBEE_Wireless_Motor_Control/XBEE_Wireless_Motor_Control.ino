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
//function Prototypes
int readQD(int LineSensePin);
long GetCentimetersFromPING(int PingPin);
void calcVelB(void);
void calcVelA(void);
void CloudyDance(void);
void StormyDance(void);
void SnowyDance(void);
void ClearDance(void);
void RainyDance(void);

QuadBase *encoderA = new QuadDecoder<1>();
IntervalTimer velTimerA;


QuadBase *encoderB = new QuadDecoder<2>();
IntervalTimer velTimerB;


// variables to be used in the timer interrupt
volatile  int oldCntA = 0; // used to store the count the last time you calculated
volatile int newCntA = 0; //  new value to use in calculating difference
volatile int diffA = 0; // difference between newCntA and oldCntA
volatile  int oldCntB = 0; // used to store the count the last time you calculated
volatile int newCntB = 0; //  new value to use in calculating difference
volatile int diffB = 0; // difference between newCntB and oldCntB

//Conversion Constants
const double rpmConvert =  2.667; // rev/min = cnt/us * us/sec * sec/min * rev/cnt
const int period = 10000; // in microseconds
int LineThreshold = 1000;

//motor pin constants
const int MLPWMPin = 5;
const int MRPWMPin = 6;
const int MLDirPin = 4;
const int MRDirPin = 7;
const int D2pin = 3;
const int SFpin = 2;

//Sensor Pin declarations
const int RaspberryPiInterrupt = 19;
const int LineSensorPin = 15;
const int PINGPin = 10;
long duration, cm = 0;


//Serial read variables
int RMotorSpeed, LMotorSpeed = 0;
char DirChar;
float LSerFloat = 0;
float RSerFloat = 0;


//Mode State Variable
bool isWeatherMode = false;

//Predicted Weather Character
char WeatherDanceChar = 0;

void setup() {
  Serial.begin(9600);
  XBEE.begin(9600);
  //EncoderA Init
  encoderA->init(mod, 0);
  encoderA->setFilter(31);
  encoderA->reset();
  encoderA->begin();
  //EncoderB Init
  encoderB->init(mod, 0);
  encoderB->setFilter(31);
  encoderB->reset();
  encoderB->begin();
  //motor pin assignments
  pinMode(SFpin, INPUT);
  pinMode(D2pin, OUTPUT);
  pinMode(MLDirPin, OUTPUT);
  pinMode(MLPWMPin, OUTPUT);
  pinMode(MRDirPin, OUTPUT);
  pinMode(MRPWMPin, OUTPUT);
  //start timer interrupts
  velTimerA.begin(calcVelA, period);
  velTimerB.begin(calcVelB, period);
  //Enable Motors
  digitalWrite(D2pin, HIGH);
}

void loop() {

  //see if there is anything in serial buffer
  if (XBEE.available() > 0)
  {
    DirChar = XBEE.read();
    if (DirChar == 's')
    {
      isWeatherMode = !isWeatherMode;
    }
  }
  if (!isWeatherMode)
  {
    //On the Line, step off to the right
    if (readQD(LineSensorPin) < LineThreshold)
    {
      digitalWrite(MLDirPin, HIGH);
      digitalWrite(MRDirPin, HIGH);
      analogWrite(MRPWMPin, 128);
      analogWrite(MLPWMPin, 128);
    }
    //Off the line, step on from the left
    else
    {
      digitalWrite(MLDirPin, LOW);
      digitalWrite(MRDirPin, LOW);
      analogWrite(MRPWMPin, 128);
      analogWrite(MLPWMPin, 128);
    }
    if (GetCentimetersFromPING(PINGPin) < 16)
    {
      //turn right
      digitalWrite(MLDirPin, LOW);
      digitalWrite(MRDirPin, HIGH);
      analogWrite(MRPWMPin, 128);
      analogWrite(MLPWMPin, 128);
      delay(800);
      //go forward
      digitalWrite(MLDirPin, HIGH);
      digitalWrite(MRDirPin, HIGH);
      analogWrite(MRPWMPin, 255);
      analogWrite(MLPWMPin, 255);
      delay(800);
      //Turn Left
      digitalWrite(MLDirPin, HIGH);
      digitalWrite(MRDirPin, LOW);
      analogWrite(MRPWMPin, 255);
      analogWrite(MLPWMPin, 255);
      delay(800);
      //go forward Until Line Detected
      digitalWrite(MLDirPin, HIGH);
      digitalWrite(MRDirPin, HIGH);
      analogWrite(MRPWMPin, 255);
      analogWrite(MLPWMPin, 255);
      while (readQD(LineSensorPin) < LineThreshold);
    }
  }
  else if (isWeatherMode)
  {
    //set motor speed state back to 0 for next update
    analogWrite(MRPWMPin, 0);
    analogWrite(MLPWMPin, 0);
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
    else if (DirChar == 'r')
    {
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
    else if (DirChar == 'w')
    {
      analogWrite(MRPWMPin, 0);
      analogWrite(MLPWMPin, 0);
      digitalWrite(RaspberryPiInterrupt, HIGH);
      delayMicroseconds(100);
      digitalWrite(RaspberryPiInterrupt, LOW);
      while (Serial.available() <= 0);
      WeatherDanceChar = Serial.read();
      switch (WeatherDanceChar)
      {
        case 's':
          StormyDance();
          break;
        case 'c':
          CloudyDance();
          break;
        case 'n':
          SnowyDance();
          break;
        case 'l':
          ClearDance();
          break;
        case 'r':
          RainyDance();
          break;
      }
    }
    //write motorvalues
    analogWrite(MRPWMPin, RMotorSpeed);
    analogWrite(MLPWMPin, LMotorSpeed);
  }
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
  Serial.print("isWeatherMode? ");
  Serial.println(isWeatherMode);
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

long GetCentimetersFromPING(int pingPin)
{
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);
  pinMode(pingPin, INPUT);
  duration = pulseIn(pingPin, HIGH);
  return duration / 29 / 2;
}


void calcVelA(void)
{
  // Use the getCount function to determine the change in counts
  // remeber to update oldCntA
  newCntA = encoderA->getCount();
  diffA = newCntA - oldCntA;
  oldCntA = newCntA;
}

void calcVelB(void)
{
  // Use the getCount function to determine the change in counts
  // remeber to update oldCntB
  newCntB = encoderB->getCount();
  diffB = newCntB - oldCntB;
  oldCntB = newCntB;
}

void CloudyDance(void)
{
  
}
void StormyDance(void)
{
  
}
void SnowyDance(void)
{
  
}
void ClearDance(void)
{
  
}
void RainyDance(void)
{
  
}

