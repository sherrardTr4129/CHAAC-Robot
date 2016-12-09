/************************************************
 ** Author: Trevor Sherrard
 ** Partner: Rodney Sanchez
 ** Class: Robotics Systems
 ** Project: CHAAC unit robot
 ** Date: 11/18/2016
 ** File: XBEE_Wireless_Motor_Control.ino
 *************************************************/
#include <stdint.h>
#include "SparkFunBME280.h"
#include "QuadDecoder.h"
#include "SPI.h"
#include "Wire.h"
#define mod 2249  // Number of ticks per wheel revolution
#define XBEE Serial1

//function Prototypes
int readQD(int LineSensePin);
long microsecondsToCentimeters(long microseconds);
long GetUltrasonicTOF();
void readandupdate(float T, float HU, float PR);

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
int LineThreshold = 400;

//motor pin constants
const int MLPWMPin = 5;
const int MRPWMPin = 6;
const int MLDirPin = 4;
const int MRDirPin = 7;
const int D2pin = 3;
const int SFpin = 2;

//Sensor Pin declarations
const int LineSensorPin = 13;
const int EchoPin = 14;
const int TriggerPin = 15;
long duration, cm = 0;
long Distance = 0;
//Serial read variables
int RMotorSpeed, LMotorSpeed = 0;
char DirChar = 0;
float LSerFloat = 0;
float RSerFloat = 0;

//Mode State Variable
bool isWeatherMode = true;

//Predicted Weather Character
char WeatherDanceChar = 0;
//Sparkfun BME280 Setup
BME280 mySensor;

//averages
float PRA = 0.0;
float HUA = 0.0;
float TA = 0.0;

//current readings
float T = 0.0;
float HU = 0.0;
float PR = 0.0;

int count = 0;

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

  //Setup Ultrasonic
  pinMode(EchoPin, INPUT);
  pinMode(TriggerPin, OUTPUT);
  //Setup Sparkfun BME280
  //***Driver settings********************************//
  //commInterface can be I2C_MODE or SPI_MODE
  //specify chipSelectPin using arduino pin names
  //specify I2C address.  Can be 0x77(default) or 0x76

  //For I2C, enable the following and disable the SPI section
  mySensor.settings.commInterface = I2C_MODE;
  mySensor.settings.I2CAddress = 0x77;

  //For SPI enable the following and dissable the I2C section
  //mySensor.settings.commInterface = SPI_MODE;
  //mySensor.settings.chipSelectPin = 10;


  //***Operation settings*****************************//

  //renMode can be:
  //  0, Sleep mode
  //  1 or 2, Forced mode
  //  3, Normal mode
  mySensor.settings.runMode = 3; //Normal mode

  //tStandby can be:
  //  0, 0.5ms
  //  1, 62.5ms
  //  2, 125ms
  //  3, 250ms
  //  4, 500ms
  //  5, 1000ms
  //  6, 10ms
  //  7, 20ms
  mySensor.settings.tStandby = 0;

  //filter can be off or number of FIR coefficients to use:
  //  0, filter off
  //  1, coefficients = 2
  //  2, coefficients = 4
  //  3, coefficients = 8
  //  4, coefficients = 16
  mySensor.settings.filter = 0;

  //tempOverSample can be:
  //  0, skipped
  //  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
  mySensor.settings.tempOverSample = 1;

  //pressOverSample can be:
  //  0, skipped
  //  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
  mySensor.settings.pressOverSample = 1;

  //humidOverSample can be:
  //  0, skipped
  //  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
  mySensor.settings.humidOverSample = 1;

  Serial.print("Program Started\n");
  Serial.print("Starting BME280... result of .begin(): 0x");

  //Calling .begin() causes the settings to be loaded
  delay(10);  //Make sure sensor had enough time to turn on. BME280 requires 2ms to start up.
  Serial.println(mySensor.begin(), HEX);

  Serial.print("Displaying ID, reset and ctrl regs\n");

  Serial.print("ID(0xD0): 0x");
  Serial.println(mySensor.readRegister(BME280_CHIP_ID_REG), HEX);
  Serial.print("Reset register(0xE0): 0x");
  Serial.println(mySensor.readRegister(BME280_RST_REG), HEX);
  Serial.print("ctrl_meas(0xF4): 0x");
  Serial.println(mySensor.readRegister(BME280_CTRL_MEAS_REG), HEX);
  Serial.print("ctrl_hum(0xF2): 0x");
  Serial.println(mySensor.readRegister(BME280_CTRL_HUMIDITY_REG), HEX);

  Serial.print("\n\n");

  Serial.print("Displaying all regs\n");
  uint8_t memCounter = 0x80;
  uint8_t tempReadData;
  for (int rowi = 8; rowi < 16; rowi++ )
  {
    Serial.print("0x");
    Serial.print(rowi, HEX);
    Serial.print("0:");
    for (int coli = 0; coli < 16; coli++ )
    {
      tempReadData = mySensor.readRegister(memCounter);
      Serial.print((tempReadData >> 4) & 0x0F, HEX);//Print first hex nibble
      Serial.print(tempReadData & 0x0F, HEX);//Print second hex nibble
      Serial.print(" ");
      memCounter++;
    }
    Serial.print("\n");
  }


  Serial.print("\n\n");

  Serial.print("Displaying concatenated calibration words\n");
  Serial.print("dig_T1, uint16: ");
  Serial.println(mySensor.calibration.dig_T1);
  Serial.print("dig_T2, int16: ");
  Serial.println(mySensor.calibration.dig_T2);
  Serial.print("dig_T3, int16: ");
  Serial.println(mySensor.calibration.dig_T3);

  Serial.print("dig_P1, uint16: ");
  Serial.println(mySensor.calibration.dig_P1);
  Serial.print("dig_P2, int16: ");
  Serial.println(mySensor.calibration.dig_P2);
  Serial.print("dig_P3, int16: ");
  Serial.println(mySensor.calibration.dig_P3);
  Serial.print("dig_P4, int16: ");
  Serial.println(mySensor.calibration.dig_P4);
  Serial.print("dig_P5, int16: ");
  Serial.println(mySensor.calibration.dig_P5);
  Serial.print("dig_P6, int16: ");
  Serial.println(mySensor.calibration.dig_P6);
  Serial.print("dig_P7, int16: ");
  Serial.println(mySensor.calibration.dig_P7);
  Serial.print("dig_P8, int16: ");
  Serial.println(mySensor.calibration.dig_P8);
  Serial.print("dig_P9, int16: ");
  Serial.println(mySensor.calibration.dig_P9);

  Serial.print("dig_H1, uint8: ");
  Serial.println(mySensor.calibration.dig_H1);
  Serial.print("dig_H2, int16: ");
  Serial.println(mySensor.calibration.dig_H2);
  Serial.print("dig_H3, uint8: ");
  Serial.println(mySensor.calibration.dig_H3);
  Serial.print("dig_H4, int16: ");
  Serial.println(mySensor.calibration.dig_H4);
  Serial.print("dig_H5, int16: ");
  Serial.println(mySensor.calibration.dig_H5);
  Serial.print("dig_H6, uint8: ");
  Serial.println(mySensor.calibration.dig_H6);

  Serial.println();

  //Enable Motors
  digitalWrite(D2pin, HIGH);
  Serial.println("Init is done!");
}

void loop() {

  if (XBEE.available() > 0)
  {
    DirChar = XBEE.read();
    if (DirChar == 1)
    {
      isWeatherMode = !isWeatherMode;
    }
  }


  if (!isWeatherMode)
  {
    int LineSensorVal = readQD(LineSensorPin);
    long Distance = GetUltrasonicTOF();

    //On the Line, step off to the right
    if (LineSensorVal < LineThreshold)
    {
      digitalWrite(MLDirPin, HIGH);
      digitalWrite(MRDirPin, HIGH);
      analogWrite(MRPWMPin, 0);
      analogWrite(MLPWMPin, 160);
    }
    //Off the line, step on from the left
    else if (LineSensorVal > LineThreshold)
    {
      digitalWrite(MLDirPin, LOW);
      digitalWrite(MRDirPin, LOW);
      analogWrite(MRPWMPin, 160);
      analogWrite(MLPWMPin, 0);

    }
    if (Distance < 15)
    {
      //turn right
      digitalWrite(MLDirPin, HIGH);
      digitalWrite(MRDirPin, HIGH);
      analogWrite(MRPWMPin, 128);
      analogWrite(MLPWMPin, 128);
      delay(500);
      //go forward
      digitalWrite(MLDirPin, HIGH);
      digitalWrite(MRDirPin, LOW);
      analogWrite(MRPWMPin, 255);
      analogWrite(MLPWMPin, 255);
      delay(1700);
      //Turn Left
      digitalWrite(MLDirPin, LOW);
      digitalWrite(MRDirPin, LOW);
      analogWrite(MRPWMPin, 255);
      analogWrite(MLPWMPin, 255);
      delay(400);
      //go forward
      digitalWrite(MLDirPin, HIGH);
      digitalWrite(MRDirPin, LOW);
      analogWrite(MRPWMPin, 255);
      analogWrite(MLPWMPin, 255);
      delay(900);
      //Turn Left
      digitalWrite(MLDirPin, LOW);
      digitalWrite(MRDirPin, LOW);
      analogWrite(MRPWMPin, 255);
      analogWrite(MLPWMPin, 255);
      delay(150);
      //go forward Until Line Detected
      digitalWrite(MLDirPin, HIGH);
      digitalWrite(MRDirPin, LOW);
      analogWrite(MRPWMPin, 160);
      analogWrite(MLPWMPin, 128);
      while (readQD(LineSensorPin) < 300);
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
    else if (DirChar == 2)
    {
      analogWrite(MRPWMPin, 0);
      analogWrite(MLPWMPin, 0);
      readandupdate(T, HU, PR);
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
  /*
    Serial.print("Speed L, MotorDir: " );
    Serial.print(LMotorSpeed);
    Serial.print(",");
    Serial.println(digitalRead(MLDirPin));
    Serial.print("Speed R, MotorDir: " );
    Serial.print(RMotorSpeed);
    Serial.print(",");
    Serial.println(digitalRead(MRDirPin));
    Serial.println();
  */
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

long GetUltrasonicTOF()
{
  digitalWrite(TriggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(TriggerPin, LOW);
  duration = pulseIn(EchoPin, HIGH);
  return microsecondsToCentimeters(duration);
}

void readandupdate(float T, float HU, float PR)
{
  PRA = 0.0;
  HUA = 0.0;
  TA = 0.0;
  while (count < 101)
  {
    T = mySensor.readTempF();
    PR = (mySensor.readFloatPressure()) / 3386.39;
    HU = (mySensor.readFloatHumidity());
    PRA = PR + PRA;
    HUA = HU + HUA;
    TA = T + TA;
    count = count + 1;
  }
  count = 0.0;
  PRA = (PRA / 100.0);
  HUA = (HUA / 100.0);
  TA = (TA / 100.0);
  //Print These over Serial
  Serial.print("Degrees F: ");
  Serial.println(TA, 2);
  Serial.print("Pressure: ");
  Serial.println(PRA, 2);
  Serial.print("%RH: ");
  Serial.println(HUA, 2);
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

long microsecondsToCentimeters(long microseconds)
{
  return microseconds / 29 / 2;
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
