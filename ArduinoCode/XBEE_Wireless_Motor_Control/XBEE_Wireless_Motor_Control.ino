/************************************************
 ** Author: Trevor Sherrard
 ** Partner: Rodney Sanchez
 ** Class: Robotics Systems 
 ** Project: CHAAC unit robot
 ** Date: 11/18/2016
 ** File: XBEE_Wireless_Motor_Control.ino
 *************************************************/
#define XBEE Serial1

//motor pin constants
int MLPWMpin = 5;
int MRPWMpin = 6;
int MLDirPin = 4;
int MRDirPin = 7;
int D2pin = 3;
int SFpin = 2;

String strLDir = "";
String strRDir = "";
String strLVel = "";
String strRVel = "";

int LDir = 0;
int RDir = 0;
int LVel = 0;
int RVel = 0;

void setup(){
  Serial.begin(9600);
  XBEE.begin(9600);
  //motor pin assignments
  pinMode(SFpin, INPUT);
  pinMode(D2pin, OUTPUT);
  pinMode(MLDirPin, OUTPUT);
  pinMode(MLPWMpin, OUTPUT);
  pinMode(MRDirPin, OUTPUT);
  pinMode(MRPWMpin, OUTPUT);
} 

void loop(){
  //see if there is anything in serial buffer
  if(XBEE.available() > 0)
  {
    /*commands should be sent over in the format:
     * LDir,RDir,LVel,RVel
     * No spaces between; and null terminated String format
     */
    strLDir = XBEE.readStringUntil(',');
    XBEE.read();
    strRDir = XBEE.readStringUntil(',');
    XBEE.read();
    strLVel = XBEE.readStringUntil(',');
    XBEE.read();
    strRVel = XBEE.readStringUntil('\0');
  }
  //Enable Motors
  digitalWrite(D2pin, HIGH);
  //conver from string to int
  LDir = strLDir.toInt();
  RDir = strRDir.toInt();
  LVel = strLVel.toInt();
  RVel = strRVel.toInt();
  //Left Vel bound checks
  if(LVel < 0)
  {
    LVel = 0;
  }
  else if(LVel > 255)
  {
    LVel = 255;
  }

  //Right Vel bounds checks
  if(RVel < 0)
  {
    RVel = 0;
  }
  else if(RVel > 255)
  {
    RVel = 255;
  }

  //Left Motor Direction Control
  if(LDir != 0)
  {
    if(LDir = -1)
    {
      digitalWrite(MLDirPin, HIGH);
    }
    else if(LDir = 1)
    {
      digitalWrite(MLDirPin, HIGH);
    }
  }
  //Right Motor Direction Control
  if(RDir != 0)
  {
    if(RDir = -1)
    {
      digitalWrite(MRDirPin, HIGH);
    }
    else if(RDir = 1)
    {
      digitalWrite(MRDirPin, HIGH);
    }
  }
  //Write Vel values to motors
  analogWrite(MLPWMpin, LVel);
  analogWrite(MRPWMpin, RVel);

  //refresh delay
  delay(100);
}






