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
int MLPWMPin = 5;
int MRPWMPin = 6;
int MLDirPin = 4;
int MRDirPin = 7;
int D2pin = 3;
int SFpin = 2;

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
  //see if there is anything in serial buffer
  if (XBEE.available() > 0)
  {
    DirChar = XBEE.read();
    if (DirChar == 'l')
    {
      LSerFloat = XBEE.parseFloat();
      if (LSerFloat < 120)
      {
        digitalWrite(MLDirPin, LOW);
        LMotorSpeed = map(int(LSerFloat), 0, 400, 255, 0);

      }
      else if (LSerFloat > 210)
      {
        digitalWrite(MLDirPin, HIGH);
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
  Serial.println();
  delay(5);
}
