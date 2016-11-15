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

int MotorSpeed = 128;

char SerChar = 0;

void setup(){
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

void loop(){
  //set motor speed state back to 0 for next update
  analogWrite(MRPWMPin, 0);
  analogWrite(MLPWMPin, 0);
  SerChar = 0;
  //see if there is anything in serial buffer
  if(XBEE.available() > 0)
  {
    SerChar = XBEE.read();
    XBEE.println("ACK");
  }
  //Enable Motors
  digitalWrite(D2pin, HIGH);
  Serial.println(SerChar);
  // direction values for specific motors may need to be flipped. 
  // The directions are assumed currently; they are not tested
  if(SerChar == 'r')
  {
    //go right
    digitalWrite(MRDirPin, HIGH);
    digitalWrite(MLDirPin, LOW);
    analogWrite(MRPWMPin, MotorSpeed);
    analogWrite(MLPWMPin, MotorSpeed);
  }
  else if(SerChar == 'l')
  {
    //go left
    digitalWrite(MRDirPin, LOW);
    digitalWrite(MLDirPin, HIGH);
    analogWrite(MRPWMPin, MotorSpeed);
    analogWrite(MLPWMPin, MotorSpeed);
  }
  else if(SerChar == 'f')
  {
    //go forward
    digitalWrite(MRDirPin, HIGH);
    digitalWrite(MLDirPin, HIGH);
    analogWrite(MRPWMPin, MotorSpeed);
    analogWrite(MLPWMPin, MotorSpeed);
  }
  else if(SerChar == 'b')
  {
    //go back
    digitalWrite(MRDirPin, LOW);
    digitalWrite(MLDirPin, LOW);
    analogWrite(MRPWMPin, MotorSpeed);
    analogWrite(MLPWMPin, MotorSpeed);
  }
  //refresh delay
  delay(50);
}
