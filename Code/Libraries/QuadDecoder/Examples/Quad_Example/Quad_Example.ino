#include <QuadDecoder.h>
#include <ADC.h>
#define mod 2249	// Number of ticks per revolution

///////////////////// GLOBAL DATA TYPES ///////////////////////////
// Encoder A is for motor B and vis versa.  Tried to switch it and stopped working
QuadBase *encoderA = new QuadDecoder<1>();

double revs = 0.0;
///////////////////// Pin Deffinitions  ///////////////////////////
const int nD2 = 3;
const int M1DIR = 4;
const int M1PWM = 5;
const int nSF = 2;
const int M1FB = A0;



void setup() {
  // Quadtrature decoder setup
  // initialize the decoder with the # of counts per rev and which decoder to use
  encoderA->init(mod, 0); 
  encoderA->setFilter(31); // filter to help reduce noise 
  //encoderA->invertCHB(); // invert a channel if direction is wrong
  encoderA->reset(); // resets decoder valrables
  encoderA->begin(); // start decoder

  Serial.begin(9600);
  // Pin assignments
  pinMode(nD2, OUTPUT);
  pinMode(M1DIR, OUTPUT);
  pinMode(M1PWM, OUTPUT);
  pinMode(nSF, INPUT);
  pinMode(M1FB, INPUT);
  // enable motor controller
  digitalWrite(nD2, HIGH);
}


void loop() {
  for (int i = 0; i < 200; i++)
  {
    switch (i) {
      case 0:
        digitalWrite(M1DIR, LOW);
        analogWrite(M1PWM, 200);
        break;
      case 100:
        digitalWrite(M1DIR, HIGH);
        analogWrite(M1PWM, 200);
        break;
    }
	// calculate Revolutions and display it on terminal
    revs = double(encoderA->getCount()) / mod;
    Serial.println(revs);
    delay(10);
  }
}
