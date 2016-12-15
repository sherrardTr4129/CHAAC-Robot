#include <QuadDecoder.h>
#include <ADC.h>
#define mod 2249	// Number of ticks per revolution

///////////////////// GLOBAL DATA TYPES ///////////////////////////
// Encoder A is for motor B and vis versa.  Tried to switch it and stopped working
QuadBase *encoderA = new QuadDecoder<1>();
IntervalTimer velTimer;

///////////////////// Pin Deffinitions  ///////////////////////////
const int nD2 = 3;
const int M1DIR = 4;
const int M1PWM = 5;
const int nSF = 2;
const int M1FB = A0;

const int period = 10000;
const double rpmConvert =  60000000 / (2248.86*period) ; // rev/min = cnt/us * us/sec * sec/min * rev/cnt
int oldRevA = 0;
int oldCntA = 0;
int newRevA = 0;
int newCntA = 0;
int diffA = 0;

void calcVel(void) 
{
  newCntA = encoderA->getCount();
  diffA = (newCntA - oldCntA);
  oldCntA = newCntA;
}

void setup() {
  // Quadtrature decoder setup
  encoderA->init(mod, 0);
  encoderA->setFilter(31);
  //encoderA->invertCHB();
  encoderA->reset();
  encoderA->begin();
  Serial.begin(9600);
  
  pinMode(nD2, OUTPUT);
  pinMode(M1DIR, OUTPUT);
  pinMode(M1PWM, OUTPUT);
  pinMode(nSF, INPUT);
  pinMode(M1FB, INPUT);
  // enable motor controller
  digitalWrite(nD2, HIGH);

  velTimer.begin(calcVel, period);

}
void loop() {
  for (int i = 0; i < 200; i++)
    {
    switch (i) {
      case 50:
        digitalWrite(M1DIR, LOW);
        analogWrite(M1PWM, 200);
        break;
      case 75:
        digitalWrite(M1DIR, HIGH);
        analogWrite(M1PWM, 200);
        break;
      case 150:
        digitalWrite(M1DIR, LOW);
        analogWrite(M1PWM, 0);
        break;
    }
	// calculate RPM and display it on terminal
    Serial.println( diffA * rpmConvert, DEC);
    delay(10);
    }
}
