/*
* File:     QuadDecoder.h
* Author:   Matt Haywood
* Date:     10/31/14
*
* Description: 
* 		definition of class Quadrature
*
*/


#ifndef QuadDecoder_h_
#define QuadDecoder_h_
#include <vector>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "Arduino.h"
#include "kinetis.h"

// Enumoration to make it easier to get register info
enum reggister {CNTIN, MOD, SC, QDCTRL, MODE, CNT, FILTER, FMS, FLTCTRL, IRQ};

// a typedef for regesters 
typedef volatile uint32_t* reg;
// Base class for inheritance
class QuadBase{
public:
	virtual ~QuadBase(){}
	virtual void reset() = 0;
	virtual void init(int,int PINSET=1) = 0;
	virtual void init(int PINSET=1) = 0;
	virtual void begin() = 0;
	virtual void stop() = 0;
	virtual void setMod(int) = 0;
	virtual int getMod() = 0;
	virtual void setCNTIN(int) = 0;
	virtual void setFilter(unsigned int) = 0;
	virtual void setPreScale(unsigned int) = 0;
	virtual void invertCHA() = 0;
	virtual void invertCHB() = 0;
	virtual int getCount() = 0;
	virtual void incREV(bool) = 0;
	virtual int getRev() = 0;	
	virtual int getRegister(reggister) = 0;
};

template <int N> class QuadDecoder;
// Pointers to  correct instance for ISR
extern QuadDecoder<1> *apQDcd1;	    
extern QuadDecoder<2> *apQDcd2;

// Derived template class for the 2 quadrature decoders
template <int N>    // Valid values are 1 or 2 to set up as FTM1 or FTM2
class QuadDecoder : public QuadBase
{
public:
	QuadDecoder()
	{
	    // Order of contstructor execution not guaranteed, 
	    //   start with start()
	    if (N<2)
		{	// Point to this instance for ISR
			apQDcd1=reinterpret_cast<QuadDecoder<1>*>(this);
		// Without reintepret_cast compiler complains about unused 
		//   case.  Pointer for unused case not used, so don't
		//   care what it is.  Used case casts to existing type.
	    }else
		{
			apQDcd2=reinterpret_cast<QuadDecoder<2>*>(this);
		};
	    revs = 0;
	};
	void reset();
	void init(int,int PINSET=1);
	void init(int PINSET=1);
	void begin();
	void stop();
	void setMod(int);
	int getMod();
	void setCNTIN(int);
	void setFilter(unsigned int);
	void setPreScale(unsigned int);
	void invertCHA();
	void invertCHB();
	int getCount();
	void incREV(bool);
	int getRev();
	int getRegister(reggister);

	void ftm_isr();
private:
	void checkProtection();
	void enProtection();
	int revs;		// revolution based off modulus
	// register variables so that register can be called generically 
	const reg _FTM_CNTIN = ((N<2) ? &FTM1_CNTIN : &FTM2_CNTIN);
	const reg _FTM_MOD  = ((N<2) ? &FTM1_MOD : &FTM2_MOD);
	const reg _FTM_SC = ((N<2) ? &FTM1_SC : &FTM2_SC);
	const reg _FTM_QDCTRL = ((N<2) ? &FTM1_QDCTRL : &FTM2_QDCTRL);
	const reg _FTM_MODE = ((N<2) ? &FTM1_MODE : &FTM2_MODE);
	const reg _FTM_CNT = ((N<2) ? &FTM1_CNT : &FTM2_CNT);
	const reg _FTM_FILTER = ((N<2) ? &FTM1_FILTER : &FTM2_FILTER);
	const reg _FTM_FMS = ((N<2) ? &FTM1_FMS : &FTM2_FMS);
	const reg _FTM_FLTCTRL = ((N<2) ? &FTM1_FLTCTRL : &FTM2_FLTCTRL);
	uint8_t _IRQ_FTM = ((N<2) ? IRQ_FTM1 : IRQ_FTM2);
};

// Interrupt service rutine
template <int N>
void QuadDecoder<N>::ftm_isr(void)
{
	*_FTM_SC &= 0x19;
	((*_FTM_QDCTRL & FTM_QDCTRL_TOFDIR)==2) ? incREV(true) : incREV(false);
	*_FTM_SC |= FTM_SC_TOIE;
}
// get the register value mostly needed for debug
template <int N>
int QuadDecoder<N>::getRegister(reggister name)
{
	switch(name)
	{
	case CNTIN:
		return *_FTM_CNTIN;
		break;
	case MOD:
		return *_FTM_MOD;
		break;
	case SC:
		return *_FTM_SC;
		break;
	case QDCTRL:
		return *_FTM_QDCTRL;
		break;
	case MODE:
		return *_FTM_MODE;
		break;
	case CNT:
		return *_FTM_CNT;
		break;
	case FILTER:
		return *_FTM_FILTER;
		break;
	case FMS:
		return *_FTM_FMS;
		break;
	case FLTCTRL:
		return *_FTM_FLTCTRL;
		break;
	case IRQ:
		return _IRQ_FTM;
		break;
  default:
    return 0;
    break;
	}
}

// Setup Registers
template <int N>
void QuadDecoder<N>::init(int MOD, int PINSET)
{
	*_FTM_SC &= 0x00;
	*_FTM_CNT = 0;
	*_FTM_MOD = MOD;
	
	// enable the clock FTM
	(N<2) ? SIM_SCGC3 |= SIM_SCGC3_FTM2 : SIM_SCGC6 |= SIM_SCGC6_FTM1;
	

	// CHECK WRITE PROTECTION
	checkProtection();
	// FTM mode enable
	*_FTM_MODE |= FTM_MODE_FTMEN;
	// Quad enable
	*_FTM_QDCTRL |= FTM_QDCTRL_QUADEN;
	// set clk as external & set prescale
	*_FTM_SC |= FTM_SC_CLKS(3);
	*_FTM_SC |= FTM_SC_PS(0);

	// Multiplex PINS
	if(PINSET == 0 && N == 1)
	{
		PORTB_PCR0 = PORT_PCR_MUX(6); // FTM1 CH0 PHASE A Teensy PIN 16/A2
		PORTB_PCR1 = PORT_PCR_MUX(6); // FTM1 CH1 PHASE B Teensy PIN 17/A3
	}
	else if (N == 1)
	{
		PORTA_PCR12 = PORT_PCR_MUX(7); // FTM1 CH0 PHASE A Teensy PIN 3
		PORTA_PCR13 = PORT_PCR_MUX(7); // FTM1 CH1 PHASE B Teensy PIN 4
	}
	else
	{
		PORTB_PCR18 = PORT_PCR_MUX(6); // FTM2 CH0 PHASE A Teensy PIN 32
		PORTB_PCR19 = PORT_PCR_MUX(6); // FTM2 CH1 PHASE B Teensy PIN 25
	}
	// Enable write protection
	//enProtection();
	*_FTM_FMS |= FTM_FMS_WPEN;
}

// same initialization but modulus not defined yet
template <int N>
void QuadDecoder<N>::init(int PINSET)
{	
	*_FTM_SC &= 0x00;
	*_FTM_CNT = 0;
	
	// enable the clock FTM
	(N<2) ? SIM_SCGC3 |= SIM_SCGC3_FTM2 : SIM_SCGC6 |= SIM_SCGC6_FTM1;
	
	// CHECK WRITE PROTECTION
	checkProtection();
	// FTM mode enable
	*_FTM_MODE |= FTM_MODE_FTMEN;
	// Quad enable
	*_FTM_QDCTRL |= FTM_QDCTRL_QUADEN;
	// set clk as external & set prescale
	*_FTM_SC |= FTM_SC_CLKS(3);
	*_FTM_SC |= FTM_SC_PS(2);

	// Multiplex PINS
	if(PINSET == 0 && N <2)
	{
		PORTB_PCR0 = PORT_PCR_MUX(6); // FTM1 CH0 PHASE A Teensy PIN 16/A2
		PORTB_PCR1 = PORT_PCR_MUX(6); // FTM1 CH1 PHASE B Teensy PIN 17/A3
	}
	else if (N<2)
	{
		PORTA_PCR12 = PORT_PCR_MUX(7); // FTM1 CH0 PHASE A Teensy PIN 3
		PORTA_PCR13 = PORT_PCR_MUX(7); // FTM1 CH1 PHASE B Teensy PIN 4
	}
	else
	{
		PORTB_PCR18 = PORT_PCR_MUX(6); // FTM2 CH0 PHASE A Teensy PIN 32
		PORTB_PCR19 = PORT_PCR_MUX(6); // FTM2 CH1 PHASE B Teensy PIN 25
	}
	// Enable write protection
	//enProtection();
	*_FTM_FMS |= FTM_FMS_WPEN;
}

// Function to handle inverting channel A
template <int N>
void QuadDecoder<N>::invertCHA()
{
	// determine channel is inverted
	*_FTM_QDCTRL |= FTM_QDCTRL_PHAPOL;
}

// Function to handle inverting channel B
template <int N>
void QuadDecoder<N>::invertCHB()
{
	// determine channel is inverted

	*_FTM_QDCTRL |= FTM_QDCTRL_PHBPOL;
}

// CHECK WRITE PROTECTION
template <int N>
void QuadDecoder<N>::checkProtection()
{
	if((*_FTM_FMS&FTM_FMS_WPEN) == 1)
	{
		*_FTM_MODE |= FTM_MODE_WPDIS;
	}
}

// Enable write protection
template <int N>
void QuadDecoder<N>::enProtection()
{
	*_FTM_FMS |= FTM_FMS_WPEN;
}

// Set mod value
template <int N>
void QuadDecoder<N>::setMod(int modNum)
{
	// CHECK WRITE PROTECTION
	//checkProtection();
	*_FTM_CNT = 0;
	*_FTM_MOD = modNum;
	// Enable write protection
	//enProtection();
}

// set Filter value valid rang is 0-15
template <int N>
void QuadDecoder<N>::setFilter(unsigned int filtNum)
{
	*_FTM_FILTER |= FTM_FILTER_CH0FVAL(filtNum);
	*_FTM_FILTER |= FTM_FILTER_CH1FVAL(filtNum);
	*_FTM_QDCTRL |= FTM_QDCTRL_PHAFLTREN |FTM_QDCTRL_PHBFLTREN;
}

// get the number of counts 
template <int N>
int QuadDecoder<N>::getCount()
{
	return (*_FTM_CNT + revs*getMod());
}

// Reset variables and clears count register
template <int N>
void QuadDecoder<N>::reset()
{
	revs = 0;
	*_FTM_CNT = 0;
}
// start decoders
template <int N>
void QuadDecoder<N>::begin()
{
	NVIC_ENABLE_IRQ(_IRQ_FTM);
	*_FTM_SC |= FTM_SC_TOIE;
}
// stop decoders
template <int N>
void QuadDecoder<N>::stop()
{
	NVIC_DISABLE_IRQ(_IRQ_FTM);
	*_FTM_SC &= !FTM_SC_TOIE;
}
// increment the revolutions
template <int N>
void QuadDecoder<N>::incREV(bool increase)
{
	increase ? revs++ : revs--;
}
// set the count value
template <int N>
void QuadDecoder<N>::setCNTIN(int cnt)
{
	*_FTM_CNTIN = cnt;
}
// set the prescale
template <int N>
void QuadDecoder<N>::setPreScale(unsigned int num)
{
	if((*_FTM_FMS&FTM_FMS_WPEN) == 1)
	{
		*_FTM_FMS |= FTM_FMS_WPEN;
	}
	*_FTM_SC |= FTM_SC_PS(num);
	*_FTM_FMS |= FTM_FMS_WPEN;
}

// get the number of revolutions 
template <int N>
int QuadDecoder<N>::getRev()
{
	return revs;
}
// returns the modulus of the decoder
template <int N>
int QuadDecoder<N>::getMod()
{
	return *_FTM_MOD;
}

#endif
