#include "kinetis.h"
#include "QuadDecoder.h"
// Pointers to  correct instance for ISR
QuadDecoder<1> *apQDcd1;	    
QuadDecoder<2> *apQDcd2;
void ftm1_isr(void) 
{
	apQDcd1->ftm_isr();
}

void ftm2_isr(void) 
{
	apQDcd2->ftm_isr();
}