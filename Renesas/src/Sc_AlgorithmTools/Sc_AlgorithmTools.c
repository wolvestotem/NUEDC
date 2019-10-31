#include "Sc_AlgorithmTools.h"

void Sc_Trigger(short (*_Fun) (void))
{
	while(_Fun() != 1);
}

void Sc_TimeHolder(void (*_Fun) (void), uint32_t _milliseconds)
{
	uint32_t entrytime = currenttime();
	while(currenttime() - entrytime < _milliseconds && Software_Enabled())
		_Fun();
}

void Sc_Counter(short (*_Fun) (void), uint16_t desired_count)
{

}
