
#ifndef BLUETOOTH_DECODER_BLUETOOTH_DECODER_H_
#define BLUETOOTH_DECODER_BLUETOOTH_DECODER_H_

#include "../stdafx.h"

struct Node{
	uint8_t dat;
	struct Node * next;
};

void Decoder_Init(void);
void Decoder_Recv(uint8_t _dat);
void Land_Reset(void);
short Decode_Land(void);

#endif /* BLUETOOTH_DECODER_BLUETOOTH_DECODER_H_ */
