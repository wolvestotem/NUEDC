#include "Bluetooth_Decoder.h"

static struct Node * DataNode;
static struct Node Circle[5];
static short Land_Enabled = 0;

static const uint8_t * QueueToBeChecked[2] = {"74159", "52684"};

void Decoder_Init(void)
{
	int i;
	for(i = 0;i < 4;i++)
		Circle[i].next = &Circle[i+1];
	Circle[4].next = &Circle[0];

	for(i = 0;i < 5;i++)
		Circle[i].dat = 0;

	DataNode = &Circle[0];
}

static void Data_Check(void)
{
	int i, j;
	for(i = 0;i < 2;i++)
	{
		int trans_token = 20;
		struct Node * tmp = DataNode -> next;
		for(j = 0;j < 5;j++)
		{
			if(tmp->dat == QueueToBeChecked[i][j])
			{
				tmp = tmp->next;
				if(j == 4)
				{
					trans_token = i;
				}
			}
			else
				break;
			switch(trans_token)
			{
			case 0:
				Disable_SoftWare();
				break;
			case 1:
				Land_Enabled = 1;
#ifdef SCCOPTERTEXT
				BIT1_INDICATOR = ~BIT1_INDICATOR;
				my_printf("Land Enabled\n");
#endif
				break;
			}
		}
	}
}

void Decoder_Recv(uint8_t _dat)
{
	DataNode->dat = _dat;

	Data_Check();
	DataNode = DataNode->next;
}

void Land_Reset(void)
{
#ifdef SCCOPTERTEXT
	my_printf("Land Reset\n");
#endif
	Land_Enabled = 0;
}

short Decode_Land(void)
{
	return Land_Enabled;
}
