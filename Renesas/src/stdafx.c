#include "stdafx.h"

static short Global_Start_Token = 1;

short Software_Enabled(void)
{
	return Global_Start_Token;
}

void Enable_Software(void)
{
	Global_Start_Token = 1;
	my_printf("Software Enabled\n");
}

void Disable_SoftWare(void)
{
	Global_Start_Token = 0;
	my_printf("Software Disabled\n");
}
