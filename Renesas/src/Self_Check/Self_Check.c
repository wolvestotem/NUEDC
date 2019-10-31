#include "Self_Check.h"

short self_checked_token = 0;

void Follow_Car_Self_Check(void)
{
	int i;
//	SELF_CHECKED_LED = 0U;
#ifdef SCSELFCHECKTEXT
	my_printf("Renesas:start self check\n");
#endif
	for(i = 0;i<=200;i++)
	{
		if(Electric_Match_SPI_Decode() == OPENMV_DECODE_SUCCESS)
		{
			self_checked_token = 1;
#ifdef SCSELFCHECKTEXT
			my_printf("Renesas:check OK!\n");
#endif
			BUZZER_PIN = 1U;
			delay_ms(1000);
			BUZZER_PIN = 0U;
//			SELF_CHECKED_LED = 1U;
			break;
		}
	}
	if(self_checked_token != 1)
	{
#ifdef SCSELFCHECKTEXT
		my_printf("Renesas:check failed\n");
#endif
		while(Software_Enabled())
		{
			BUZZER_PIN = 1U;
			delay_ms(100);
			BUZZER_PIN = 0U;
			delay_ms(100);
		}
	}
}

void OpenMV_Assert(void)
{
	while(Electric_Match_SPI_Decode() != OPENMV_DECODE_SUCCESS && Software_Enabled());

	if(EMD.blackAvailable)
	{
		BIT0_INDICATOR = 1U;
	}
	else
	{
		BIT0_INDICATOR = 0U;
	}
	if(EMD.carAvailable)
	{
		BIT1_INDICATOR = 1U;
	}
	else
	{
		BIT1_INDICATOR = 0U;
	}
	if(currenttime() % 1000 <= 100)
	{
		BUZZER_PIN = 1U;
	}
	else
	{
		BUZZER_PIN = 0U;
	}
}
