#include "Safety_Switch.h"

short Set_Mode_And_Safety_Switch(void)  // Mode will be returned
{
	int i;
	uint32_t last_record_time = currenttime();
	short return_value;
	while(Software_Enabled())
	{
		return_value = 0;
		if(SAFE_SWITCH_PIN != ACTIVATE_VOLTAGE)   //Low voltage means activate
		{
			last_record_time = currenttime();
		}
		if(BIT0_SWITCH_PIN == ACTIVATE_VOLTAGE)
		{
			BIT0_INDICATOR = 1U;
			return_value += 0x01;
		}
		else
			BIT0_INDICATOR = 0U;
		if(BIT1_SWITCH_PIN == ACTIVATE_VOLTAGE)
		{
			BIT1_INDICATOR = 1U;
			return_value += 0x02;
		}
		else
			BIT1_INDICATOR = 0U;
		if(return_value == 1)
		{
			Mission_Two();
		}
		if(currenttime() - last_record_time > 1000)
		{
			/* 11 Seconds in total */
			/*for(i = 0;i < 2;i++)
			{
				BUZZER_PIN = 1U;
				delay_ms(500);
				BUZZER_PIN = 0U;
				delay_ms(500);
			}
			for(i = 0;i < 4;i++)
			{
				BUZZER_PIN = 1U;
				delay_ms(250);
				BUZZER_PIN = 0U;
				delay_ms(250);
			}
			for(i = 0;i < 8;i++)
			{
				BUZZER_PIN = 1U;
				delay_ms(125);
				BUZZER_PIN = 0U;
				delay_ms(125);
			}
			for(i = 0;i < 16;i++)
			{
				BUZZER_PIN = 1U;
				delay_ms(62);
				BUZZER_PIN = 0U;
				delay_ms(62);
			}
			for(i = 0;i < 32;i++)
			{
				BUZZER_PIN = 1U;
				delay_ms(31);
				BUZZER_PIN = 0U;
				delay_ms(31);
			}*/
			float delay_interval = 500;
#ifdef SCPATHTEXT
			my_printf("Mode %d Selected\n", return_value);
#endif
			DISTANCE_INDICATOR = 0U;
			for(i = 0;i < 2;i++)
			{
				BUZZER_PIN = 1U;
				delay_ms(60);
				BUZZER_PIN = 0U;
				delay_ms(60);
			}
			delay_ms(500);
			for(i = 0;i < return_value;i++)
			{
				BUZZER_PIN = 1U;
				delay_ms(500);
				BUZZER_PIN = 0U;
				delay_ms(500);
			}
			if(return_value == 0)
				delay_ms(500);

			for(i = 0;i < 2;i++)
			{
				BUZZER_PIN = 1U;
				delay_ms(60);
				BUZZER_PIN = 0U;
				delay_ms(60);
			}
			delay_ms(1000);
			last_record_time = currenttime();
			while(currenttime() - last_record_time < 5000)
			{
				BUZZER_PIN = 1U;
				delay_ms(max((int)delay_interval,40));
				BUZZER_PIN = 0U;
				delay_ms(max((int)delay_interval,40));
				delay_interval /= 1.28f;
			}
			BUZZER_PIN = 0U;
			delay_ms(1000);
			/*for(i = 0;i < 3;i++)
			{
				BUZZER_PIN = 1U;
				delay_ms(100);
				BUZZER_PIN = 0U;
				delay_ms(100);
			}*/
			BIT0_INDICATOR = 0U;
			BIT1_INDICATOR = 0U;
			break;
		}
	}
	return return_value;
}
