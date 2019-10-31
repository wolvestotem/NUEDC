#include "Mavlink_Head.h"

void abort(void)
{

}

float get_Copter_Height(void)
{
	float *temp = get_height();
	return *temp;
}

void Copter_PreArm(void)
{
#ifdef SCCOPTERTEXT
	my_printf("Sending PreArm\n");
#endif
	prearm();
}

void Wait_For_Take_Off(void)
{
	READY_TO_TAKEOFF_LED = 0U;
	while(takeoff_check() != 1 && Software_Enabled())
	{
		Copter_PreArm();
		delay_ms(6000);
	}
	READY_TO_TAKEOFF_LED = 1U;
	BUZZER_PIN = 1U;
	delay_ms(500);
	BUZZER_PIN = 0U;
	delay_ms(500);
	BUZZER_PIN = 1U;
	delay_ms(1500);
	BUZZER_PIN = 0U;
#ifdef SCCOPTERTEXT
	my_printf("Ready to takeoff\n");
#endif
}

void Take_Off(void)
{
	while(Software_Enabled())
	{
		READY_TO_TAKEOFF_LED = ~ READY_TO_TAKEOFF_LED;
		if(arm())
		{
			int i;
			//delay_ms(500);
			for(i = 0;i < 40;i++)
			{
				BUZZER_PIN = 1U;
				delay_ms(66);
				BUZZER_PIN = 0U;
				delay_ms(66);
			}

			if(mav_takeoff(get_Copter_Height() + COPTER_HEIGHT + 0.2f))
			{
#ifdef SCCOPTERTEXT
				my_printf("TakeOff!\n");
#endif
				delay_ms(1000);
				Sc_Set_SoftWare_Velocity(0.0f, 0.0f, 0.0f);

				while(get_Copter_Height() < 0.4f && Software_Enabled());

#ifdef SCCOPTERTEXT
				my_printf("TakeOff Complete!\n");
#endif
				break;
			}
			else
			{
#ifdef SCCOPTERTEXT
				my_printf("Takeoff Failed\n");
#endif
				delay_ms(3000);
			}
		}
		else
		{
#ifdef SCCOPTERTEXT
			my_printf("Arm Failed\n");
#endif
			delay_ms(3000);
		}
	}
	READY_TO_TAKEOFF_LED = 0U;
}

void Copter_Land(void)
{
#ifdef SCCOPTERTEXT
	my_printf("Landing\n");
#endif
	mav_land();
}

short Copter_New_Land(float x, float y)
{
	new_land(y,x,0.0f,30.0f,50.0f,0.0f);
	if(get_Copter_Height() <= 0.3f)
	{
#ifdef SCCOPTERTEXT
		my_printf("Height = %f. Land\n", get_Copter_Height());
#endif
		return !COPTER_LAND_CONTROL_ENABLED;
	}
	return COPTER_LAND_CONTROL_ENABLED;
}

short Copter_New_Land_Optional(float x, float y, float _des_height, float _des_climbrate)
{
	new_land(y,x,0.0f,_des_height,_des_climbrate,0.0f);
	if(get_Copter_Height() <= _des_height / 100.0f)
	{
#ifdef SCCOPTERTEXT
		my_printf("Height = %f. Land\n", get_Copter_Height());
#endif
		return !COPTER_LAND_CONTROL_ENABLED;
	}
	return COPTER_LAND_CONTROL_ENABLED;
}

void Copter_Level_Calibration(void)  // block
{
	int i;
	for(i = 0;i <= 10;i++)
	{
		BIT0_INDICATOR = 1U;
		BIT1_INDICATOR = 0U;
		delay_ms(500);
		BIT0_INDICATOR = 0U;
		BIT1_INDICATOR = 1U;
		delay_ms(500);
	}
	gyro_calibrate();
	BIT0_INDICATOR = 0U;
	BIT1_INDICATOR = 0U;

	delay_ms(10000);
	BIT0_INDICATOR = 1U;
	BIT1_INDICATOR = 1U;
	BUZZER_PIN = 1U;
	delay_ms(2000);
	BUZZER_PIN = 0U;

	while(Software_Enabled());
}

void MavLink_Start(void)
{
	init(&SCI1_Serial_Send,&delay_ms,&currenttime,&mavlink_printf);
}

void Copter_Add_Yaw(float _add)  //positive when clockwise
{
#ifdef SCCOPTERTEXT
	my_printf("Add yaw: %.4f", _add);
#endif
	set_yaw(_add);
}
/*      DEBUG CODE      */
/*uint8_t recv[1];
if(sci5_receive_available())
{
	SCI5_Serial_Receive(recv,1);
	switch(recv[0])
	{
	case '0':
		Bluetooth_Writestring("\nSc:0 received\n");
		if(armcheck())
		{
			delay_ms(1000);
			mav_takeoff(1.0f);
		}
		break;
	case '1':
		Bluetooth_Writestring("\nSc:1 received\n");
		set_vel(0.2,0,0);
		delay_ms(1500);
		set_vel(0,0,0);
		break;
	case '2':
		Bluetooth_Writestring("\nSc:2 received\n");
		set_vel(0,0.2,0);
		delay_ms(1500);
		set_vel(0,0,0);
		break;
	case '3':
		Bluetooth_Writestring("\nSc:3 received\n");
		set_vel(0,0,0.2);
		delay_ms(1500);
		set_vel(0,0,0);
		break;
	case '4':
		Bluetooth_Writestring("\nSc:4 received\n");
		set_vel(0,0,0);
		break;
	case '5':
		Bluetooth_Writestring("\nSc:5 received\n");
		mav_land();
		break;
	default: my_printf("Command %c unresolvable",recv[0]); break;
	}
}*/

