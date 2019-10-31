#include "PathSeeking.h"
/*
float last_x = 0.0f, last_y = 0.0f;

struct PID_Param xFollowCar = {0.004f,0.0f,0.60f,0.0f,0.0f,0.0f,0.0f};
struct PID_Param yFollowCar = {0.004f,0.0f,0.60f,0.0f,0.0f,0.0f,0.0f};

short OpenMV_Trans_Token;
uint32_t highspeedtime;

void follow_car(void)
{
	highspeedtime = currenttime();
	while(Software_Enabled())
	{
		float entrytime = currenttime();
		OpenMV_Trans_Token = OPENMV_RESET;
		my_printf("%ld\n",currenttime());
		while(currenttime() - entrytime < GETDATATIME)   //loop time = GETDATATIME(100ms)  !RETHINK HERE!
		{
			if(OpenMV_Trans_Token != OPENMV_DECODE_SUCCESS)
				OpenMV_Trans_Token = Follow_Car_SPI_Decode();
			else
				break;
			//if(currenttime() - entrytime < GETDATATIME - OPENMVTIMEOUT - 1)
			//	Follow_Car_SPI_Decode();
		}
		if(OpenMV_Trans_Token != OPENMV_DECODE_SUCCESS)   //lost car or spi failed
		{
			last_x /= DAMPINGRATE;
			last_y /= DAMPINGRATE;
			my_printf("lostcar\n");
			Sc_Set_SoftWare_Velocity(last_x, last_y, 0.0f);
			//continue;
		}
		else
		{
			float xControl, yControl;
			xControl = Sc_PIDWithPosition(0.0f, - nav_dat.x, &xFollowCar);
			yControl = Sc_PIDWithPosition(0.0f, - nav_dat.y, &yFollowCar);

			last_x = xControl;
			last_y = yControl;

			my_printf("found\n");
			Sc_Set_SoftWare_Velocity(xControl, yControl, 0.0f);
		}
		if(fabs(last_x) < LOW_SPEED_THRESHOLD && fabs(last_y) < LOW_SPEED_THRESHOLD)
		{
			BUZZER_PIN = 1U;
			if(currenttime() - highspeedtime > LOW_SPEED_TIMEOUT)
			{
				int i;
				for(i = 0;i < 4;i++)
				{
					BUZZER_PIN = 1U;
					delay_ms(100);
					BUZZER_PIN = 0U;
					delay_ms(100);
				}
				my_printf("land\n");
				Copter_New_Land(0.0f,0.0f);
				break;
			}
		}
		else
		{
			highspeedtime = currenttime();
			BUZZER_PIN = 0U;
		}
	}
}*/

//****************************************************************************
///* Tools *///

struct PID_Param Xblack_param = {0.0045f,0.00007f,0.50f,0.05f,0.0f,0.0f,0.0f};
struct PID_Param Yblack_param = {0.0045f,0.00007f,0.50f,0.05f,0.0f,0.0f,0.0f};
struct PID_Param Xcar_param = {0.0045f,0.00007f,0.48f,0.05f,0.0f,0.0f,0.0f};
struct PID_Param Ycar_param = {0.0045f,0.00007f,0.48f,0.05f,0.0f,0.0f,0.0f};
struct PID_Param Xfollow_param = {0.0045f,0.00007f,0.52f,0.05f,0.0f,0.0f,0.0f};
struct PID_Param Yfollow_param = {0.0045f,0.00007f,0.52f,0.05f,0.0f,0.0f,0.0f};
#ifndef SCB2C
struct PID_Param B2Cx_param = {0.0042f,0.0f,0.525f,0.0f,0.0f,0.0f,0.0f};
struct PID_Param B2Cy_param = {0.0042f,0.0f,0.525f,0.0f,0.0f,0.0f,0.0f};
#endif

static void DistanceIndicatorRefresh(void)  // unfinished
{
	const uint8_t startsignal[5] = {0xF0,0xF1,0xF2,0xF3,0xF4};
	//const uint8_t stopsignal;
	float height,grounderr,distance;
	height = get_Copter_Height();

	if(EMD.carAvailable)
	{
		grounderr = height * powf(sqrtf(EMD.Xcar * EMD.Xcar + EMD.Ycar * EMD.Ycar), 0.909f) * 1.148f / 100.0f;  //1 pixel = 1.265cm

		distance = sqrtf(height * height + grounderr * grounderr);

#ifdef SCDISTANCETEXT
		my_printf("distance = %.4f\n", distance);
#endif
		if(distance <= 1.5f && distance >= 0.5f)
		{
			DISTANCE_INDICATOR = 1U;
			SCI5_Serial_Send(startsignal, 5);
		}
		else
		{
			DISTANCE_INDICATOR = 0U;
		}
	}
	else
	{
		DISTANCE_INDICATOR = 0U;
	}
}

static void Get_OpenMV_Data(void)
{
#ifdef SCSPIDATAASSERT
	short OpenMV_Trans_Token = OPENMV_RESET;
	float entrytime = currenttime();
	while(currenttime() - entrytime < GETDATAASSERTTIME)
	{
		if(OpenMV_Trans_Token != OPENMV_DECODE_SUCCESS)
			OpenMV_Trans_Token = Electric_Match_SPI_Decode();
		if(currenttime() - entrytime >= GETDATATIME)
			break;
		//else
		//	Electric_Match_SPI_Decode();
	}
	if(OpenMV_Trans_Token != OPENMV_DECODE_SUCCESS)
	{
		my_printf("SPI fetch asserted! case: %d\n",OpenMV_Trans_Token);
		return;
	}
	#ifdef ALWAYSFRESHDISTANCEINDICATOR
	DistanceIndicatorRefresh();
	#endif
#else
	while(Electric_Match_SPI_Decode() != OPENMV_DECODE_SUCCESS && Software_Enabled());
#endif
}

// used in mission 1 (no block)
static void PID_to_BlackPoint(void)
{
	float xControl, yControl;

	Get_OpenMV_Data();

#ifdef SCPATHTEXT
		my_printf("X:\n");
#endif
	xControl = Sc_PIDWithPosition(0.0f, - EMD.Xblack, &Xblack_param);
#ifdef SCPATHTEXT
		my_printf("Y:\n");
#endif
	yControl = Sc_PIDWithPosition(0.0f, - EMD.Yblack, &Yblack_param);

#ifdef STOPWHENNOTFOUND
	if(EMD.blackAvailable)
	{
		Sc_Set_SoftWare_Velocity(xControl, yControl, 1.1f - COPTER_HEIGHT);
	}
	else
	{
	#ifdef SCPATHTEXT
		my_printf("Black Not Found\n");
	#endif
		Sc_Set_SoftWare_Velocity(0.0f, 0.0f, 1.1f - COPTER_HEIGHT);
	}
#else
	Sc_Set_SoftWare_Velocity(xControl, yControl, 1.1f - COPTER_HEIGHT);
	if(!EMD.blackAvailable)
	{
	#ifdef SCPATHTEXT
		my_printf("Black Not Found\n");
	#endif
	}
#endif
/*
	#ifdef SCPATHTEXT
		my_printf("\n");
	#endif
*/
}

// used in mission 3 (no block)
static void PID_to_Car(void)
{
	float xControl, yControl;

	Get_OpenMV_Data();

	xControl = Sc_PIDWithPosition(0.0f, - EMD.Xcar, &Xcar_param);
	yControl = Sc_PIDWithPosition(0.0f, - EMD.Ycar, &Ycar_param);

#ifdef STOPWHENNOTFOUND
	if(EMD.carAvailable)
	{
		Sc_Set_SoftWare_Velocity(xControl, yControl, 0.0f);
	}
	else
	{
	#ifdef SCPATHTEXT
		my_printf("Car Not Found\n");
	#endif
		Sc_Set_SoftWare_Velocity(0.0f, 0.0f, 0.0f);
	}
#else
	Sc_Set_SoftWare_Velocity(xControl, yControl, 0.0f);
#endif
}

// stablize before seeking cars (no block)
static void Takeoff_Stablize(void)
{
	float xControl, yControl;

	Get_OpenMV_Data();

	xControl = Sc_PIDWithPosition(0.0f, - EMD.Xblack, &Xblack_param);
	yControl = Sc_PIDWithPosition(0.0f, - EMD.Yblack, &Yblack_param);

#ifdef STOPWHENNOTFOUND
	if(EMD.blackAvailable)
	{
		Sc_Set_SoftWare_Velocity(xControl, yControl, 0.0f);
	}
	else
	{
	#ifdef SCPATHTEXT
		my_printf("Black Not Found\n");
	#endif
		Sc_Set_SoftWare_Velocity(0.0f, 0.0f, 0.0f);
	}
#else
	Sc_Set_SoftWare_Velocity(xControl, yControl, 0.0f);
#endif
}

// used by mission 1 (block)
static void Land_to_BlackPoint_with_PID(void)
{
	float xControl, yControl;
	while(Software_Enabled())
	{
		Get_OpenMV_Data();

		xControl = Sc_PIDWithPosition(0.0f, - EMD.Xblack, &Xblack_param);
		yControl = Sc_PIDWithPosition(0.0f, - EMD.Yblack, &Yblack_param);

		if(!Copter_New_Land_Optional(xControl, yControl, 30.0f, 50.0f))
			break;
	}
}

// land evading car (block)
static void Land_Evading_Car(void)
{
	float xControl, yControl;
	short Land_Token = 0;
	float Evade_PID_Target = 0.0f;
	float Time_add_Interval = currenttime();
	while(Software_Enabled())
	{
		Get_OpenMV_Data();

		if(currenttime() - Time_add_Interval > 100)
		{
			Time_add_Interval = currenttime();
			Evade_PID_Target = (Evade_PID_Target <= - 44.0f) ? - 45.0f : Evade_PID_Target - 5.0f;
		}

		xControl = Sc_PIDWithPosition(0.0f, - EMD.Xcar, &Xcar_param);
		yControl = Sc_PIDWithPosition(Evade_PID_Target, - EMD.Ycar, &Ycar_param);

		Sc_Set_SoftWare_Velocity(xControl, yControl, 0.0f);

		if(fabs(EMD.Xcar) <= 5 && fabs(EMD.Ycar - 45.0f) <= 5)
		{
			Land_Token++;
			if(Land_Token >= 4)
			{
#ifdef SCPATHTEXT
				my_printf("Landing\n");
#endif
				Copter_New_Land(0.0f,0.0f);
				break;
			}
		}
		if(!EMD.carAvailable)
		{
#ifdef SCPATHTEXT
			my_printf("Lost Car, Landing\n");
#endif
			Copter_New_Land(0.0f,0.0f);
			break;
		}
	}
}

// seeking car (block)
/*static void From_BlackPoint_to_Car(void)
{
	int Car_Found_Token = 0;
	float Seek_Adjust = 0.0f;
	uint32_t Last_Height_Adjust_Time = 0;
	float xControl, yControl;
	while(Software_Enabled())
	{
		Get_OpenMV_Data();
		if(!EMD.carAvailable)  // car not found
		{
			if(EMD.blackAvailable)  // have black
			{
				if(fabs(EMD.Yblack) > 40.0f && currenttime() - Last_Height_Adjust_Time > HEIGHT_ADJUST_LEAST_INTERVAL)
				{
					Last_Height_Adjust_Time = currenttime();
					Seek_Adjust = (Seek_Adjust >= 0.5f) ? 0.5f : Seek_Adjust+0.1f;
					if(Seek_Adjust >= 0.5f)
					{
#ifdef SCPATHTEXT
						my_printf("Reaching Seek Adjust Limit\n");
#endif
					}
				}

				xControl = Sc_PIDWithPosition(0.0f, - EMD.Xblack, &Xblack_param);
				yControl = Sc_PIDWithPosition(50.0f, - EMD.Yblack, &Yblack_param);

				Sc_Set_SoftWare_Velocity(xControl, yControl, 0.0f + Seek_Adjust);
			}
#ifdef SCPATHTEXT
			else
			{
				my_printf("BlackPoint and Car Not Found\n");
				Sc_Set_SoftWare_Velocity(0.0f, 0.0f, 0.0f + Seek_Adjust);
			}
#endif
		}
		else  //found car
		{
			xControl = Sc_PIDWithPosition(0.0f, - EMD.Xcar, &Xcar_param);
			yControl = Sc_PIDWithPosition(0.0f, - EMD.Ycar, &Ycar_param);

			Sc_Set_SoftWare_Velocity(xControl, yControl, 0.0f + Seek_Adjust);

			if(fabs(EMD.Xcar) <= CAR_AREA_THRESHOLD && fabs(EMD.Ycar) <= CAR_AREA_THRESHOLD)
			{
				if(currenttime() - Last_Height_Adjust_Time > HEIGHT_ADJUST_LEAST_INTERVAL)
				{
					Last_Height_Adjust_Time = currenttime();
					Seek_Adjust = (Seek_Adjust <= 0.0f) ? 0.0f : Seek_Adjust-0.1f;
				}
				if(Seek_Adjust == 0.0f)
					Car_Found_Token++;
			}
			if(Car_Found_Token >= 5)
				break;
		}
	}
}*/
#ifdef SCB2C
static void From_BlackPoint_to_Car(void)
{
	short Car_Found_Token = 0;
	short Delay_init_Token = 0;
	short Car_Reservation = 0;
	float Seek_Adjust = 0.2f;
	uint32_t Last_Height_Adjust_Time = 0;
	float xControl, yControl;
	float Last_X = 0.0f, Last_Y = 0.0f;
	float Car_PID_Target = -50.0f, Black_PID_Target = 0.0f;
	uint32_t Time_add_Interval = currenttime();
	while(Software_Enabled())
	{
		Get_OpenMV_Data();
		if(Delay_init_Token >= 2 && !EMD.carAvailable)
		{
#ifdef SCPATHTEXT
			my_printf("Using Delay\n");
#endif
			Sc_Set_SoftWare_Velocity_With_Speed_Limit(0.0f, 0.15f, 0.0f + Seek_Adjust);
		}
		else if(!EMD.carAvailable && !Car_Reservation)  // car not found
		{
			if(EMD.blackAvailable)  // have black
			{
				if(fabs(EMD.Yblack) > 40.0f && currenttime() - Last_Height_Adjust_Time > HEIGHT_ADJUST_LEAST_INTERVAL)
				{
					Last_Height_Adjust_Time = currenttime();
					Seek_Adjust = (Seek_Adjust >= 0.39f) ? 0.4f : Seek_Adjust + 0.1f;
					if(Seek_Adjust >= 0.39f)
					{
						Delay_init_Token++;
#ifdef SCPATHTEXT
						my_printf("Reaching Seek Adjust Limit\n");
#endif
					}
				}

				if(currenttime() - Time_add_Interval > 20)
				{
					Time_add_Interval = currenttime();
					Black_PID_Target = (Black_PID_Target >= 49.0f) ? 50.0f : Black_PID_Target + 7.0f;
				}

#ifdef SCPATHTEXT
				my_printf("Black_PID_Target: %.2f\n", Black_PID_Target);
#endif

				xControl = Sc_PIDWithPosition(0.0f, - EMD.Xblack, &Xblack_param);
				yControl = Sc_PIDWithPosition(Black_PID_Target, - EMD.Yblack, &Yblack_param);

				Sc_Set_SoftWare_Velocity_With_Speed_Limit(xControl, yControl, 0.0f + Seek_Adjust);

				Last_X = xControl;
				Last_Y = yControl;
			}

			else
			{
#ifdef SCPATHTEXT
				my_printf("BlackPoint and Car Not Found\n");
#endif

				Sc_Set_SoftWare_Velocity_With_Speed_Limit(Last_X, Last_Y, 0.0f + Seek_Adjust);
			}

		}
		else  //found car
		{
			if(EMD.carAvailable)
			{
				Delay_init_Token = (Delay_init_Token > 0) ? Delay_init_Token - 1 : 0;
				if(currenttime() - Time_add_Interval > 20)
				{
					Time_add_Interval = currenttime();
					Car_PID_Target = (Car_PID_Target >= 0.0f) ? 0.0f : Car_PID_Target + 7.0f;
				}

				#ifdef SCPATHTEXT
					my_printf("Car_PID_Target: %.2f\n", Car_PID_Target);
				#endif

				xControl = Sc_PIDWithPosition(0.0f, - EMD.Xcar, &Xcar_param);
				yControl = Sc_PIDWithPosition(Car_PID_Target, - EMD.Ycar, &Ycar_param);

				Sc_Set_SoftWare_Velocity_With_Speed_Limit(xControl, yControl, 0.0f + Seek_Adjust);

				Last_X = xControl;
				Last_Y = yControl;

				if(fabs(EMD.Xcar) <= 20 && fabs(EMD.Ycar) <= 20 && !Car_Reservation)
					Car_Reservation = 1;
				if(fabs(EMD.Xcar) <= CAR_AREA_THRESHOLD && fabs(EMD.Ycar) <= CAR_AREA_THRESHOLD)
				{
					if(currenttime() - Last_Height_Adjust_Time > HEIGHT_ADJUST_LEAST_INTERVAL)
					{
						Last_Height_Adjust_Time = currenttime();
						Seek_Adjust = (Seek_Adjust <= 0.0f) ? 0.0f : Seek_Adjust - 0.1f;
					}
					if(fabs(Seek_Adjust) <= 1e-3f)
						Car_Found_Token++;
				}
				else
				{
					Car_Found_Token = 0;
				}
				if(Car_Found_Token >= 5)
					break;
			}
			else
			{
#ifdef SCPATHTEXT
				my_printf("Car Not Found\n");
#endif
				Sc_Set_SoftWare_Velocity_With_Speed_Limit(Last_X, Last_Y, 0.0f + Seek_Adjust);
			}
		}
#ifdef SCPATHTEXT
		my_printf("\n");
#endif
	}
}
#else
static void From_BlackPoint_to_Car(void)
{
	float Last_X = 0.0f, Last_Y = 0.25f;
	float xControl, yControl;
	short Car_Found_Token = 0;
	while(Software_Enabled())
	{
		Get_OpenMV_Data();
		if(EMD.carAvailable)
		{
			xControl = Sc_PID(0.0f, - EMD.Xcar, &B2Cx_param);
			yControl = Sc_PID(0.0f, - EMD.Ycar, &B2Cy_param);

			Sc_Set_SoftWare_Velocity_With_Speed_Limit(xControl, yControl, 0.0f);

			Last_X = xControl;
			Last_Y = yControl;

			if(fabs(EMD.Xcar) <= CAR_AREA_THRESHOLD && fabs(EMD.Ycar) <= CAR_AREA_THRESHOLD)
			{
				Car_Found_Token++;
			}
			else
				Car_Found_Token = 0;
			if(Car_Found_Token >= 5)
				break;
		}
		else
		{
#ifdef SCPATHTEXT
				my_printf("Car Not Found\n");
#endif
				Sc_Set_SoftWare_Velocity_With_Speed_Limit(Last_X, Last_Y, 0.0f);
		}
	}
}
#endif

// Follow Car (block)
static void Follow_Car_Block(void)
{
	float last_x = 0.0f, last_y = 0.0f;
	uint32_t highspeedtime = currenttime();
	while(Software_Enabled())
	{
		Get_OpenMV_Data();
		if(!EMD.carAvailable)
		{
			last_x /= DAMPINGRATE;
			last_y /= DAMPINGRATE;
#ifdef SCPATHTEXT
			my_printf("lostcar\n");
#endif
			Sc_Set_SoftWare_Velocity(last_x, last_y, 0.0f);
		}
		else
		{
			float xControl, yControl;

			xControl = Sc_PIDWithPosition(0.0f, - EMD.Xcar, &Xfollow_param);
			yControl = Sc_PIDWithPosition(0.0f, - EMD.Ycar, &Yfollow_param);

			last_x = xControl;
			last_y = yControl;
#ifdef SCPATHTEXT
			my_printf("found\n");
#endif
			Sc_Set_SoftWare_Velocity(xControl, yControl, 0.0f);
		}
		/*if(fabs(last_x) < LOW_SPEED_THRESHOLD && fabs(last_y) < LOW_SPEED_THRESHOLD)
		{
			BUZZER_PIN = 1U;
			if(currenttime() - highspeedtime > LOW_SPEED_TIMEOUT)
			{
				int i;
				for(i = 0;i < 4;i++)
				{
					BUZZER_PIN = 1U;
					delay_ms(100);
					BUZZER_PIN = 0U;
					delay_ms(100);
				}
				my_printf("land\n");
				Land_Evading_Car();
				break;
			}
		}
		else
		{
			highspeedtime = currenttime();
			BUZZER_PIN = 0U;
		}*/
		if(Decode_Land())
		{
			int i;
			for(i = 0;i < 4;i++)
			{
				BIT0_INDICATOR = 1U;
				BIT1_INDICATOR = 1U;
				delay_ms(100);
				BIT0_INDICATOR = 0U;
				BIT1_INDICATOR = 0U;
				delay_ms(100);
			}
			Land_Evading_Car();
			break;
		}
	}
}
///* mission *///

void Mission_One(void)
{
	while(get_Copter_Height() < 0.9f && Software_Enabled())
		PID_to_BlackPoint();
	Sc_TimeHolder(&PID_to_BlackPoint,7000);
#ifdef SCPATHTEXT
	my_printf("landing\n");
#endif
	Land_to_BlackPoint_with_PID();
}

/*void Mission_Two(void)  // unfinished
{
	while(get_Copter_Height() < 0.7f && Software_Enabled())
		PID_to_Car();
	Sc_TimeHolder(&PID_to_Car,10000);
#ifdef SCPATHTEXT
	my_printf("landing\n");
#endif
	Land_Evading_Car();
}*/

void Mission_Two(void)
{
	Get_OpenMV_Data();
#ifndef ALWAYSFRESHDISTANCEINDICATOR
	DistanceIndicatorRefresh();
#endif
}

void Mission_Three(void)
{
	Sc_TimeHolder(&Takeoff_Stablize,3000);
#ifdef SCPATHTEXT
	my_printf("Finding Car\n");
#endif
	From_BlackPoint_to_Car();
#ifdef SCPATHTEXT
	my_printf("Holding\n");
#endif
	Sc_TimeHolder(&PID_to_Car,6000);
#ifdef SCPATHTEXT
	my_printf("Land Evading Car\n");
#endif
	Land_Evading_Car();
	//Copter_Land();
}

void Mission_Four_and_Five(void)
{
	Sc_TimeHolder(&Takeoff_Stablize,3000);
	From_BlackPoint_to_Car();
	BIT0_INDICATOR = 1U;
	BIT1_INDICATOR = 1U;
	Land_Reset();
	Follow_Car_Block();
	BIT0_INDICATOR = 0U;
	BIT1_INDICATOR = 0U;
}
