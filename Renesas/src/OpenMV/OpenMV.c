#include "OpenMV.h"

//uint16_t debugint = 0;

struct Data nav_dat = {0,0,0};
struct Electric_Match_Data EMD = {0,0,0,0,0,0};

short Follow_Car_SPI_Decode(void)
{
	uint32_t temparr[7];
	int i = 0;
	uint32_t entrytime = currenttime();

	//PORT3.PODR.BIT.B0 = 0U;

	while(SPI_Recv_One() != 0xFE && currenttime() - entrytime < OPENMVTIMEOUT);   //eat trash
	if(!(currenttime() - entrytime < OPENMVTIMEOUT))
		return OPENMV_GET_TIMEOUT;
	/*0xFE*/
	do
	{
		temparr[i] = SPI_Recv_One();
		i++;
	}while(i<7 && temparr[i-1] != 0xFF);
	if(temparr[i-1] == 0xFF)
	{
		//check data
		int sum = 0;
		for(i = 0;i < 3;i++)
			sum += temparr[i] % 10;
		if(temparr[3] == sum)
		{
			//data assignment
			nav_dat.yaw_angle = temparr[0] - 120;
			nav_dat.x = temparr[1] - 80;
			nav_dat.y = temparr[2] - 60;
			//PORT3.PODR.BIT.B0 = 1U;
			return OPENMV_DECODE_SUCCESS;
		}
		else
		{
			//PORT3.PODR.BIT.B0 = 1U;
			return OPENMV_DECODE_FAILED;
		}
	}
	else
		return OPENMV_GET_FAILED;
}

short Electric_Match_SPI_Decode(void)
{
	uint32_t temparr[7];
	int i = 0;
	uint32_t entrytime = currenttime();

	//PORT3.PODR.BIT.B0 = 0U;

	while(SPI_Recv_One() != 0xFE && currenttime() - entrytime < OPENMVTIMEOUT);   //eat trash
	if(!(currenttime() - entrytime < OPENMVTIMEOUT))
	{
		EMD.blackAvailable = 0;
		EMD.carAvailable = 0;
		return OPENMV_GET_TIMEOUT;
	}
	/*0xFE*/
	do
	{
		temparr[i] = SPI_Recv_One();
		i++;
	}while(i<7 && temparr[i-1] != 0xFF);
	if(temparr[i-1] == 0xFF)
	{
		//check data
		int sum = 0;
		for(i = 0;i < 4;i++)
			sum += temparr[i] % 10;
		if(temparr[4] == sum)
		{
			//data assignment
			if(temparr[0] != 253 && temparr[1] != 253)
			{
				EMD.blackAvailable = 1;
				EMD.Xblack = temparr[0] - 80;
				EMD.Yblack = temparr[1] - 60;
			}
			else
			{
				EMD.blackAvailable = 0;
				//EMD.Xblack = 0;
				//EMD.Yblack = 0;
			}
			if(temparr[2] != 253 && temparr[3] != 253)
			{
				EMD.carAvailable = 1;
				EMD.Xcar = temparr[2] - 80;
				EMD.Ycar = temparr[3] - 60;
			}
			else
			{
				EMD.carAvailable = 0;
				//EMD.Xcar = 0;
				//EMD.Ycar = 0;
			}
#ifdef SCOPENMVTEXT
			my_printf("%d, %d, %d, %d, %d, %d\n",EMD.blackAvailable,EMD.Xblack,EMD.Yblack,EMD.carAvailable,EMD.Xcar,EMD.Ycar);
#endif
			//PORT3.PODR.BIT.B0 = 1U;
			return OPENMV_DECODE_SUCCESS;
		}
		else
		{
			//PORT3.PODR.BIT.B0 = 1U;
			EMD.blackAvailable = 0;
			EMD.carAvailable = 0;
			return OPENMV_DECODE_FAILED;
		}
	}
	else
		EMD.blackAvailable = 0;
		EMD.carAvailable = 0;
		return OPENMV_GET_FAILED;
}
