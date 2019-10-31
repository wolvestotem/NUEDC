/*
 * Sc_PID.c
 *
 *  Created on: 2017年7月15日
 *      Author: msi
 */

#include "Sc_PID.h"

float Sc_PID(float target, float source, struct PID_Param *param)
{
	float timepassed;
	float control;
	float variation;
	float WKp,WKi,WKd;
	//if(fabs(target - source)<=3.0f)    hysteresis
	//	return 0.0f;
	timepassed = currenttime() - param -> lasttime;
	control = 0.0f;
	variation = target - source;
	if(timepassed <= _ExpectedIntervalTimeMax)    //make sure the interval may not be too long
		param -> Intergral += variation * timepassed * param->Ki;
	if(fabs(param -> Intergral) >= param -> iMax)
		param -> Intergral = param -> Intergral > 0? param -> iMax : - param -> iMax;
	//must have p

	WKp = variation * param->Kp;
	WKi = param -> Intergral;
	WKd = (variation - param -> lastvariation) / timepassed * param->Kd;


	control = WKp + WKi + WKd;
	/*if(fabs(variation > 7))
		control = 0.0f;*/
#ifdef SCPIDTEXT
	{
		/*my_printf("Source: %.4f Target: %.4f\nVariation: %.4f TimePassed: %.4f\n",source,target,variation,timepassed);
		my_printf("Weight of kp: %.4f\n",WKp);
		if(fabs(param->Ki) > 1e-6)
			my_printf("Weight of ki: %.4f\n",WKi);
		if(fabs(param->Kd) > 1e-6)
			my_printf("Weight of kd: %.4f\n",WKd);*/

		my_printf("Src: %.4f Targ: %.4f\nVar: %.4f TiP: %.4f\n",source,target,variation,timepassed);
		my_printf("Wkp: %.4f\n",WKp);
		if(fabs(param->Ki) > 1e-6)
			my_printf("Wki: %.4f\n",WKi);
		if(fabs(param->Kd) > 1e-6)
			my_printf("Wkd: %.4f\n",WKd);
	}
#endif

	param -> lastvariation = variation;
	param -> lasttime += timepassed;

	return control;
}

float Sc_PIDWithPosition(float target, float source, struct PID_Param *param)
{
	float timepassed;
	float control;
	float variation;
	float WKp,WKi,WKd;
	//if(fabs(target - source)<=3.0f)    hysteresis
	//	return 0.0f;
	timepassed = currenttime() - param -> lasttime;
	control = 0.0f;
	variation = target - source;
	if(timepassed <= _ExpectedIntervalTimeMax)    //make sure the interval may not be too long
		param -> Intergral += variation * timepassed * param->Ki;
	if(fabs(param -> Intergral) >= param -> iMax)
		param -> Intergral = param -> Intergral > 0? param -> iMax : - param -> iMax;
	//must have p

	WKp = variation * param->Kp;
	WKi = param -> Intergral;
	WKd = (source - param -> lastvariation) / timepassed * param->Kd;


	control = WKp + WKi + WKd;
	/*if(fabs(variation > 7))
		control = 0.0f;*/
#ifdef SCPIDTEXT
	{
		/*my_printf("Source: %.4f Target: %.4f\nVariation: %.4f TimePassed: %.4f\n",source,target,variation,timepassed);
		my_printf("Weight of kp: %.4f\n",WKp);
		if(fabs(param->Ki) > 1e-6)
			my_printf("Weight of ki: %.4f\n",WKi);
		if(fabs(param->Kd) > 1e-6)
			my_printf("Weight of kd: %.4f\n",WKd);*/

		my_printf("Src: %.4f Targ: %.4f\nVar: %.4f TiP: %.4f\n",source,target,variation,timepassed);
		my_printf("Wkp: %.4f\n",WKp);
		if(fabs(param->Ki) > 1e-6)
			my_printf("Wki: %.4f\n",WKi);
		if(fabs(param->Kd) > 1e-6)
			my_printf("Wkd: %.4f\n",WKd);
	}
#endif

	param -> lastvariation = source;
	param -> lasttime += timepassed;

	return control;
}

/*float Sc_PIDNoDebug(float target, float source, struct PID_Param *param)
{
	float timepassed;
	float control;
	float variation;
	float WKp,WKi,WKd;
	if(fabs(target - source)<=5.0f)
		return 0.0f;
	timepassed = currenttime() - param -> lasttime;
	control = 0.0f;
	variation = target - source;
	//if(variation * param -> Intergral < 0.0f)
	if(timepassed <= _ExpectedIntervalTimeMax)    //make sure the interval may not be too long
		param -> Intergral += variation * timepassed;
	if(fabs(variation) < 0.01f || variation * param -> Intergral < 0.0f)
		param -> Intergral = 0.0f;
	if(fabs(param -> Intergral) >= _IntergralMaxNew)
		param -> Intergral = param -> Intergral > 0? _IntergralMaxNew : -_IntergralMaxNew;
	//must have p

	WKp = variation * param->Kp;
	WKi = param -> Intergral * param->Ki;
	if(fabs(variation - param -> lastvariation) < 1e-6)
		WKd = param -> lastWKd;
	else
		WKd = (variation - param -> lastvariation) / timepassed * param->Kd;

	control = WKp + WKi + WKd;

#ifdef SCDEBUGTEXTNEW
	{
		my_printf("Source: %.4f Target: %.4f\nVariation: %.4f TimePassed: %.4f\n",source,target,variation,timepassed);
		my_printf("Weight of kp: %.4f\n",WKp);
		if(fabs(param->Ki) > 1e-6)
			my_printf("Weight of ki: %.4f\n",WKi);
		if(param->Kd > 1e-6)
			my_printf("Weight of kd: %.4f\n",WKd);
	}
#endif

	param -> lastvariation = variation;
	param -> lasttime += timepassed;
	param -> lastWKd = WKd;
	//my_printf("control: %f\n",control);
	return control;
}*/
