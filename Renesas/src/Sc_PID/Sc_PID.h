/*
 * Sc_PID.h
 *
 *  Created on: 2017年7月15日
 *      Author: msi
 */

#ifndef SC_PATHPLAN_SC_PID_SC_PID_H_
#define SC_PATHPLAN_SC_PID_SC_PID_H_

#include "../stdafx.h"

//#define SCDEBUGTEXT
//#define SCDEBUGTEXTNEW
//#define SPEEDLIMITER
//#define PRINTALLDATA

#define _ExpectedIntervalTimeMax 1000
//#define _IntergralMaxNew 25

struct PID_Param{
	float Kp;
	float Ki;
	float Kd;
	float iMax;
	float lasttime;
	float Intergral;
	float lastvariation;
};

float Sc_PID(float target, float source, struct PID_Param *param);
float Sc_PIDNoDebug(float target, float source, struct PID_Param *param);
float Sc_PIDWithPosition(float target, float source, struct PID_Param *param);

#endif /* SC_PATHPLAN_SC_PID_SC_PID_H_ */
