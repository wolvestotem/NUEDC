
#ifndef __SC_COPTERCONTROL__
#define __SC_COPTERCONTROL__

// Includes
#include "../stdafx.h"

// Symbol

// Macro Variables
#define _PI 3.14159265

// extern variables
//extern float SoftWareHeadAngle;

// Function Claim
void Sc_Set_SoftWare_Velocity(float Sc_x, float Sc_y, float Sc_z);
void Sc_Set_SoftWare_Velocity_With_Speed_Limit(float Sc_x, float Sc_y, float Sc_z);
void Sc_Adjust_SoftWareHeadAngle(float _addangle);
void Sc_Set_SoftWareHeadAngle(float _setangle);
//-----------------------------------------------------------------
float WrapDegtoRad(float _Deg);  //Wrap degree to -pi ~ pi Rad
float WrapRadtoDeg(float _Rad);  //Wrap rad to -180 ~ 180 Deg
float WrapDeg180(float _Deg);    //Wrap degree to -180 ~ 180 Deg
float WrapRad180(float _Rad);    //Wrap rad to -pi ~ pi Rad

#endif
