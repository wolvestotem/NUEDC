#ifndef __MAVLINK_HEAD__
#define __MAVLINK_HEAD__

#include "../stdafx.h"

#define COPTER_LAND_CONTROL_ENABLED 1

void abort(void);
float get_Copter_Height(void);
void Copter_PreArm(void);
void Wait_For_Take_Off(void);
void Take_Off(void);
void Copter_Land(void);
short Copter_New_Land_Optional(float x, float y, float _des_height, float _des_climbrate);
short Copter_New_Land(float x, float y);
void Copter_Level_Calibration(void);
void MavLink_Start(void);
void Copter_Add_Yaw(float _add);

#endif
