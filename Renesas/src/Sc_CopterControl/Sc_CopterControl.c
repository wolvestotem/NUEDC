
#include "Sc_CopterControl.h"

/*			Head(90)
 *
 * Left(180)		Right(0)
 *
 * 		   Back(270)
 */

/* Global Variables */
static float SoftWareHeadAngle = 90.0f;  // use degree
const float RealHeadAngle = 90.0f;

/* Function Claim */
static void Sc_Set_Real_Velocity(float Sc_x, float Sc_y, float Sc_z);

/* Function Prototype */
void Sc_Set_SoftWare_Velocity(float Sc_x, float Sc_y, float Sc_z)  // change software velocity to real velocity
{
	if(fabs(SoftWareHeadAngle - RealHeadAngle) < 1e-3)
	{
		Sc_Set_Real_Velocity(Sc_x, Sc_y, Sc_z);
	}
	else
	{
		float Cal_x, Cal_y;
		float angle_dif = SoftWareHeadAngle - RealHeadAngle;
		Cal_x = Sc_x * cosf(angle_dif) - Sc_y * sinf(angle_dif);
		Cal_y = Sc_x * sinf(angle_dif) + Sc_y * cosf(angle_dif);

		Sc_Set_Real_Velocity(Cal_x, Cal_y, Sc_z);
	}
}

static float Sc_speed_limiter(float _MinSpeed, float inputspeed, float _MaxSpeed)
{
	if(_MinSpeed > _MaxSpeed)
		my_printf("wrong parameter for speed limiter\n");
	else
		inputspeed = (inputspeed > _MaxSpeed) ? _MaxSpeed : (inputspeed < _MinSpeed) ? _MinSpeed : inputspeed;
	return inputspeed;
}

void Sc_Set_SoftWare_Velocity_With_Speed_Limit(float Sc_x, float Sc_y, float Sc_z)  // change software velocity to real velocity
{
	if(fabs(SoftWareHeadAngle - RealHeadAngle) < 1e-3)
	{
		Sc_x = Sc_speed_limiter(-0.28f, Sc_x, 0.28f);
		Sc_y = Sc_speed_limiter(-0.28f, Sc_y, 0.28f);
		Sc_Set_Real_Velocity(Sc_x, Sc_y, Sc_z);
	}
	else
	{
		float Cal_x, Cal_y;
		float angle_dif = SoftWareHeadAngle - RealHeadAngle;
		Cal_x = Sc_x * cosf(angle_dif) - Sc_y * sinf(angle_dif);
		Cal_y = Sc_x * sinf(angle_dif) + Sc_y * cosf(angle_dif);

		Cal_x = Sc_speed_limiter(-0.50f, Cal_x, 0.50f);
		Cal_y = Sc_speed_limiter(-0.50f, Cal_y, 0.50f);

		Sc_Set_Real_Velocity(Cal_x, Cal_y, Sc_z);
	}
}

static void Sc_Set_Real_Velocity(float Sc_x, float Sc_y, float Sc_z)
{
	/* Add Real Velocity Here */
	set_new_vel(Sc_y, Sc_x, COPTER_HEIGHT + Sc_z);
#ifdef SCVELOCITYTEXT
	my_printf("Real Vel: %.4f, %.4f, %.4f\n",Sc_y, Sc_x, COPTER_HEIGHT + Sc_z);
#endif
}

void Sc_Adjust_SoftWareHeadAngle(float _addangle)
{
	SoftWareHeadAngle += _addangle;
}

void Sc_Set_SoftWareHeadAngle(float _setangle)
{
	SoftWareHeadAngle += _setangle;
}

/* Tools */
float WrapDegtoRad(float _Deg)  //Wrap degree to -pi ~ pi Rad
{
	while(_Deg > 180.0f || _Deg < -180.0f)
		_Deg += (_Deg > 180.0f) ? -360.0f : 360.0f;
	return _Deg / 180.0f * _PI;
}

float WrapRadtoDeg(float _Rad) //Wrap rad to -180 ~ 180 Deg
{
	while(_Rad > _PI || _Rad < -_PI)
		_Rad += (_Rad > _PI) ? -2 * _PI : 2 * _PI;
	return _Rad / _PI * 180.0f;
}

float WrapDeg180(float _Deg)
{
	while(_Deg > 180.0f || _Deg < -180.0f)
		_Deg += (_Deg > 180.0f) ? -360.0f : 360.0f;
	return _Deg;
}

float WrapRad180(float _Rad)
{
	while(_Rad > _PI || _Rad < -_PI)
		_Rad += (_Rad > _PI) ? -2 * _PI : 2 * _PI;
	return _Rad;
}
