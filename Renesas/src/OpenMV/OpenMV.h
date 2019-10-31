
#ifndef OPENMV_OPENMV_H_
#define OPENMV_OPENMV_H_

#include "../stdafx.h"
#include "../cg_src/r_cg_rspi.h"

#define OPENMV_DECODE_SUCCESS 1
#define OPENMV_GET_FAILED 2
#define OPENMV_DECODE_FAILED 3
#define OPENMV_GET_TIMEOUT 4
#define OPENMV_RESET 5

#define OPENMVTIMEOUT 30

struct Data{
	int yaw_angle;
	int x;
	int y;
};

struct Electric_Match_Data{
	int blackAvailable;
	int Xblack;
	int Yblack;
	int carAvailable;
	int Xcar;
	int Ycar;
};

extern struct Data nav_dat;
extern struct Electric_Match_Data EMD;

short Electric_Match_SPI_Decode(void);

#endif /* OPENMV_OPENMV_H_ */
