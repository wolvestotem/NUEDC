
#ifndef PATHSEEKING_PATHSEEKING_H_
#define PATHSEEKING_PATHSEEKING_H_

#include "../stdafx.h"
#include "../OpenMV/OpenMV.h"
#include "../Sc_PID/Sc_PID.h"

#define GETDATATIME 60
#define GETDATAASSERTTIME 200
#define DAMPINGRATE 1.15f
#define LOW_SPEED_THRESHOLD 0.14f
#define LOW_SPEED_TIMEOUT 3000
#define HEIGHT_ADJUST_LEAST_INTERVAL 500
#define CAR_AREA_THRESHOLD 13

extern struct Data nav_dat;

//void follow_car(void);
void Mission_One(void);
void Mission_Two(void);
void Mission_Three(void);
void Mission_Four_and_Five(void);

#endif /* PATHSEEKING_PATHSEEKING_H_ */
