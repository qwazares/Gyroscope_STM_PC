#ifndef __MAIN_H
#define __MAIN_H

#include "gyroscope.h"

//-------------- FLAG -------------------------
#define gyro_int_flag 0
#define BUTTON_int_flag 1
#define conf_int_flag 2

//--------------Config Byte -----------------------------
#define TRANSMITION_BIT 0
#define FULLSCALE_BIT_1 1
#define FULLSCALE_BIT_2 2
#define DATARATE_BIT_1 3
#define DATARATE_BIT_2 4

#define FULLSCALE_MASK (1<<FULLSCALE_BIT_1 | 1<<FULLSCALE_BIT_2)
#define FULLSCALE_BIT_250 0
#define FULLSCALE_BIT_500 (1<<FULLSCALE_BIT_1)
#define FULLSCALE_BIT_2000 (1<<FULLSCALE_BIT_2)

#define DATARATE_MASK (1<<DATARATE_BIT_1 | 1<<DATARATE_BIT_2)
#define DATARATE_BIT_95		0
#define DATARATE_BIT_190	(1<<DATARATE_BIT_1)
#define DATARATE_BIT_380	(1<<DATARATE_BIT_2)
#define DATARATE_BIT_760	(1<<DATARATE_BIT_1 | 1<<DATARATE_BIT_2)

//-------------- Data Byte -----------------------------
#define START_BIT 7
#define BIT_X_L 0
#define BIT_X_H 1
#define BIT_Y_L 2
#define BIT_Y_H 3
#define BIT_Z_L 4
#define BIT_Z_H 5

#endif
