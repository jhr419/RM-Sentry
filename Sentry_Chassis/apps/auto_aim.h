#ifndef AUTO_AIM_H
#define AUTO_AIM_H

#include "main.h"
#include "typedef.h"

typedef struct{
	union_fp32 target_yaw;
	union_fp32 target_pitch;
	uint8_t fire_mode;
	union_fp32 confidence;
}aim_info_t;

#endif
