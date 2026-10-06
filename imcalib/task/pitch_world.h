// 世界俯仰头文件
// 版本：v1.2
#ifndef PITCH_WORLD_H
#define PITCH_WORLD_H

#include <stdbool.h>

bool Pitch_World_Calc(float pitch, const float quat[4], float *pitch_world);

#endif
