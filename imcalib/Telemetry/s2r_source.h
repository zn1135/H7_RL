#ifndef S2R_SOURCE_H
#define S2R_SOURCE_H

#include "s2r_telemetry.h"

/* ============================================================================
 * 采样适配层 (gap 测试)
 * 只读现有固件的全局状态 (imu_state / motor_state / leg_l / leg_r / rl_control /
 * action_state / hi229_data ...), 通过变化检测推断事件, 不修改任何现有结构体,
 * 也不改动现有任务逻辑; 所有序号与时间戳都由本模块自行维护。
 * ==========================================================================*/

/* 通信任务每拍调用一次 (由 S2R_Pump 内部调用) */
void S2R_Source_Tick(void);

/* 会话切换 / 重入时清本地序号与变化检测基线 */
void S2R_Source_Reset(void);

#endif
