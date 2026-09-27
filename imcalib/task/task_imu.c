#include "robot_control.h"
#include "hi229.h"
#include "Attitude_Algorithm.h"
#include "machine_config.h"

void imu_task_init(void)
{
    Attitude_Init(&imu_state);
}

/* 读取 + 按机器表取轴/乘符号 + 单位换算 */
void imu_task_body(void)
{
    hi229_data_t sample;
    imu_state_t next = imu_state;
    const imu_cfg_t *cfg;
    uint32_t primask;
    uint8_t i;

    HI229_Process();
    if (!HI229_Online())
    {
        imu_state.online = 0u;
        return;
    }
    sample = HI229_Snapshot();

    /* 去重 */
    if (next.online && sample.ts == next.last_timestamp_ms)
        return;

    /* 首帧初始化 */
    if (!next.online)
        Attitude_Init(&next);

    /* 欧拉角、四元数分别按源通道重映射，再独立应用极性 */
    cfg = &machine->imu;
    next.quat[0] = sample.quat[0];
    for (i = 0u; i < 3u; i++)
    {
        next.quat[i + 1u]        = (float)cfg->quat_sign[i]
                                 * sample.quat[cfg->quat_src[i] + 1u];
        next.euler_deg[i]        = (float)cfg->eul_sign[i] * sample.eul[cfg->eul_src[i]];
        next.gyro_rad_s[i]       = (float)cfg->gyr_sign[i] * sample.gyr[i] * 0.01745329251994f;
        next.acc_g[i]            = (float)cfg->acc_sign[i] * sample.acc[i];
    }

    /* 四元数归一化 + deg→rad */
    next.online = Attitude_Update(&next) ? 1u : 0u;
    next.last_timestamp_ms = sample.ts;
    primask = __get_PRIMASK();
    __disable_irq();
    imu_state = next;
    __set_PRIMASK(primask);
}
