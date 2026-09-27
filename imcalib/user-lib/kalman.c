#include "kalman.h"

/* ================= 带加速度输入的一维卡尔曼 ================= */

/* 初始化 */
void Kalman_Accel_Init(kalman_accel_t *kf, float v0, float p0, float q,
                       float r, float p_max)
{
    kf->vel   = v0;
    kf->p     = p0;
    kf->q     = q;
    kf->r     = r;
    kf->p_max = p_max;
}

/* 单周期: 加速度预测 → 协方差钳位 → 观测修正 */
float Kalman_Accel_Update(kalman_accel_t *kf, float accel, float z_vel, float dt)
{
    float k;

    kf->vel += accel * dt;
    kf->p   += kf->q;
    if (kf->p > kf->p_max)
    {
        kf->p = kf->p_max;      /* 防静止后 p 无限涨 */
    }
    k = kf->p / (kf->p + kf->r);
    kf->vel += k * (z_vel - kf->vel);
    kf->p    = (1.0f - k) * kf->p;
    return kf->vel;
}
