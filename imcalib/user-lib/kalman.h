#ifndef KALMAN_H
#define KALMAN_H

/* ================= 带加速度输入的一维卡尔曼速度估计 (同 Leg2_v1) =================
 * 过程: v = v + a·dt ; 观测: z = v ; 详见 md/LQR_PLAN.md §2.8 */

typedef struct {
    float vel;      /* 速度估计 */
    float p;        /* 误差协方差 */
    float q;        /* 过程噪声 */
    float r;        /* 观测噪声 */
    float p_max;    /* 协方差上限 */
} kalman_accel_t;

void  Kalman_Accel_Init(kalman_accel_t *kf, float v0, float p0, float q,
                        float r, float p_max);
float Kalman_Accel_Update(kalman_accel_t *kf, float accel, float z_vel, float dt);

#endif
