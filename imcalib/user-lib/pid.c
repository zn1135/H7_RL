#include "pid.h"
#include <math.h>

static void pid_param_init(pid_t *pid, uint8_t mode, float maxout,
                           float intergral_limit, float kp, float ki, float kd);

static float ABS(float num)
{
    return (num < 0.0f) ? -num : num;
}

/* 限制输出 */
void abs_limit(float *value, float abs_max)
{
    if (!(*value * 0.0f == 0.0f))
    {
        *value = 0.0f;
        return;
    }
    if (*value > abs_max) *value = abs_max;
    if (*value < -abs_max) *value = -abs_max;
}

/* 软化死区 */
float Deadband_Soften(float err, float deadband)
{
    if (deadband <= 0.0f) return err;
    if (err > deadband) return err - deadband;
    if (err < -deadband) return err + deadband;
    return 0.0f;
}

/* 环绕角度 [-π, π] (最短路径) */
float Angle_Wrap_180(float angle)
{
    while (angle > 3.14159265f) angle -= 6.28318530f;
    while (angle < -3.14159265f) angle += 6.28318530f;
    return angle;
}

/* 计算PID */
float pid_calc(pid_t *pid, float get, float set, float delta_time)
{
    pid->get[NOW] = get;
    pid->set[NOW] = set;
    pid->err[NOW] = set - get;
    if (pid->angle_wrap) pid->err[NOW] = Angle_Wrap_180(pid->err[NOW]);
    if (pid->max_err != 0.0f && ABS(pid->err[NOW]) > pid->max_err)
        return 0.0f;

    float e_now = Deadband_Soften(pid->err[NOW], pid->deadband);
    float e_last = Deadband_Soften(pid->err[LAST], pid->deadband);

    if (pid->pid_mode == POSITION_PID)
    {
        pid->pout = pid->p * e_now;
        pid->iout += pid->i * e_now * delta_time;
        if (pid->d == 0.0f)
        {
            pid->dout = 0.0f;
        }
        else
        {
            pid->dout = pid->d * (pid->err[NOW] - pid->err[LAST]);
        }

        abs_limit(&pid->iout, pid->IntegralLimit);
        pid->pos_out = pid->pout + pid->iout + pid->dout;
        abs_limit(&pid->pos_out, pid->MaxOutput);
        pid->last_pos_out = pid->pos_out;
    }
    else if (pid->pid_mode == DELTA_PID)
    {
        float e_llast = Deadband_Soften(pid->err[LLAST], pid->deadband);
        pid->pout = pid->p * (e_now - e_last);
        pid->iout = pid->i * e_now * delta_time;
        pid->dout = pid->d * (e_now - 2.0f * e_last + e_llast);
        abs_limit(&pid->iout, pid->IntegralLimit);
        pid->delta_u = pid->pout + pid->iout + pid->dout;
        pid->delta_out = pid->last_delta_out + pid->delta_u;
        abs_limit(&pid->delta_out, pid->MaxOutput);
        pid->last_delta_out = pid->delta_out;
    }

    pid->err[LLAST] = pid->err[LAST];
    pid->err[LAST] = pid->err[NOW];
    pid->get[LLAST] = pid->get[LAST];
    pid->get[LAST] = pid->get[NOW];
    pid->set[LLAST] = pid->set[LAST];
    pid->set[LAST] = pid->set[NOW];
    return (pid->pid_mode == POSITION_PID) ? pid->pos_out : pid->delta_out;
}

/* 计算前馈 */
float FeedForwardController(FFC_t *ffc, float target, float num1, float num2)
{
    ffc->rin = target;
    float result = num1 * (ffc->rin - ffc->lastRin)
        + num2 * (ffc->rin - 2.0f * ffc->lastRin + ffc->perRin);
    float old_last = ffc->lastRin;
    ffc->lastRin = ffc->rin;
    ffc->perRin = old_last;
    ffc->FFC_pos_out = result;
    return result;
}

/* 初始化参数 */
void PID_struct_init(pid_t *pid, uint8_t mode, float maxout,
                     float intergral_limit, float kp, float ki, float kd,
                     float ff_param1, float ff_param2)
{
    pid->f_param_init = pid_param_init;
    pid->f_param_init(pid, mode, maxout, intergral_limit, kp, ki, kd);
    FeedForwardParamInit(&pid->xFeedForward, ff_param1, ff_param2);
}

/* 初始化前馈 */
void FeedForwardParamInit(FFC_t *ffc, float param1, float param2)
{
    ffc->num1 = param1;
    ffc->num2 = param2;
}

/* 保存参数 */
static void pid_param_init(pid_t *pid, uint8_t mode, float maxout,
                           float intergral_limit, float kp, float ki, float kd)
{
    pid->IntegralLimit = intergral_limit;
    pid->MaxOutput = maxout;
    pid->pid_mode = mode;
    pid->p = kp;
    pid->i = ki;
    pid->d = kd;
}
