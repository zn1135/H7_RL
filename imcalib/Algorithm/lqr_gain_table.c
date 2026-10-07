#include "lqr_gain_table.h"
#include <math.h>
#include <string.h>

#if MACHINE_DEFAULT == MACHINE_ID_SMALL_WHEELLEG
#include "../../tools/matlab/baseline/lqr_gain_small_legacy.inc"
static const lqr_gain_desc_t *const selected_gain = &lqr_gain_small;
#else
static const lqr_gain_desc_t *const selected_gain = &lqr_gain_big;
#endif

const lqr_gain_desc_t *LQR_Gain_Info(void)
{
    return selected_gain;
}

uint8_t LQR_Gain_Check(const lqr_gain_desc_t *gain, const machine_cfg_t *cfg,
                       uint8_t machine_id, float dt)
{
    float low;
    float high;
    uint8_t i;

    if (gain == NULL || cfg == NULL || gain->eval == NULL || !gain->checked)
    {
        return LQR_GAIN_BAD_TABLE;
    }
    if (gain->machine_id != machine_id)
    {
        return LQR_GAIN_BAD_MACHINE;
    }
    if (gain->state_schema != LQR_STATE_HIP_FRONT_V2 || gain->input_schema != LQR_INPUT_WHEELS_HIPS_V1)
    {
        return LQR_GAIN_BAD_SCHEMA;
    }
    if (!isfinite(dt) || dt <= 0.0f || !isfinite(gain->dt) || !isfinite(cfg->lqr.dt)
        || fabsf(gain->dt - dt) > 1.0e-7f || fabsf(cfg->lqr.dt - dt) > 1.0e-7f)
    {
        return LQR_GAIN_BAD_PERIOD;
    }
    if (!isfinite(gain->wheel_r) || !isfinite(gain->leg_lu) || !isfinite(gain->leg_lg)
        || !isfinite(cfg->wheel_r) || !isfinite(cfg->leg_lu) || !isfinite(cfg->leg_lg)
        || gain->wheel_r <= 0.0f || gain->leg_lu <= 0.0f || gain->leg_lg <= 0.0f
        || fabsf(gain->wheel_r - cfg->wheel_r) > 1.0e-6f
        || fabsf(gain->leg_lu - cfg->leg_lu) > 1.0e-6f
        || fabsf(gain->leg_lg - cfg->leg_lg) > 1.0e-6f)
    {
        return LQR_GAIN_BAD_GEOMETRY;
    }
    if (!isfinite(gain->len_min) || !isfinite(gain->len_max)
        || !isfinite(cfg->leg_len_min) || !isfinite(cfg->leg_len_max)
        || gain->len_min <= 0.0f || gain->len_min >= gain->len_max
        || cfg->leg_len_min <= 0.0f || cfg->leg_len_min >= cfg->leg_len_max)
    {
        return LQR_GAIN_BAD_DOMAIN;
    }
    low = fmaxf(gain->len_min, cfg->leg_len_min);
    high = fminf(gain->len_max, cfg->leg_len_max);
    if (low >= high)
    {
        return LQR_GAIN_BAD_DOMAIN;
    }
    for (i = 0u; i < 2u; i++)
    {
        if (!isfinite(cfg->lqr.leg_len_init[i]) || cfg->lqr.leg_len_init[i] < low
            || cfg->lqr.leg_len_init[i] > high)
        {
            return LQR_GAIN_BAD_DOMAIN;
        }
    }
    return LQR_GAIN_OK;
}

uint8_t LQR_Gain_Compatible(void)
{
    static uint8_t checked;
    static uint8_t compatible;

    if (!checked)
    {
        compatible = (uint8_t)(LQR_Gain_Check(selected_gain, machine,
            Machine_Id(), MACHINE_LQR_DT) == LQR_GAIN_OK);
        checked = 1u;
    }
    return compatible;
}

uint8_t LQR_Ready(void)
{
    return (uint8_t)(machine->lqr_configured && LQR_Gain_Compatible());
}

uint8_t LQR_Gain_Eval(float lL, float lR, float K[40], uint8_t legacy)
{
    uint8_t i;

    if (K == NULL)
    {
        return 0u;
    }
    memset(K, 0, 40u * sizeof(float));
    if (!LQR_Gain_Compatible() || !isfinite(lL) || !isfinite(lR))
    {
        return 0u;
    }
    lL = fminf(fmaxf(lL, selected_gain->len_min), selected_gain->len_max);
    lR = fminf(fmaxf(lR, selected_gain->len_min), selected_gain->len_max);
    if (legacy)
    {
#if MACHINE_DEFAULT == MACHINE_ID_SMALL_WHEELLEG
        LQR_K_Small_Legacy(lL, lR, K);
        /* 旧表状态基变换 */
        for (i = 16u; i < 32u; i++)
        {
            K[i] = -K[i];
        }
#else
        return 0u;
#endif
    }
    else
    {
        selected_gain->eval(lL, lR, K);
    }
    for (i = 0u; i < 40u; i++)
    {
        if (!isfinite(K[i]))
        {
            memset(K, 0, 40u * sizeof(float));
            return 0u;
        }
    }
    return 1u;
}

void LQR_K_WBR(float lL, float lR, float K[40])
{
    (void)LQR_Gain_Eval(lL, lR, K, 0u);
}
