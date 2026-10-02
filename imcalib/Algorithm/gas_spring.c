#include "gas_spring.h"
#include "machine_config.h"
#include "../../Drivers/CMSIS/DSP/Include/arm_math.h"
#include <math.h>

/* Leg3 同机模型，输出伸腿力 N */
float Leg_SpringF(float L0)
{
    float sqrt_tmp1;
    float theta;

    if (!isfinite(L0))
    {
        return NAN;
    }
    L0 = fminf(fmaxf(L0, 0.05F), 0.45F);
    theta = acosf((0.1066F - L0 * L0) / 0.105F);
    arm_sqrt_f32(fmaxf(0.0473512858F -
        0.0206288453F * arm_cos_f32(theta - 0.213803F), 0.0F), &sqrt_tmp1);
    return 150.0F *
        (0.0103144227F * arm_sin_f32(theta - 0.213803F) * L0 /
        fmaxf(0.0525F * arm_sin_f32(theta) * sqrt_tmp1, 1.0E-6F));
}

/* 抵消伸腿力后叠加原力矩 */
uint8_t Gas_Spring_Apply(const leg_state_t *left, const leg_state_t *right,
                         const float base_dm[4], float raw_dm[4])
{
    float result[4];
    uint8_t i;
#if GAS_SPRING_COMP_ENABLE && MACHINE_DEFAULT == MACHINE_ID_BIG_WHEELLEG
    const leg_state_t *leg[2];
    float delta[2];
    float force;
    uint8_t side;
#endif

    if (raw_dm == NULL)
    {
        return 0u;
    }
    for (i = 0u; i < 4u; i++)
    {
        result[i] = base_dm != NULL ? base_dm[i] : 0.0f;
    }
    for (i = 0u; i < 4u; i++)
    {
        raw_dm[i] = 0.0f;
    }
    if (base_dm == NULL)
    {
        return 0u;
    }
    for (i = 0u; i < 4u; i++)
    {
        if (!isfinite(result[i]))
        {
            return 0u;
        }
    }
#if GAS_SPRING_COMP_ENABLE && MACHINE_DEFAULT == MACHINE_ID_BIG_WHEELLEG
    leg[0] = left;
    leg[1] = right;
    for (side = 0u; side < 2u; side++)
    {
        if (leg[side] == NULL || !leg[side]->output.valid || !leg[side]->output.force_valid)
        {
            return 0u;
        }
        force = -Leg_SpringF(leg[side]->output.virtual_leg_length);
        if (!isfinite(force) || !Leg_Force_Map_Forward(leg[side], force, 0.0f, delta)
            || !isfinite(delta[0]) || !isfinite(delta[1]))
        {
            return 0u;
        }
        result[side * 2u] += delta[0];
        result[side * 2u + 1u] += delta[1];
        if (!isfinite(result[side * 2u]) || !isfinite(result[side * 2u + 1u]))
        {
            return 0u;
        }
    }
#else
    (void)left;
    (void)right;
#endif
    for (i = 0u; i < 4u; i++)
    {
        raw_dm[i] = result[i];
    }
    return 1u;
}
