#ifndef LQR_GAIN_TABLE_H
#define LQR_GAIN_TABLE_H

#include <stdint.h>
#include "machine_config.h"

#define LQR_STATE_HIP_FRONT_V2 2u
#define LQR_INPUT_WHEELS_HIPS_V1 1u

typedef void (*lqr_gain_eval_t)(float lL, float lR, float K[40]);
typedef struct {
    const char *table_id;
    const char *model;
    uint8_t machine_id;
    uint8_t state_schema;
    uint8_t input_schema;
    uint8_t checked;
    float dt;
    float len_min;
    float len_max;
    float wheel_r;
    float leg_lu;
    float leg_lg;
    float x_eq[10];
    float u_eq[4];
    lqr_gain_eval_t eval;
} lqr_gain_desc_t;

enum {
    LQR_GAIN_OK = 0,
    LQR_GAIN_BAD_TABLE,
    LQR_GAIN_BAD_MACHINE,
    LQR_GAIN_BAD_SCHEMA,
    LQR_GAIN_BAD_PERIOD,
    LQR_GAIN_BAD_GEOMETRY,
    LQR_GAIN_BAD_DOMAIN,
};

extern const lqr_gain_desc_t lqr_gain_small;
extern const lqr_gain_desc_t lqr_gain_big;
const lqr_gain_desc_t *LQR_Gain_Info(void);
uint8_t LQR_Gain_Check(const lqr_gain_desc_t *gain, const machine_cfg_t *cfg,
                       uint8_t machine_id, float dt);
uint8_t LQR_Gain_Compatible(void);
uint8_t LQR_Ready(void);
uint8_t LQR_Gain_Eval(float lL, float lR, float K[40], uint8_t legacy);
void LQR_K_WBR(float lL, float lR, float K[40]);

#endif
