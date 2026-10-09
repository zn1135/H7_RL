"""Run real RL/LQR torque code and independent gas compensation on a host.

PID and leg geometry are production sources. Only hardware sends and the
controller's upstream state are fixtures. CMSIS transcendental calls use a libm
shim; MCU precision, network, RTOS and CAN timing are not tested.
"""

from pathlib import Path
import os
import re
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]


def read(path):
    return (ROOT / path).read_text(encoding="utf-8")


def without_includes(source):
    return re.sub(r"^#include[^\n]*\n", "", source, flags=re.M)


def production_function(source, name):
    match = re.search(r"^(?:static\s+)?(?:void|float|uint8_t|int16_t|uint16_t)\s+" + name + r"\([^)]*\)\s*\{", source, re.M)
    if match is None:
        raise AssertionError("missing production function: " + name)
    depth, end = 1, match.end()
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[match.start():end]


def production_enum(source, name):
    match = re.search(r"typedef\s+enum\s*\{[^}]*\}\s*" + name + r"\s*;", source)
    if match is None:
        raise AssertionError("missing production enum: " + name)
    return match.group()


HEADERS = r"""
#include <assert.h>
#include <math.h>
#include <float.h>
#include "arm_math.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "machine_config.h"
#include "gas_spring.h"
#include "gravity_comp.h"
#include "pid.h"
#include "simple-function.h"
#include "kalman.h"
#define RL_ACTION_SIZE 6
#define CTRL_DT MACHINE_CTRL_DT
typedef int rl_model_t;
typedef struct { int unused; } rc_command_t;
typedef struct { int unused; } imu_state_t;
"""

STUBS = r"""
static machine_cfg_t fixture_machine;
const machine_cfg_t *const machine = &fixture_machine;
lqr_debug_t lqr_debug;
typedef enum { HAL_OK, HAL_ERROR } HAL_StatusTypeDef;
static uint8_t torque_output_enabled, output_debug_dm_sent, output_debug_dji_sent;
static compensation_debug_t compensation_debug;
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
static volatile float rl_output_dm_cmd_nm[4], rl_output_wheel_cmd_nm[2];
static float fixture_sent_dm[4], fixture_sent_wheel[2];
static HAL_StatusTypeDef fixture_dm_status;
static HAL_StatusTypeDef Dm_Send_Zero(void)
{
    memset(fixture_sent_dm, 0, sizeof(fixture_sent_dm));
    return fixture_dm_status;
}
static HAL_StatusTypeDef Dji_All_Stop(void)
{
    memset(fixture_sent_wheel, 0, sizeof(fixture_sent_wheel));
    return HAL_OK;
}
static HAL_StatusTypeDef Dm_Send_Torque(const float value[4])
{
    memcpy(fixture_sent_dm, value, sizeof(fixture_sent_dm));
    return fixture_dm_status;
}
static HAL_StatusTypeDef Dji_Send_Wheel_Torque(float left, float right)
{
    fixture_sent_wheel[0] = left;
    fixture_sent_wheel[1] = right;
    return HAL_OK;
}
"""

ARM_MATH_SHIM = r"""
#ifndef TEST_ARM_MATH_H
#define TEST_ARM_MATH_H
#include <math.h>
static inline float arm_sin_f32(float x) { return sinf(x); }
static inline float arm_cos_f32(float x) { return cosf(x); }
static inline int arm_sqrt_f32(float x, float *value)
{
    *value = sqrtf(x);
    return 0;
}
#endif
"""

HARNESS = r"""
static leg_state_t left_leg, right_leg;
static rl_torque_param_t parameters;
static const float actions[6] = {0.3f, -0.1f, 3.5f, -0.2f, 0.15f, -3.0f};
static const float wheel_velocity[2] = {1.0f, -2.0f};

static void near(float actual, float expected)
{
    if (!isfinite(actual) || !isfinite(expected) || fabsf(actual - expected) > 0.0001f)
    {
        fprintf(stderr, "actual=%g expected=%g\n", actual, expected);
        assert(0);
    }
}

static float reference_clip(float value, float bound)
{
    return fminf(fmaxf(value, -bound), bound);
}

static void fixture_setup(void)
{
    unsigned i;

    fixture_machine = machine_table[MACHINE_DEFAULT];
    fixture_machine.dm_trq_clamp = 40.0f;
    fixture_machine.dji_trq_clamp = 3.9f;
    fixture_machine.rl.configured = 1u;
    for (i = 0u; i < 6u; i++)
    {
        fixture_machine.rl.sign[i] = i < 3u ? -1 : 1;
    }
    fixture_machine.rl.zero[1] = fixture_machine.rl.zero[3] = 3.08f;
    memset(&left_leg, 0, sizeof(left_leg));
    left_leg.config.configured = 1u;
    left_leg.config.lu = 0.21f;
    left_leg.config.lg = 0.25f;
    left_leg.input.hip_f = 2.0f;
    left_leg.input.hip_b = 0.4f;
    left_leg.input.d_hip_f = 0.3f;
    left_leg.input.d_hip_b = -0.2f;
    right_leg = left_leg;
    right_leg.input.hip_f = 2.2f;
    right_leg.input.hip_b = 0.65f;
    right_leg.input.d_hip_f = -0.1f;
    right_leg.input.d_hip_b = 0.2f;
    assert(Leg_Solve(&left_leg) && left_leg.output.force_valid);
    assert(Leg_Solve(&right_leg) && right_leg.output.force_valid);
    memset(&parameters, 0, sizeof(parameters));
    parameters.dof_pos[0] = left_leg.output.thigh_angle + 0.1f;
    parameters.dof_pos[1] = left_leg.output.virtual_shank_angle + 0.05f;
    parameters.dof_pos[3] = right_leg.output.thigh_angle - 0.07f;
    parameters.dof_pos[4] = right_leg.output.virtual_shank_angle - 0.03f;
    parameters.p_gains[0] = 7.0f;
    parameters.p_gains[1] = 11.0f;
    parameters.p_gains[3] = 13.0f;
    parameters.p_gains[4] = 17.0f;
    parameters.d_gains[0] = 1.0f;
    parameters.d_gains[1] = 0.8f;
    parameters.d_gains[3] = 1.2f;
    parameters.d_gains[4] = 0.6f;
    parameters.wheel_pid[0][0] = 0.1f;
    parameters.wheel_pid[1][0] = 0.15f;
    memset(&lqr_debug, 0, sizeof(lqr_debug));
    lqr_debug.hip_enable = lqr_debug.wheel_enable = 1u;
    lqr_debug.trq_max_hip = 40.0f;
    lqr_debug.trq_max_wheel = 1.8f;
    torque_output_enabled = 1u;
    fixture_dm_status = HAL_OK;
}

static void reference_rl(float virtual_tau[6], float dm[4], float wheel[2])
{
    float q[6], velocity[6], target, error;
    unsigned i, side;

    q[0] = left_leg.output.thigh_angle;
    q[1] = left_leg.output.virtual_shank_angle;
    q[2] = q[5] = 0.0f;
    q[3] = right_leg.output.thigh_angle;
    q[4] = right_leg.output.virtual_shank_angle;
    velocity[0] = left_leg.input.d_hip_f;
    velocity[1] = left_leg.output.d_virtual_shank_angle;
    velocity[2] = wheel_velocity[0];
    velocity[3] = right_leg.input.d_hip_f;
    velocity[4] = right_leg.output.d_virtual_shank_angle;
    velocity[5] = wheel_velocity[1];
    for (i = 0u; i < 6u; i++)
    {
        if (i == 2u || i == 5u)
        {
            side = i == 2u ? 0u : 1u;
            target = actions[i] * 10.0f;
            virtual_tau[i] = reference_clip(parameters.wheel_pid[side][0] * (target - velocity[i]), machine->dji_trq_clamp);
            wheel[side] = reference_clip(virtual_tau[i], machine->dji_trq_clamp);
        }
        else
        {
            target = actions[i] * 0.5f + parameters.dof_pos[i];
            error = remainderf(target - q[i], 2.0f * LEG_PI);
            virtual_tau[i] = reference_clip(parameters.p_gains[i] * error
                - parameters.d_gains[i] * velocity[i], machine->dm_trq_clamp);
        }
    }
    dm[0] = virtual_tau[0] + virtual_tau[1] * left_leg.output.vshank_jac[1];
    dm[1] = virtual_tau[1] * left_leg.output.vshank_jac[0];
    dm[2] = virtual_tau[3] + virtual_tau[4] * right_leg.output.vshank_jac[1];
    dm[3] = virtual_tau[4] * right_leg.output.vshank_jac[0];
}

static uint8_t run_rl(torque_output_t *torque)
{
    rl_torque_state_t state;

    RL_Torque_State_Init(&state, &parameters);
    Torque_Output_Clear(torque);
    torque->valid = RL_Torque_Compute(&left_leg, &right_leg, &parameters,
        wheel_velocity, actions, &state, torque);
    return torque->valid;
}

static void make_lqr(lqr_state_t *state, leg_balance_t *balance)
{
    memset(state, 0, sizeof(*state));
    state->valid = 1u;
    state->leg_len_tgt[0] = left_leg.output.virtual_leg_length + 0.01f;
    state->leg_len_tgt[1] = right_leg.output.virtual_leg_length - 0.008f;
    state->roll = 0.02f;
    state->u[LQR_U_WL] = 1.2f;
    state->u[LQR_U_WR] = -2.4f;
    state->u[LQR_U_BL] = 0.9f;
    state->u[LQR_U_BR] = -1.3f;
    Leg_Balance_Init(balance);
}

static uint8_t run_lqr(lqr_state_t *state, leg_balance_t *balance, torque_output_t *torque)
{
    Torque_Output_Clear(torque);
    torque->valid = Leg_Balance_Compute(balance, state, &left_leg, &right_leg, CTRL_DT, torque);
    return torque->valid;
}


#if GAS_SPRING_COMP_ENABLE && MACHINE_DEFAULT == MACHINE_ID_BIG_WHEELLEG
#define EXPECT_GAS_ACTIVE 1
#else
#define EXPECT_GAS_ACTIVE 0
#endif

static float reference_spring(float length)
{
    double l = fmin(fmax((double)length, 0.05), 0.45);
    double theta = acos((0.21 * 0.21 + 0.25 * 0.25 - l * l) / (2.0 * 0.21 * 0.25));
    double s1 = 0.04863, s2 = 0.21210, phi = 0.213803;
    double spring_length = sqrt(s1 * s1 + s2 * s2 - 2.0 * s1 * s2 * cos(theta - phi));
    return (float)(150.0 * s1 * s2 * sin(theta - phi) * l /
        (0.21 * 0.25 * sin(theta) * spring_length));
}

static float expected_force(const leg_state_t *leg)
{
    return EXPECT_GAS_ACTIVE ? -reference_spring(leg->output.virtual_leg_length) : 0.0f;
}

static void assert_sent_zero(void)
{
    unsigned i;
    for (i = 0u; i < 4u; i++)
    {
        assert(fixture_sent_dm[i] == 0.0f && rl_output_dm_cmd_nm[i] == 0.0f);
    }
    for (i = 0u; i < 2u; i++)
    {
        assert(fixture_sent_wheel[i] == 0.0f && rl_output_wheel_cmd_nm[i] == 0.0f);
    }
}

static void rl_pd_and_model_baseline(void)
{
    torque_output_t torque;
    float virtual_tau[6], base[4], wheel[2], expected;
    const leg_state_t *leg[2];
    unsigned side, i;

    fixture_setup();
    leg[0] = &left_leg; leg[1] = &right_leg;
    reference_rl(virtual_tau, base, wheel);
    assert(run_rl(&torque));
    for (side = 0u; side < 2u; side++)
    {
        for (i = 0u; i < 2u; i++)
        {
            expected = base[side * 2u + i] + leg[side]->output.force_map[i][0] * expected_force(leg[side]);
            near(torque.dm[side * 2u + i], reference_clip(expected, machine->dm_trq_clamp));
        }
    }
    near(torque.dji[0], wheel[0]); near(torque.dji[1], wheel[1]);
    if (!EXPECT_GAS_ACTIVE)
    {
        left_leg.output.force_valid = right_leg.output.force_valid = 0u;
        left_leg.output.force_map[0][0] = right_leg.output.force_map[1][1] = NAN;
        assert(run_rl(&torque));
        for (i = 0u; i < 4u; i++)
        {
            near(torque.dm[i], reference_clip(base[i], machine->dm_trq_clamp));
        }
    }
}

static void rl_adds_only_force_once(void)
{
    torque_output_t torque;
    const leg_state_t *leg[2];
    float virtual_tau[6], base[4], wheel[2], force, delta[2], a, b, c, d, det;
    unsigned side;

    fixture_setup();
    leg[0] = &left_leg; leg[1] = &right_leg;
    reference_rl(virtual_tau, base, wheel);
    assert(run_rl(&torque));
    for (side = 0u; side < 2u; side++)
    {
        force = expected_force(leg[side]);
        a = leg[side]->output.force_map[0][0]; b = leg[side]->output.force_map[0][1];
        c = leg[side]->output.force_map[1][0]; d = leg[side]->output.force_map[1][1];
        det = a * d - b * c;
        assert(fabsf(base[side * 2u] + a * force) < machine->dm_trq_clamp);
        assert(fabsf(base[side * 2u + 1u] + c * force) < machine->dm_trq_clamp);
        delta[0] = torque.dm[side * 2u] - base[side * 2u];
        delta[1] = torque.dm[side * 2u + 1u] - base[side * 2u + 1u];
        near((d * delta[0] - b * delta[1]) / det, force);
        near((-c * delta[0] + a * delta[1]) / det, 0.0f);
        if (EXPECT_GAS_ACTIVE)
        {
            assert(force < -1.0f);
        }
    }
    near(torque.dji[0], wheel[0]); near(torque.dji[1], wheel[1]);
}

static void rl_compensates_before_limit(void)
{
    torque_output_t torque;
    float virtual_tau[6], base[4], wheel[2], delta, desired_thigh, wrong_order;
    unsigned i;

    fixture_setup();
    fixture_machine.dm_trq_clamp = 0.5f;
    reference_rl(virtual_tau, base, wheel);
    if (EXPECT_GAS_ACTIVE)
    {
        /* Build a mapped sum above the shared virtual/motor bound. */
        left_leg.output.vshank_jac[1] = -2.0f;
        left_leg.output.force_map[0][0] = -1.1f / expected_force(&left_leg);
        parameters.dof_pos[1] = left_leg.output.virtual_shank_angle - actions[1] * 0.5f
            + (-0.5f + parameters.d_gains[1] * left_leg.output.d_virtual_shank_angle) / parameters.p_gains[1];
        reference_rl(virtual_tau, base, wheel);
        delta = left_leg.output.force_map[0][0] * expected_force(&left_leg);
        assert(fabsf(delta) > 1.0f);
        desired_thigh = -delta - virtual_tau[1] * left_leg.output.vshank_jac[1];
        parameters.dof_pos[0] = left_leg.output.thigh_angle - actions[0] * 0.5f
            + (desired_thigh + parameters.d_gains[0] * left_leg.input.d_hip_f) / parameters.p_gains[0];
        reference_rl(virtual_tau, base, wheel);
        assert(run_rl(&torque));
        near(torque.dm[0], 0.0f);
        wrong_order = reference_clip(reference_clip(base[0], 0.5f) + delta, 0.5f);
        assert(fabsf(torque.dm[0] - wrong_order) > 0.4f);
    }
    else
    {
        assert(run_rl(&torque));
        for (i = 0u; i < 4u; i++)
        {
            near(torque.dm[i], reference_clip(base[i], 0.5f));
        }
    }
    near(torque.dji[0], wheel[0]); near(torque.dji[1], wheel[1]);
}

static void lqr_force_survives_length_gate(void)
{
    lqr_state_t state;
    leg_balance_t balance;
    torque_output_t torque;
    const leg_state_t *leg[2];
    float force, raw[2], tp;
    unsigned side, i;

    fixture_setup();
    leg[0] = &left_leg; leg[1] = &right_leg;
    lqr_debug.len_pid_enable = 0u;
    lqr_debug.trq_max_hip = 2.0f;
    make_lqr(&state, &balance);
    assert(run_lqr(&state, &balance, &torque));
    assert(fabsf(balance.leg_len[0].pos_out) > 0.1f);
    for (side = 0u; side < 2u; side++)
    {
        force = machine->lqr.support_force[side] + expected_force(leg[side]);
        tp = -state.u[LQR_U_BL + side];
        near(balance.F[side], machine->lqr.support_force[side]);
        near(balance.Tp[side], tp);
        assert(Leg_Force_Map_Forward(leg[side], force, tp, raw));
        for (i = 0u; i < 2u; i++)
        {
            near(torque.dm[side * 2u + i], reference_clip(raw[i], 2.0f));
        }
    }
    near(torque.dji[0], 1.2f); near(torque.dji[1], -1.8f);
    lqr_debug.hip_enable = 0u;
    assert(run_lqr(&state, &balance, &torque));
    assert(balance.Tp[0] == 0.0f && balance.Tp[1] == 0.0f);
    for (side = 0u; side < 2u; side++)
    {
        assert(Leg_Force_Map_Forward(leg[side], machine->lqr.support_force[side] + expected_force(leg[side]), 0.0f, raw));
        for (i = 0u; i < 2u; i++)
        {
            near(torque.dm[side * 2u + i], reference_clip(raw[i], 2.0f));
        }
    }
    Leg_Balance_Reset(&balance);
    assert(balance.F[0] == 0.0f && balance.F[1] == 0.0f && balance.Tp[0] == 0.0f);
    assert(balance.leg_len[0].pos_out == 0.0f && balance.roll.err[LAST] == 0.0f);
}

static void lqr_pid_baseline(void)
{
    lqr_state_t state;
    leg_balance_t balance;
    torque_output_t torque;
    const leg_state_t *leg[2];
    float force[2], roll, error, expected[2], tg;
    gravity_leg_result_t gravity;
    unsigned side, i, prime;

    for (prime = 0u; prime < 2u; prime++)
    {
        fixture_setup();
        leg[0] = &left_leg; leg[1] = &right_leg;
        lqr_debug.len_pid_enable = 1u;
        make_lqr(&state, &balance);
        balance.len_prime_enable = prime;
        roll = -(machine->lqr.roll.kp + machine->lqr.roll.kd) * state.roll;
        for (side = 0u; side < 2u; side++)
        {
            error = state.leg_len_tgt[side] - leg[side]->output.virtual_leg_length;
            error = (leg[side]->output.virtual_leg_length
                + reference_clip(error, machine->lqr.len_rate * MACHINE_CTRL_DT))
                - leg[side]->output.virtual_leg_length;
            force[side] = (machine->lqr.leg_len[side].kp
                + (prime ? 0.0f : machine->lqr.leg_len[side].kd)) * error
                + (side == 0u ? roll : -roll) + machine->lqr.support_force[side];
        }
        assert(run_lqr(&state, &balance, &torque));
        for (side = 0u; side < 2u; side++)
        {
            near(balance.F[side], force[side]);
            tg = 0.0f;
            if (machine->gravity != NULL)
            {
                assert(Gravity_Comp_Compute(machine->gravity, leg[side]->output.virtual_leg_length,
                    state.x[LQR_X_THL + side * 2u], &gravity));
                tg = gravity.torque;
            }
            assert(Leg_Force_Map_Forward(leg[side], force[side] + expected_force(leg[side]), -state.u[LQR_U_BL + side] + tg, expected));
            for (i = 0u; i < 2u; i++)
            {
                near(torque.dm[side * 2u + i], reference_clip(expected[i], lqr_debug.trq_max_hip));
            }
        }
        near(torque.dji[0], 1.2f); near(torque.dji[1], -1.8f);
    }
}

static void failures_dispatch_zero(void)
{
    lqr_state_t state;
    leg_balance_t balance;
    torque_output_t torque;
    float virtual_tau[6], base[4], wheel[2], force;
    unsigned scenario;

    for (scenario = 0u; scenario < 8u; scenario++)
    {
        if (scenario >= 4u && !EXPECT_GAS_ACTIVE)
        {
            continue;
        }
        fixture_setup();
        switch (scenario)
        {
        case 0: left_leg.output.valid = 0u; break;
        case 1: right_leg.output.vshank_jac[0] = NAN; break;
        case 2: right_leg.output.force_valid = 0u; break;
        case 3: right_leg.output.force_map[0][0] = NAN; break;
        case 4: right_leg.output.force_valid = 0u; break;
        case 5: right_leg.output.virtual_leg_length = NAN; break;
        case 6: right_leg.output.force_map[0][0] = NAN; break;
        default:
            parameters.dof_pos[1] += 1.0f;
            reference_rl(virtual_tau, base, wheel);
            assert(fabsf(virtual_tau[1]) > 1.0f);
            left_leg.output.vshank_jac[1] = (FLT_MAX * 0.75f) / virtual_tau[1];
            force = expected_force(&left_leg);
            left_leg.output.force_map[0][0] = (FLT_MAX * 0.75f) / force;
            reference_rl(virtual_tau, base, wheel);
            assert(isfinite(base[0]) && isfinite(left_leg.output.force_map[0][0] * force));
            break;
        }
        if (scenario == 2u || scenario == 3u)
        {
            make_lqr(&state, &balance);
            assert(!run_lqr(&state, &balance, &torque));
        }
        else
        {
            assert(!run_rl(&torque));
        }
        output_dispatch(&torque);
        assert_sent_zero();
    }
}

static void production_output_gate(void)
{
    torque_output_t torque;
    unsigned enabled, valid, i;

    fixture_setup();
    assert(run_rl(&torque));
    for (enabled = 0u; enabled < 2u; enabled++)
    {
        for (valid = 0u; valid < 2u; valid++)
        {
            torque_output_enabled = (uint8_t)enabled;
            torque.valid = (uint8_t)valid;
            output_dispatch(&torque);
            for (i = 0u; i < 4u; i++)
            {
                near(fixture_sent_dm[i], enabled && valid ? torque.dm[i] : 0.0f);
                near(rl_output_dm_cmd_nm[i], fixture_sent_dm[i]);
            }
            for (i = 0u; i < 2u; i++)
            {
                near(fixture_sent_wheel[i], enabled && valid ? torque.dji[i] : 0.0f);
            }
            assert(output_debug_dm_sent && output_debug_dji_sent);
        }
    }
    fixture_dm_status = HAL_ERROR;
    output_dispatch(&torque);
    assert(!output_debug_dm_sent && output_debug_dji_sent);
}

static void full_motor_output(void)
{
    const float full_actions[6] = {0.0f, 0.0f, 100.0f, 0.0f, 0.0f, -100.0f};
    const float stopped_wheels[2] = {0.0f, 0.0f};
    const float moderate_actions[6] = {0.0f, 0.0f, 3.0f, 0.0f, 0.0f, -3.0f};
    rl_torque_state_t controller;
    torque_output_t torque;
    unsigned i;

    fixture_setup();
    fixture_machine = machine_table[MACHINE_ID_BIG_WHEELLEG];
    near(machine->dm_trq_clamp, 54.0f);
    near(machine->dji_trq_clamp, 4.84375f);
    memset(left_leg.output.force_map, 0, sizeof(left_leg.output.force_map));
    memset(right_leg.output.force_map, 0, sizeof(right_leg.output.force_map));
    left_leg.output.vshank_jac[0] = right_leg.output.vshank_jac[0] = 1.0f;
    left_leg.output.vshank_jac[1] = right_leg.output.vshank_jac[1] = 0.0f;
    left_leg.input.d_hip_f = left_leg.output.d_virtual_shank_angle = -100.0f;
    right_leg.input.d_hip_f = right_leg.output.d_virtual_shank_angle = 100.0f;
    RL_Torque_State_Init(&controller, &parameters);
    Torque_Output_Clear(&torque);
    assert(RL_Torque_Compute(&left_leg, &right_leg, &parameters,
        stopped_wheels, full_actions, &controller, &torque));
    for (i = 0u; i < 4u; i++)
    {
        near(torque.dm[i], i < 2u ? 54.0f : -54.0f);
    }
    near(torque.dji[0], 4.84375f); near(torque.dji[1], -4.84375f);
    assert(Dji_Torque_To_Current(0u, torque.dji[0]) == 16384);
    assert(Dji_Torque_To_Current(1u, torque.dji[1]) == -16384);
    assert(Dji_Torque_To_Current(0u, 100.0f) == 16384);
    assert(Dm_Float_To_Uint(torque.dm[0], -54.0f, 54.0f, 12u) == 4095u);
    assert(Dm_Float_To_Uint(torque.dm[2], -54.0f, 54.0f, 12u) == 0u);

    RL_Torque_State_Init(&controller, &parameters);
    assert(RL_Torque_Compute(&left_leg, &right_leg, &parameters,
        stopped_wheels, moderate_actions, &controller, &torque));
    near(torque.dji[0], 3.0f); near(torque.dji[1], -4.5f);

    fixture_setup();
    fixture_machine = machine_table[MACHINE_ID_BIG_WHEELLEG];
    memset(left_leg.output.force_map, 0, sizeof(left_leg.output.force_map));
    memset(right_leg.output.force_map, 0, sizeof(right_leg.output.force_map));
    memset(parameters.p_gains, 0, sizeof(parameters.p_gains));
    parameters.d_gains[0] = parameters.d_gains[1] = 1.0f;
    left_leg.input.d_hip_f = -30.0f;
    left_leg.output.d_virtual_shank_angle = 25.0f;
    RL_Torque_State_Init(&controller, &parameters);
    assert(RL_Torque_Compute(&left_leg, &right_leg, &parameters,
        stopped_wheels, moderate_actions, &controller, &torque));
    near(torque.dm[0], 30.0f - 25.0f * left_leg.output.vshank_jac[1]);
    assert(torque.dm[0] > 40.0f && torque.dm[0] < 54.0f);
}

static void rl_control_period(void)
{
    rl_torque_state_t controller;
    torque_output_t torque;
    float error;
    float expected_dt;

    fixture_setup();
    expected_dt = MACHINE_DEFAULT == MACHINE_ID_BIG_WHEELLEG ? 0.002f : 0.001f;
    near(MACHINE_RL_CTRL_DT,expected_dt);
    RL_Torque_State_Init(&controller,&parameters);
    controller.controller[VJ_L_WHEEL].i=0.4f;
    controller.controller[VJ_L_WHEEL].IntegralLimit=1000.0f;
    error=actions[VJ_L_WHEEL]*10.0f-wheel_velocity[0];
    assert(RL_Torque_Compute(&left_leg,&right_leg,&parameters,
        wheel_velocity,actions,&controller,&torque));
    near(controller.controller[VJ_L_WHEEL].iout,0.4f*error*expected_dt);
    assert(RL_Torque_Compute(&left_leg,&right_leg,&parameters,
        wheel_velocity,actions,&controller,&torque));
    near(controller.controller[VJ_L_WHEEL].iout,2.0f*0.4f*error*expected_dt);
}

int main(int argc, char **argv)
{
    assert(argc == 2);
    switch (atoi(argv[1]))
    {
    case 0: rl_pd_and_model_baseline(); break;
    case 1: rl_adds_only_force_once(); break;
    case 2: rl_compensates_before_limit(); break;
    case 3: lqr_force_survives_length_gate(); break;
    case 4: lqr_pid_baseline(); break;
    case 5: failures_dispatch_zero(); break;
    case 6: production_output_gate(); break;
    case 7: full_motor_output(); break;
    case 8: rl_control_period(); break;
    default: assert(0); break;
    }
    return 0;
}

"""


class GasSpringIntegrationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = os.environ.get("CC") or shutil.which("gcc")
        if compiler is None:
            raise unittest.SkipTest("no native C compiler; set CC to host gcc")
        cls.temporary = tempfile.TemporaryDirectory(prefix="gas-spring-integration-")
        cls.addClassCleanup(cls.temporary.cleanup)
        folder = Path(cls.temporary.name)
        cls.environment = os.environ.copy()
        resolved = shutil.which(compiler) or compiler
        cls.environment["PATH"] = str(Path(resolved).resolve().parent) + os.pathsep + cls.environment.get("PATH", "")
        (folder / "arm_math.h").write_text(ARM_MATH_SHIM, encoding="utf-8")
        rl_source = read("imcalib/Algorithm/rl_torque.c")
        rl_preamble = "\n".join(re.findall(r"^#define RL_TQ_\w+[^\n]*", rl_source, re.M))
        rl_preamble += "\n" + re.search(r"enum\s*\{[^}]*\bVJ_NUM\b[^}]*\};", rl_source).group()
        dji_defines = "\n".join(re.findall(r"^#define DJI_\w+\s+[^\n]+", read("imcalib/user-lib/dji.h"), re.M))
        content = [
            HEADERS,
            production_enum(read("imcalib/user-lib/dm.h"), "dm_motor_idx_t"),
            production_enum(read("imcalib/user-lib/dji.h"), "dji_motor_type_t"),
            dji_defines,
            without_includes(read("imcalib/user-lib/machine_config.c")).split("const machine_cfg_t *const machine")[0],
            without_includes(read("imcalib/Algorithm/torque_output.h")),
            without_includes(read("imcalib/Algorithm/rl_torque.h")),
            without_includes(read("imcalib/Algorithm/lqr_balance.h")),
            without_includes(read("imcalib/Algorithm/leg_balance.h")),
            STUBS,
            production_function(read("imcalib/user-lib/dji.c"), "Dji_Torque_To_Current"),
            production_function(read("imcalib/user-lib/dm.c"), "Dm_Float_To_Uint"),
            without_includes(read("imcalib/Algorithm/gas_spring.c")),
            without_includes(read("imcalib/Algorithm/gravity_comp.c")),
            rl_preamble,
            *[production_function(rl_source, name) for name in (
                "RL_Torque_Array_Finite", "RL_Torque_State_Init", "RL_Torque_Compute")],
            without_includes(read("imcalib/Algorithm/leg_balance.c")),
            "static compensation_debug_t pending_compensation;\nstatic uint8_t ctrl_strategy;\n"
            "static struct { uint8_t drive; } control_frame = {1};\nstatic uint32_t ctrl_fault;\n"
            "#define FAULT_NONE 0\n#define CTRL_STRATEGY_COMPENSATION 4\n",
            production_function(read("imcalib/task/task_actuation.c"), "output_dispatch"),
            HARNESS,
        ]
        harness = folder / "gas_integration.c"
        harness.write_text("\n".join(content), encoding="utf-8")
        cls.executables = {}
        for machine_id, machine_name in ((0, "big"), (1, "small")):
            for enabled in (0, 1):
                variant = f"{machine_name}_{enabled}"
                executable = folder / (variant + (".exe" if os.name == "nt" else ""))
                result = subprocess.run([
                    compiler, "-std=c99", "-Wall", "-Wextra", "-Werror", "-DLEG_TRIG_LIBM=1",
                    f"-DMACHINE_DEFAULT={machine_id}", f"-DGAS_SPRING_COMP_ENABLE={enabled}",
                    "-I", str(folder), "-I", str(ROOT / "imcalib/Algorithm"),
                    "-I", str(ROOT / "imcalib/user-lib"), str(harness),
                    str(ROOT / "imcalib/Algorithm/leg_solver.c"), str(ROOT / "imcalib/user-lib/pid.c"),
                    str(ROOT / "imcalib/user-lib/simple-function.c"),
                    "-lm", "-o", str(executable),
                ], capture_output=True, text=True, env=cls.environment)
                if result.returncode:
                    raise AssertionError(variant + "\n" + result.stdout + result.stderr)
                cls.executables[variant] = executable

    def run_case(self, case):
        for variant, executable in self.executables.items():
            with self.subTest(variant=variant):
                result = subprocess.run([str(executable), str(case)], capture_output=True, text=True,
                                        env=self.environment, timeout=15)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_rl_pd_baseline_off_and_model_increment_when_enabled(self):
        self.run_case(0)

    def test_rl_adds_only_force_once_and_preserves_wheels(self):
        self.run_case(1)

    def test_rl_compensation_precedes_final_motor_limit(self):
        self.run_case(2)

    def test_lqr_force_survives_length_pid_gate_and_preserves_torque(self):
        self.run_case(3)

    def test_lqr_pid_primed_startup_and_original_baseline(self):
        self.run_case(4)

    def test_failed_feedback_mapping_and_nonfinite_sum_dispatch_zero(self):
        self.run_case(5)

    def test_real_output_dispatch_retains_total_switch_and_valid_gate(self):
        self.run_case(6)

    def test_full_motor_output_reaches_wire_limits_and_preserves_mapped_torque(self):
        self.run_case(7)

    def test_rl_pid_uses_its_own_execution_period(self):
        self.run_case(8)

    def test_normal_vofa_layout_has_no_gas_page_hook(self):
        source = read("imcalib/task/task_comm.c")
        normal = production_function(source, "Robot_Control_Send_Vofa")
        self.assertIn("lqr_state.x", normal)
        self.assertIn("Vofa_Send(dbg,", normal)
        self.assertNotIn("Robot_Control_Gas_Vofa", source)


if __name__ == "__main__":
    unittest.main()
