/* 仅主机替身测试: 直接驱动 S2R_Pump() + 采样适配层
 * 运行需要主机 C 编译器:  CC=gcc python tests/test_s2r.py
 * 本文件用替身取代现有固件全局量与 HAL 接口, 不链接固件工程。 */
#include "s2r_telemetry.h"
#include "robot_control.h"
#include "machine_config.h"
#include "hi229.h"
#include "mono_ns.h"
#include "dma_cache.h"
#include "Vofa_send.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

#define __get_PRIMASK() 0u
#define __disable_irq() ((void)0)
#define __set_PRIMASK(x) ((void)(x))
#define Dma_Cache_Clean_Tx(p, n) ((void)(p), (void)(n))
#include "../imcalib/Telemetry/s2r_telemetry.c"
#include "../imcalib/Telemetry/s2r_source.c"

/* ---- 现有固件全局量替身 ---- */
imu_state_t imu_state;
motor_state_t motor_state;
leg_state_t leg_l, leg_r;
action_state_t action_state;
input_command_t input_command;
robot_state_t robot_state;
rl_control_state_t rl_control;
volatile ctrl_strategy_t ctrl_strategy;
volatile uint32_t ctrl_fault;
uint8_t torque_output_enabled;
volatile float rl_output_dm_cmd_nm[DM_MOTOR_NUM];
volatile float rl_output_wheel_cmd_nm[DJI_MOTOR_NUM];
uint32_t dm_last_submit_mask;
dm_motor_feedback_t dm_motor_feedback[DM_MOTOR_NUM];
dji_motor_feedback_t dji_motor_feedback[DJI_MOTOR_NUM];
hi229_data_t hi229_data;
const dm_motor_config_t dm_motor_config[DM_MOTOR_NUM] = {{0}};
const dji_motor_config_t dji_motor_config[DJI_MOTOR_NUM] = {{0}};
static machine_cfg_t test_machine = {.name = "host-fixture"};
const machine_cfg_t *const machine = &test_machine;
UART_HandleTypeDef huart1, huart8;

static uint8_t rl_engaged_stub;
static uint64_t test_us = 1u;
static uint8_t dma_saved[S2R_FRAME_MAX];
static uint16_t dma_len;
static uint32_t dma_calls;
static uint8_t reject_dma;
static FILE *capture;

uint8_t Machine_Id(void) { return 0u; }
uint8_t output_task_rl_engaged(void) { return rl_engaged_stub; }
uint8_t output_task_lqr_engaged(void) { return 0u; }
uint64_t Mono_Ns_Get(void) { return test_us * 1000u; }
uint32_t HAL_GetTick(void) { return (uint32_t)(test_us / 1000u); }
/* boot_id 由 diag.initialized 旁路, 这里只提供链接符号 */
HAL_StatusTypeDef HAL_RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef *cfg)
{
    (void)cfg;
    return HAL_ERROR;
}

HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *uart, const uint8_t *data, uint16_t length)
{
    dma_calls++;
    if (reject_dma)
    {
        return HAL_BUSY;
    }
    memcpy(dma_saved, data, length);
    dma_len = length;
    uart->gState = HAL_UART_STATE_BUSY_TX;
    if (capture)
    {
        assert(fwrite(data, 1, length, capture) == length);
    }
    return HAL_OK;
}

static uint32_t u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/* 单拍 + 排空队列 (每拍最多发一帧) */
static void pump(void)
{
    uint32_t guard = 0u;
    do
    {
        (VOFA_UART)->gState = HAL_UART_STATE_READY;
        (void)S2R_Pump();
        guard++;
    } while ((queue.count || diag.pending) && guard < 64u);
}

/* 成功推理: 计数变化被采样层识别 */
static void infer_ok(void)
{
    rl_control.policy.run_ok++;
    rl_control.observation.valid = 1u;
    rl_control.observation.history_ready = 1u;
    action_state.rl_ready = 1u;
    action_state.updated = 1u;
}

int main(int argc, char **argv)
{
    uint32_t i, session, calls;
    FILE *meta;
    assert(argc == 3);
    capture = fopen(argv[1], "wb");
    assert(capture);
    /* boot_id 与真实时钟由台架验证: 这里旁路懒初始化 */
    memset(&diag, 0, sizeof(diag));
    diag.boot = 0x1020304050607080ULL;
    diag.enabled = 1u;
    diag.initialized = 1u;
    diag.health_time = 1u;
    diag.meta_restart = 1u;

    robot_state.motor_enabled = 1u;
    torque_output_enabled = 1u;
    imu_state.online = 1u;
    imu_state.quat[0] = 1.0f;
    for (i = 0u; i < DM_MOTOR_NUM; i++)
    {
        motor_state.dm.online[i] = 1u;
    }
    for (i = 0u; i < DJI_MOTOR_NUM; i++)
    {
        motor_state.dji.online[i] = 1u;
    }
    for (i = 0u; i < 125u; i++)
    {
        rl_control.observation.history[i] = (float)i;
    }
    for (i = 0u; i < 25u; i++)
    {
        rl_control.observation.obs[i] = (float)(100 + i);
    }
    for (i = 0u; i < 6u; i++)
    {
        test_machine.rl.sign[i] = 1;
    }
    test_machine.dm_trq_clamp = 40.0f;
    test_machine.dji_trq_clamp = 3.9f;

    /* 配置快照每 10 拍一次, 首次快照后立刻出 META */
    for (i = 0u; i < 10u; i++)
    {
        pump();
    }
    assert(diag.config_id == 1u && diag.session == 0u && !diag.meta_restart);

    /* 投入: 建会话 + 起始事件 */
    rl_engaged_stub = 1u;
    pump();
    assert(diag.session == 1u && (diag.flags & S2R_WARMUP) && !(diag.flags & S2R_STARTED));

    /* 观测 + 一次成功推理: 观测在成帧时冻结 */
    input_command.height_cmd = 0.2f;
    infer_ok();
    pump();
    assert(diag.published_seq == 1u && (diag.flags & S2R_STARTED));
    assert(policy_record.payload[0] == 1u);
    assert(u32(policy_record.payload + 40) == 0x42C80000u);   /* 冻结 obs[0] = 100 */
    rl_control.observation.obs[0] = 999.0f;
    assert(u32(policy_record.payload + 40) == 0x42C80000u);

    /* 控制帧: 采样追踪量 + 执行层实际消费的序号 */
    test_us += 10000u;
    leg_l.output.thigh_angle = 1.25f;
    rl_control.torque_state.virtual_torque[0] = 6.0f;
    rl_control.torque_state.controller[0].err[NOW] = 7.0f;
    rl_output_dm_cmd_nm[0] = 1.5f;
    pump();
    assert(u32(control_record.payload + 4) == 1u);            /* used_policy_seq */
    assert(u32(control_record.payload + 92) == 0u);           /* 提交位不可见 */
    assert(u32(control_record.payload + 96) == 0u);           /* 未饱和 → 无限幅位 */
    assert(u32(control_record.payload + 132) == 0x3FA00000u); /* trace[0] = 1.25 */
    assert(u32(control_record.payload + 260) == 0x40C00000u); /* trace[32] = 6.0 */

    /* IMU 帧: 新帧检测 + 在线掩码 */
    hi229_data.online = true;
    hi229_data.ts = 11u;
    hi229_data.last_rx_tick = 1u;
    hi229_data.quat[0] = 1.0f;
    pump();
    assert(u32(imu_record.payload) == 1u);
    assert(u32(imu_record.payload + 12) == 31u);
    assert(S2R_Published_Seq() == 1u);

    /* 推理失败: 清 ACTIVE 且不发布 */
    rl_control.policy.run_fail++;
    pump();
    assert(diag.policy_seq == 2u && !diag.published_seq && !(diag.flags & S2R_ACTIVE));

    /* 观测失效 → 历史重置并轮转会话段 */
    rl_control.observation.valid = 0u;
    pump();
    assert(diag.session == 2u);
    rl_control.observation.valid = 1u;
    infer_ok();
    pump();
    session = diag.session;
    assert(session == 2u && S2R_Published_Seq() == 1u);

    /* 配置变化: 重建快照 + 重开会话 */
    rl_control.param.obs_dof_pos[0] = 1.0f;
    for (i = 0u; i < 10u; i++)
    {
        pump();
    }
    assert(diag.config_id == 2u);

    /* 故障位传播 */
    ctrl_fault = FAULT_IMU;
    pump();
    assert(status_flags() & S2R_FAULT);
    ctrl_fault = 0u;
    pump();
    assert(!(status_flags() & S2R_FAULT));

    /* DMA 忙/提交拒绝: 不得改写 DMA 拥有的字节 */
    test_us += 30000u;
    hi229_data.ts = 12u;
    hi229_data.last_rx_tick = 2u;
    (VOFA_UART)->gState = HAL_UART_STATE_READY;
    reject_dma = 1u;
    (void)S2R_Pump();                       /* 采样入队, 发送被拒 */
    assert(diag.pending);
    memcpy(dma_saved, dma_buffer, diag.pending_len);
    (void)S2R_Pump();
    assert(!memcmp(dma_saved, dma_buffer, diag.pending_len));
    reject_dma = 0u;
    (void)S2R_Pump();
    assert(!diag.pending);
    calls = dma_calls;
    (void)S2R_Pump();
    assert(dma_calls == calls && !memcmp(dma_buffer, dma_saved, dma_len));
    pump();

    /* META 分片收齐 (外部 Python 校验 JSON) */
    while (diag.meta_offset < diag.meta_len)
    {
        test_us += 300000u;
        (VOFA_UART)->gState = HAL_UART_STATE_READY;
        (void)S2R_Pump();
    }
    meta = fopen(argv[2], "wb");
    assert(meta && diag.meta_len < sizeof(meta_json) && !s2r_init_error);
    assert(fwrite(meta_json, 1, diag.meta_len, meta) == diag.meta_len);
    assert(!fclose(meta));

    /* 退出会话 → 重入产生新会话号 */
    rl_engaged_stub = 0u;
    pump();
    assert(!diag.session);
    rl_engaged_stub = 1u;
    pump();
    assert(diag.session == session + 1u);

    /* 投入中不允许切回旧 VOFA */
    s2r_diagnostic_requested = 0u;
    assert(S2R_Pump() && diag.enabled);
    s2r_diagnostic_requested = 1u;

    /* 失能且无会话时才允许切回 (返回 0 = 调用方发旧 VOFA) */
    rl_engaged_stub = 0u;
    pump();
    robot_state.motor_enabled = 0u;
    s2r_diagnostic_requested = 0u;
    (VOFA_UART)->gState = HAL_UART_STATE_READY;
    assert(!S2R_Pump());
    assert(!fclose(capture));
    puts("telemetry sampler/session/DMA/META mock: PASS");
    return 0;
}
