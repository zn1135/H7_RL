#include "s2r_telemetry.h"
#include "s2r_source.h"
#include "robot_control.h"
#include "machine_config.h"
#include "hi229.h"
#include "mono_ns.h"
#include "dma_cache.h"
#include "Vofa_send.h"
#include "s2r_build_info.h"

#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdarg.h>

/* ============================================================================
 * S2R1 整机诊断遥测 (gap 测试)
 * 只读现有固件状态, 不修改任何现有结构体; 现有代码只在 commTask 调用 S2R_Pump()。
 * 采样与事件推断见 s2r_source.c, 本文件负责成帧、排队、CRC 与 DMA 发送。
 * ==========================================================================*/

volatile uint8_t s2r_diagnostic_requested = S2R_DIAGNOSTIC_DEFAULT;
volatile uint8_t s2r_record_requested;
volatile uint8_t s2r_replay_requested;
volatile uint8_t s2r_record_state;
volatile uint32_t s2r_record_frames;
volatile uint32_t s2r_init_error;

typedef struct {
    uint64_t boot;
    uint32_t session, next_session, flags, event_seq;
    uint32_t policy_seq, published_seq, obs_seq, history_seq, epoch;
    uint32_t control_seq, used_seq, imu_seq, state_seq;
    uint64_t obs_time, policy_start, policy_end, control_start;
    uint64_t last_policy, last_control, last_control_frame, last_imu_frame;
    uint64_t last_history, health_time, meta_time, meta_chunk_time;
    uint64_t imu_rx_us;
    uint64_t tx_ready_us;       /* 下一帧允许发送的最早时刻 (帧间空隙) */
    uint64_t motor_rx_us[S2R_MOTOR_NUM];
    uint32_t send_mask, wrap_mask, motor_clamp, virtual_clamp;
    float trace[S2R_TRACE_NUM];
    uint32_t h[25];
    uint32_t busy_total;
    uint8_t enabled, pending, trace_valid, initialized, policy_pending;
    uint16_t pending_len;
    uint16_t meta_len, meta_offset;
    uint32_t meta_id, config_hash, config_id, meta_session;
    uint8_t meta_restart;
    uint32_t fault;
} s2r_state_t;

static s2r_state_t diag;
static s2r_queue_t queue;
static s2r_record_t policy_record, history_record, control_record, imu_record;
static s2r_record_t comm_record;
static uint8_t dma_buffer[S2R_FRAME_MAX] __attribute__((aligned(DMA_CACHE_LINE_SIZE), section(".s2r_dma")));
static char meta_json[8192];

/* ============================================================================
 * 板端短窗录制缓存 (md §10a)
 * 存储格式 (紧凑字节流): u16 len | u8 type | u8 res | u32 flags | u32 session |
 *                       u64 t_us | payload[len]     合计 20 + len 字节
 * 只存已编码前的记录 (payload + 生成时刻/标志), 回放时由队列重新分配序号并编码。
 * ==========================================================================*/
#define S2R_RECORD_HEADER 20u
static uint8_t record_buffer[S2R_RECORD_BYTES];
static uint32_t record_used, record_read, record_frames, record_dump_frames;
static uint64_t record_last_frame, record_last_dump;
static uint8_t record_state;    /* 0 空闲 / 1 录制中 / 2 回放中 */

typedef struct {
    machine_cfg_t machine;
    rl_observation_param_t observation;
    rl_torque_param_t torque;
    leg_config_t legs[2];
    float pid[6][10];
    uint32_t model, machine_id;
} s2r_config_t;

static s2r_config_t active_config, policy_config, meta_config;

/* 配置快照: 只读机器表与 RL 控制参数 */
static void config_snapshot(s2r_config_t *out)
{
    uint32_t i, key = __get_PRIMASK();
    const pid_t *p;
    __disable_irq();
    out->machine = *machine;
    out->observation = rl_control.param;
    out->model = rl_control.policy.selected_model;
    out->machine_id = Machine_Id();
    out->torque = rl_control.torque_param[out->model];
    out->legs[0] = leg_l.config;
    out->legs[1] = leg_r.config;
    for (i = 0u; i < 6u; i++)
    {
        p = &rl_control.torque_state.controller[i];
        out->pid[i][0] = p->p;
        out->pid[i][1] = p->i;
        out->pid[i][2] = p->d;
        out->pid[i][3] = p->MaxOutput;
        out->pid[i][4] = p->IntegralLimit;
        out->pid[i][5] = p->deadband;
        out->pid[i][6] = p->max_err;
        out->pid[i][7] = p->angle_wrap;
        out->pid[i][8] = p->pid_mode;
        out->pid[i][9] = out->torque.d_gains[i];
    }
    __set_PRIMASK(key);
}

/* 短临界区 */
static uint32_t lock(void)
{
    uint32_t key = __get_PRIMASK();
    __disable_irq();
    return key;
}

static void unlock(uint32_t key)
{
    __set_PRIMASK(key);
}

uint64_t S2R_Now_Us(void)
{
    return Mono_Ns_Get() / 1000u;
}

static void maximum(uint32_t *value, uint32_t sample)
{
    if (sample > *value)
    {
        *value = sample;
    }
}

static void period(uint64_t now, uint64_t *previous, uint32_t base, uint32_t limit, uint32_t overrun)
{
    uint32_t dt;
    if (*previous)
    {
        dt = (uint32_t)(now - *previous);
        if (!diag.h[base + 3u] || dt < diag.h[base])
        {
            diag.h[base] = dt;
        }
        maximum(&diag.h[base + 1u], dt);
        diag.h[base + 2u] += dt;
        diag.h[base + 3u]++;
        if (dt > limit)
        {
            diag.h[overrun]++;
        }
    }
    *previous = now;
}

static void floats(uint8_t *p, const float *values, uint32_t count)
{
    uint32_t i;
    for (i = 0u; i < count; i++)
    {
        S2R_Put_F32(p + i * 4u, values ? values[i] : NAN);
    }
}

static uint32_t status_flags(void)
{
    uint32_t flags = diag.flags | S2R_SNAPSHOT_VALID;
    uint32_t i;
    uint8_t online = imu_state.online;
    if (robot_state.motor_enabled && torque_output_enabled)
    {
        flags |= S2R_OUTPUT_ENABLED;
    }
    for (i = 0u; i < DM_MOTOR_NUM; i++)
    {
        online = (uint8_t)(online && motor_state.dm.online[i]);
    }
    for (i = 0u; i < DJI_MOTOR_NUM; i++)
    {
        online = (uint8_t)(online && motor_state.dji.online[i]);
    }
    if (online)
    {
        flags |= S2R_ONLINE;
    }
    if (robot_state.fallen)
    {
        flags |= S2R_FALLEN;
    }
    if (ctrl_fault || s2r_init_error)
    {
        flags |= S2R_FAULT;
    }
    if (!(flags & S2R_OUTPUT_ENABLED) || !online || robot_state.fallen || ctrl_fault)
    {
        flags &= ~S2R_ACTIVE;
    }
    return flags;
}

static void record_init(s2r_record_t *r, uint8_t type, uint16_t len, uint64_t now)
{
    uint32_t key = lock();
    r->type = type;
    r->len = len;
    r->t_us = now;
    r->session = diag.session;
    r->flags = status_flags();
    unlock(key);
}

static void emit(s2r_record_t *r, uint8_t priority)
{
    uint32_t key = lock();
    if (diag.enabled && diag.boot)
    {
        (void)S2R_Queue_Push(&queue, r, priority);
    }
    unlock(key);
}

/* 事件可去重 */
static void event(uint16_t code, uint16_t reason, uint32_t detail)
{
    static s2r_record_t r;      /* 仅 commTask 上下文调用, 不放 1 KB 在栈上 */
    uint32_t key;
    uint32_t i;
    key = lock();
    record_init(&r, S2R_EVENT, 24u, S2R_Now_Us());
    S2R_Put_U32(r.payload, ++diag.event_seq);
    S2R_Put_U16(r.payload + 4, code);
    S2R_Put_U16(r.payload + 6, reason);
    S2R_Put_U32(r.payload + 8, diag.policy_seq);
    S2R_Put_U32(r.payload + 12, diag.control_seq);
    unlock(key);
    S2R_Put_U32(r.payload + 16, detail);
    S2R_Put_U32(r.payload + 20, 0u);
    if (code == 2u)
    {
        r.flags |= S2R_START_EDGE;
    }
    for (i = 0u; i < ((code == 2u || code == 3u) ? 3u : 1u); i++)
    {
        emit(&r, 1u);
    }
}

/* ============================================================================
 * 板端短窗录制: 存 / 取 / 状态机 (md §10a)
 * ==========================================================================*/
static uint32_t record_get_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static uint64_t record_get_u64(const uint8_t *p)
{
    return (uint64_t)record_get_u32(p) | ((uint64_t)record_get_u32(p + 4) << 32);
}

/* 存一条完整快照; 缓存满则停录并报告 (不能影响控制) */
static void record_store(const s2r_record_t *r)
{
    uint32_t need = (uint32_t)S2R_RECORD_HEADER + r->len;
    if (record_used + need > S2R_RECORD_BYTES)
    {
        record_state = 2u;
        record_read = 0u;
        record_dump_frames = 0u;
        record_last_dump = 0u;
        event(10u, 0u, record_frames);      /* 缓存满 */
        return;
    }
    S2R_Put_U16(record_buffer + record_used, r->len);
    record_buffer[record_used + 2u] = r->type;
    record_buffer[record_used + 3u] = 0u;
    S2R_Put_U32(record_buffer + record_used + 4u, r->flags);
    S2R_Put_U32(record_buffer + record_used + 8u, r->session);
    S2R_Put_U64(record_buffer + record_used + 12u, r->t_us);
    memcpy(record_buffer + record_used + S2R_RECORD_HEADER, r->payload, r->len);
    record_used += need;
    record_frames++;
}

/* 回放一条: 还原成记录塞进发送队列 (序号/CRC 由队列与编码器重新生成) */
static uint8_t record_dump_one(void)
{
    static s2r_record_t r;
    uint16_t len;
    uint32_t need;

    if (record_read + S2R_RECORD_HEADER > record_used)
    {
        return 0u;
    }
    len = (uint16_t)(record_buffer[record_read] | ((uint16_t)record_buffer[record_read + 1u] << 8));
    need = (uint32_t)S2R_RECORD_HEADER + len;
    if (len > S2R_PAYLOAD_MAX || record_read + need > record_used)
    {
        return 0u;
    }
    r.type = record_buffer[record_read + 2u];
    r.len = len;
    r.flags = record_get_u32(record_buffer + record_read + 4u) | S2R_REPLAYING;
    r.session = record_get_u32(record_buffer + record_read + 8u);
    r.t_us = record_get_u64(record_buffer + record_read + 12u);
    memcpy(r.payload, record_buffer + record_read + S2R_RECORD_HEADER, len);
    record_read += need;
    return S2R_Queue_Push(&queue, &r, 0u);
}

/* 每拍: 录制/回放状态机
 * 自动模式 (S2R_RECORD_AUTO=1, 默认): 建会话(投入)即开录, 会话结束(失能)后再录一小段尾巴
 * 即停 → 自动回放导出, 全程不需要调试器;
 * 调试器写 s2r_record_requested / s2r_replay_requested 仍可用作手动覆盖。 */
static void record_tick(uint64_t now)
{
    static uint8_t record_prev_request;
    static uint32_t record_prev_session;
    static uint64_t record_stop_at;
    static uint8_t record_manual;

#if S2R_RECORD_AUTO
    if (record_state == 0u && diag.session && !record_prev_session)
    {
        record_used = 0u;
        record_read = 0u;
        record_frames = 0u;
        record_dump_frames = 0u;
        record_last_frame = 0u;
        record_stop_at = 0u;
        record_state = 1u;
        event(9u, 1u, 0u);                  /* 录制开始 (reason 1 = 投入自动触发) */
    }
    if (record_state == 1u)
    {
        if (!diag.session && !record_stop_at)
        {
            record_stop_at = now + S2R_RECORD_TAIL_US;   /* 失能后再录一段尾巴 */
        }
        if (record_stop_at && now >= record_stop_at)
        {
            record_stop_at = 0u;
            record_manual = 0u;
            record_state = 2u;               /* → 自动回放导出 */
            record_read = 0u;
            record_dump_frames = 0u;
            record_last_dump = 0u;
            event(10u, 3u, record_frames);   /* reason 3 = 会话结束自动停止 */
        }
    }
#endif
    record_prev_session = diag.session;

    if (s2r_record_requested && !record_prev_request && record_state == 0u)
    {
        record_used = 0u;
        record_read = 0u;
        record_frames = 0u;
        record_dump_frames = 0u;
        record_last_frame = 0u;
        record_stop_at = 0u;
        record_state = 1u;
        record_manual = 1u;
        event(9u, 0u, 0u);                  /* 录制开始 (reason 0 = 手动) */
    }
    else if (!s2r_record_requested && record_manual && record_state == 1u)
    {
        record_state = 2u;                  /* 手动停止 → 回放 */
        record_read = 0u;
        record_dump_frames = 0u;
        record_last_dump = 0u;
        record_manual = 0u;
        event(10u, 1u, record_frames);
    }
    record_prev_request = s2r_record_requested;
    if (s2r_replay_requested && record_state != 1u && record_used)
    {
        s2r_replay_requested = 0u;
        record_state = 2u;                  /* 再回放一轮 (补丢帧) */
        record_read = 0u;
        record_dump_frames = 0u;
        record_last_dump = 0u;
        event(10u, 2u, record_frames);
    }
    if (record_state == 2u && now - record_last_dump >= S2R_DUMP_PERIOD_US)
    {
        record_last_dump = now;
        if (record_dump_one())
        {
            record_dump_frames++;
        }
        else
        {
            record_state = 0u;
            event(11u, 0u, record_dump_frames);   /* 回放结束 */
        }
    }
    s2r_record_state = record_state;
    s2r_record_frames = record_frames;
}

/* 独立随机源 */
static uint64_t boot_id(void)
{
    RCC_PeriphCLKInitTypeDef clk;
    uint32_t start, words[2], i;
    __HAL_RCC_HSI48_ENABLE();
    start = HAL_GetTick();
    while (__HAL_RCC_GET_FLAG(RCC_FLAG_HSI48RDY) == RESET)
    {
        if (HAL_GetTick() - start > 20u)
        {
            return 0u;
        }
    }
    memset(&clk, 0, sizeof(clk));
    clk.PeriphClockSelection = RCC_PERIPHCLK_RNG;
    clk.RngClockSelection = RCC_RNGCLKSOURCE_HSI48;
    if (HAL_RCCEx_PeriphCLKConfig(&clk) != HAL_OK)
    {
        return 0u;
    }
    __HAL_RCC_RNG_CLK_ENABLE();
    __HAL_RCC_RNG_FORCE_RESET();
    __HAL_RCC_RNG_RELEASE_RESET();
    SET_BIT(RNG->CR, RNG_CR_CONDRST);
    RNG->HTCR = 0x17590ABCu;
    RNG->HTCR = 0x00007274u;
    CLEAR_BIT(RNG->CR, RNG_CR_CONDRST);
    start = HAL_GetTick();
    while (RNG->CR & RNG_CR_CONDRST)
    {
        if (HAL_GetTick() - start > 20u)
        {
            return 0u;
        }
    }
    SET_BIT(RNG->CR, RNG_CR_RNGEN);
    for (i = 0u; i < 2u; i++)
    {
        start = HAL_GetTick();
        while (!(RNG->SR & RNG_SR_DRDY))
        {
            if (HAL_GetTick() - start > 20u || (RNG->SR & (RNG_SR_SECS | RNG_SR_CECS)))
            {
                CLEAR_BIT(RNG->CR, RNG_CR_RNGEN);
                return 0u;
            }
        }
        words[i] = RNG->DR;
    }
    CLEAR_BIT(RNG->CR, RNG_CR_RNGEN);
    return ((uint64_t)words[0] << 32) | words[1];
}

/* 首次 Pump 时自初始化 (不改 main.c) */
static void init_once(void)
{
    memset(&diag, 0, sizeof(diag));
    memset(&queue, 0, sizeof(queue));
    diag.boot = boot_id();
    s2r_init_error = diag.boot ? 0u : 1u;
    diag.enabled = 1u;
    diag.initialized = 1u;
    diag.health_time = S2R_Now_Us();
    diag.meta_restart = 1u;
}

/* 会话边沿: 投入建立会话, 退出结束会话并记录原因 */
void S2R_Session_Update(uint8_t engaged)
{
    uint32_t key;
    uint8_t begin = 0u, end = 0u;
    key = lock();
    if (engaged && !diag.session)
    {
        diag.session = ++diag.next_session;
        diag.policy_seq = 0u;
        diag.published_seq = 0u;
        diag.obs_seq = 0u;
        diag.history_seq = 0u;
        diag.epoch++;
        diag.flags = S2R_WARMUP;
        diag.last_history = 0u;
        diag.meta_restart = 1u;
        begin = 1u;
    }
    else if (!engaged && diag.session)
    {
        diag.flags &= ~S2R_ACTIVE;
        end = 1u;
    }
    unlock(key);
    if (begin)
    {
        S2R_Source_Reset();
        event(1u, 0u, 0u);
    }
    if (end)
    {
        event(3u, robot_state.fallen ? 6u : (ctrl_fault ? 3u : (!robot_state.motor_enabled ? 2u : 1u)), ctrl_fault);
        key = lock();
        diag.session = 0u;
        diag.meta_restart = 1u;
        diag.flags = 0u;
        diag.published_seq = 0u;
        unlock(key);
    }
}

/* 观测历史失效: 清标记并按需重开会话段 */
void S2R_History_Reset(void)
{
    uint32_t key = lock();
    uint8_t restart = (uint8_t)(diag.session && (diag.flags & (S2R_HISTORY_VALID | S2R_STARTED)));
    diag.flags &= ~(S2R_ACTIVE | S2R_HISTORY_VALID | S2R_INFER_OK);
    diag.published_seq = 0u;
    unlock(key);
    if (restart)
    {
        event(6u, 4u, diag.epoch + 1u);
        S2R_Session_Update(0u);
        S2R_Session_Update(1u);
    }
}

void S2R_Observation(uint8_t valid, uint8_t pushed, uint64_t sample_us)
{
    uint32_t key = lock();
    diag.obs_seq++;
    diag.obs_time = sample_us;
    diag.flags &= ~(S2R_OBS_VALID | S2R_HISTORY_VALID);
    if (valid)
    {
        diag.flags |= S2R_OBS_VALID;
    }
    if (pushed)
    {
        diag.history_seq++;
        diag.flags |= S2R_HISTORY_VALID;
    }
    if (!valid)
    {
        diag.flags &= ~(S2R_ACTIVE | S2R_INFER_OK);
        diag.flags |= S2R_WARMUP;
        diag.published_seq = 0u;
    }
    unlock(key);
}

uint32_t S2R_Published_Seq(void)
{
    return diag.published_seq;
}

void S2R_Policy_Begin(const float obs[S2R_OBS_SIZE],
                      const float history[S2R_HISTORY_SIZE],
                      const float command[3])
{
    uint32_t key;
    uint64_t now = S2R_Now_Us();

    /* 回放期间不发 POLICY: 那时的 obs 是待机数据, 没用还挤链路 */
    if (record_state == 2u)
    {
        diag.policy_pending = 0u;
        return;
    }
    /* 限流档: 未到发送周期就不成帧 (序号不推进, 缺口统计保持干净) */
    if (S2R_POLICY_PERIOD_US && diag.last_policy
        && now - diag.last_policy < S2R_POLICY_PERIOD_US)
    {
        diag.policy_pending = 0u;
        return;
    }
    diag.policy_pending = 1u;
    record_init(&policy_record, S2R_POLICY, 212u, now);
    record_init(&history_record, S2R_HISTORY, 512u, policy_record.t_us);
    key = lock();
    diag.policy_seq++;
    diag.h[1]++;
    period(policy_record.t_us, &diag.last_policy, 3u,
        (uint32_t)((S2R_POLICY_PERIOD_US ? S2R_POLICY_PERIOD_US : 10000u) * 3u / 2u), 13u);
    S2R_Put_U32(policy_record.payload, diag.policy_seq);
    S2R_Put_U32(policy_record.payload + 4, diag.obs_seq);
    S2R_Put_U32(policy_record.payload + 8, diag.history_seq);
    S2R_Put_U32(policy_record.payload + 12, diag.epoch);
    S2R_Put_U64(policy_record.payload + 16, diag.obs_time);
    S2R_Put_U32(history_record.payload, diag.policy_seq);
    S2R_Put_U32(history_record.payload + 4, diag.history_seq);
    S2R_Put_U32(history_record.payload + 8, diag.epoch);
    unlock(key);
    floats(policy_record.payload + 40, obs, 25u);
    floats(policy_record.payload + 200, command, 3u);
    floats(history_record.payload + 12, history, 125u);
    diag.policy_start = policy_record.t_us;
    diag.policy_end = diag.policy_start;
    S2R_Put_U64(policy_record.payload + 24, diag.policy_start);
}

void S2R_Policy_End(uint8_t ok, const float published[S2R_ACTION_NUM],
                    const float latent[S2R_LATENT_NUM])
{
    uint32_t key;
    uint8_t first;
    uint64_t now = diag.policy_end;

    if (!diag.policy_pending)      /* 本拍被限流档跳过 */
    {
        return;
    }
    diag.policy_pending = 0u;
    S2R_Put_U64(policy_record.payload + 32, now);
    floats(policy_record.payload + 140, NULL, 6u);      /* 裁前网络动作未留存 */
    floats(policy_record.payload + 164, ok ? published : NULL, 6u);
    floats(policy_record.payload + 188, ok ? latent : NULL, 3u);
    key = lock();
    maximum(&diag.h[11], (uint32_t)(now - diag.policy_start));
    first = (uint8_t)(ok && !(diag.flags & S2R_STARTED));
    diag.flags &= ~(S2R_INFER_OK | S2R_WARMUP);
    if (ok)
    {
        diag.flags |= S2R_STARTED | S2R_INFER_OK;
        diag.published_seq = diag.policy_seq;
    }
    else
    {
        diag.flags &= ~S2R_ACTIVE;
        diag.published_seq = 0u;
        diag.h[18]++;
    }
    policy_record.flags = status_flags();
    history_record.flags = policy_record.flags;
    unlock(key);
    if (first)
    {
        event(2u, 0u, 0u);
    }
    emit(&policy_record, 0u);
    if (first || !diag.last_history || now - diag.last_history >= S2R_HISTORY_PERIOD_US)
    {
        emit(&history_record, 1u);
        diag.last_history = now;
    }
}
/* IMU 记录: 读 hi229 原始块与机体姿态 (冻结在采样时刻) */
void S2R_Imu_Record(uint32_t imu_seq, uint64_t rx_us)
{
    uint32_t i, j, mask = 0u, key;
    const float *groups[5] = {hi229_data.quat, hi229_data.gyr, hi229_data.acc,
                              imu_state.quat, imu_state.gyro_rad_s};
    const uint8_t sizes[5] = {4u, 3u, 3u, 4u, 3u};
    uint8_t valid;
    uint64_t now = S2R_Now_Us();

    key = lock();
    diag.imu_seq = imu_seq;         /* 帧龄基准按"最后一次收到帧"记, 与发送节流无关 */
    diag.imu_rx_us = rx_us;
    unlock(key);
    if (diag.last_imu_frame && now - diag.last_imu_frame < S2R_IMU_PERIOD_US)
    {
        return;
    }
    key = lock();
    diag.last_imu_frame = now;
    unlock(key);
    record_init(&imu_record, S2R_IMU, 88u, now);
    S2R_Put_U32(imu_record.payload, imu_seq);
    S2R_Put_U64(imu_record.payload + 4, rx_us);
    for (i = 0u; i < 5u; i++)
    {
        valid = i < 3u ? (uint8_t)hi229_data.online : imu_state.online;
        for (j = 0u; j < sizes[i]; j++)
        {
            valid = (uint8_t)(valid && isfinite(groups[i][j]));
        }
        if (valid)
        {
            mask |= 1u << i;
        }
    }
    S2R_Put_U32(imu_record.payload + 12, mask);
    S2R_Put_U32(imu_record.payload + 16, 0u);
    floats(imu_record.payload + 20, hi229_data.quat, 4u);
    for (i = 0u; i < 3u; i++)
    {
        S2R_Put_F32(imu_record.payload + 36 + i * 4, hi229_data.gyr[i] * 0.01745329251994f);
        S2R_Put_F32(imu_record.payload + 48 + i * 4, hi229_data.acc[i] * 9.80665f);
    }
    floats(imu_record.payload + 60, imu_state.quat, 4u);
    floats(imu_record.payload + 76, imu_state.gyro_rad_s, 3u);
    emit(&imu_record, 0u);
}

/* 执行层记录: 每条 CONTROL 帧由本拍采样得到;
 * 录制窗口内按 md 速率 (S2R_RECORD_PERIOD_US) 存进片内缓存, 不走串口 (md §10a) */
void S2R_Control(const s2r_control_sample_t *sample)
{
    uint32_t key, i, j, mask = 0u, field;
    uint64_t rx;
    uint8_t due, recording;
    uint32_t dt;
    uint8_t *p = control_record.payload;

    key = lock();
    diag.control_seq++;
    diag.state_seq = sample->state_seq;
    diag.imu_seq = sample->imu_seq;
    diag.send_mask = sample->send_mask;
    diag.used_seq = sample->used_seq;
    diag.control_start = sample->now_us;
    diag.trace_valid = sample->rl_valid;
    memcpy(diag.motor_rx_us, sample->motor_rx_us, sizeof(diag.motor_rx_us));
    memcpy(diag.trace, sample->values, sizeof(diag.trace));
    diag.wrap_mask = sample->wrap_mask;
    diag.motor_clamp = sample->motor_clamp;
    diag.virtual_clamp = sample->virtual_clamp;
    if (sample->rl_valid)
    {
        diag.h[22] |= sample->motor_clamp;
        diag.h[23] |= sample->virtual_clamp;
    }
    if (diag.imu_rx_us)
    {
        maximum(&diag.h[15], (uint32_t)(sample->now_us - diag.imu_rx_us));
    }
    for (i = 0u; i < S2R_MOTOR_NUM; i++)
    {
        rx = sample->motor_rx_us[i];
        if (rx)
        {
            maximum(&diag.h[16], (uint32_t)(sample->now_us - rx));
        }
    }
    /* 协议 POLICY_ACTIVE: 本拍执行层确实消费了已发布的策略动作 */
    diag.flags &= ~S2R_ACTIVE;
    if (sample->rl_valid && sample->used_seq && (diag.flags & S2R_STARTED)
        && diag.session && torque_output_enabled && robot_state.motor_enabled)
    {
        diag.flags |= S2R_ACTIVE;
    }
    due = (uint8_t)(!diag.last_control_frame
        || sample->now_us - diag.last_control_frame >= S2R_CONTROL_PERIOD_US);
    recording = (uint8_t)(record_state == 1u
        && (!record_last_frame
            || sample->now_us - record_last_frame >= S2R_RECORD_PERIOD_US));
    if (record_state != 0u)
    {
        due = 0u;   /* 录制/回放期间一律不发实时 CONTROL: 不挤链路, 也不混入实时帧 */
    }
    if (recording)
    {
        record_last_frame = sample->now_us;
    }
    if (due)
    {
        diag.h[2]++;
        dt = diag.last_control_frame
            ? (uint32_t)(sample->now_us - diag.last_control_frame) : 0u;
        diag.last_control_frame = sample->now_us;
        period(sample->now_us, &diag.last_control, 7u,
            (uint32_t)(S2R_CONTROL_PERIOD_US + S2R_CONTROL_PERIOD_US / 2u), 14u);
    }
    unlock(key);
    if (!due && !recording)
    {
        return;
    }

    record_init(&control_record, S2R_CONTROL, 452u, sample->now_us);
    S2R_Put_U32(p, diag.control_seq);
    S2R_Put_U32(p + 4, sample->rl_valid ? diag.used_seq : 0u);
    S2R_Put_U32(p + 8, sample->state_seq);
    S2R_Put_U32(p + 12, sample->imu_seq);
    S2R_Put_U32(p + 16, dt);
    S2R_Put_U32(p + 20, 0u);                /* 内环执行耗时本分支不可见 */
    S2R_Put_U64(p + 24, 0u);                /* DM 提交时刻不可见 */
    S2R_Put_U64(p + 32, 0u);                /* 轮提交时刻不可见 */
    for (i = 0u; i < S2R_MOTOR_NUM; i++)
    {
        field = 0u;
        rx = sample->motor_rx_us[i];
        j = (i < 4u) ? i : (i - 4u);
        if (i < 4u && motor_state.dm.online[i])
        {
            field = 0x07u | (rx ? 0x10u : 0u);
        }
        else if (i >= 4u && motor_state.dji.online[j])
        {
            field = 0x03u | (rx ? 0x10u : 0u);
        }
        S2R_Put_F32(p + 356 + i * 4, sample->tau_feedback[i]);
        S2R_Put_F32(p + 380 + i * 4, sample->q_motor[i]);
        S2R_Put_F32(p + 404 + i * 4, sample->dq_motor[i]);
        S2R_Put_F32(p + 428 + i * 4, NAN);  /* 电流无反馈 */
        S2R_Put_U64(p + 40 + i * 8, rx);
        S2R_Put_U32(p + 108 + i * 4, field);
        if (field)
        {
            mask |= 1u << i;
        }
    }
    S2R_Put_U32(p + 88, mask);
    S2R_Put_U32(p + 92, diag.send_mask);
    S2R_Put_U32(p + 96, sample->rl_valid ? sample->motor_clamp : 0u);
    S2R_Put_U32(p + 100, sample->rl_valid ? sample->virtual_clamp : 0u);
    S2R_Put_U32(p + 104, sample->rl_valid ? sample->wrap_mask : 0u);
    floats(p + 132, sample->rl_valid ? sample->values : NULL, S2R_TRACE_NUM);
    for (i = 0u; i < 4u; i++)
    {
        S2R_Put_F32(p + 332 + i * 4, sample->tau_request[i]);
    }
    for (i = 0u; i < 2u; i++)
    {
        S2R_Put_F32(p + 348 + i * 4, sample->tau_request[4u + i]);
    }
    if (recording)
    {
        record_store(&control_record);      /* 进片内缓存, 等采完慢速导出 */
    }
    else
    {
        emit(&control_record, 0u);
        maximum(&diag.h[12], (uint32_t)(S2R_Now_Us() - sample->now_us));
    }
}

/* 配置变化检测 (100 Hz: 每 10 拍一次, 与策略节拍同量级) */
static void config_check(void)
{
    static uint8_t div;
    uint32_t key, hash;
    uint8_t changed, running;

    if (++div < 10u)
    {
        return;
    }
    div = 0u;
    config_snapshot(&policy_config);
    hash = S2R_Crc32((const uint8_t *)&policy_config, sizeof(policy_config));
    key = lock();
    changed = (uint8_t)(diag.config_id && hash != diag.config_hash);
    running = (uint8_t)(diag.session != 0u);
    if (!diag.config_id || changed)
    {
        active_config = policy_config;
        diag.config_hash = hash;
        diag.config_id++;
        diag.meta_restart = 1u;
    }
    unlock(key);
    if (changed)
    {
        event(7u, 8u, diag.config_id);
        S2R_Session_Update(0u);
        S2R_Session_Update(running);
    }
}

/* META 在通信侧拼装 */
static void json_append(const char *format, ...)
{
    int count;
    va_list args;
    if (diag.meta_len >= sizeof(meta_json) - 1u)
    {
        return;
    }
    va_start(args, format);
    count = vsnprintf(meta_json + diag.meta_len, sizeof(meta_json) - diag.meta_len, format, args);
    va_end(args);
    if (count < 0 || (size_t)count >= sizeof(meta_json) - diag.meta_len)
    {
        s2r_init_error |= 2u;
        diag.meta_len = sizeof(meta_json) - 1u;
        return;
    }
    diag.meta_len += (uint16_t)count;
}

static void json_array(const char *name, const float *values, uint32_t n)
{
    uint32_t i;
    json_append("\"%s\":[", name);
    for (i = 0u; i < n; i++)
    {
        if (isfinite(values[i]))
        {
            json_append("%s%.9g", i ? "," : "", (double)values[i]);
        }
        else
        {
            json_append("%snull", i ? "," : "");
        }
    }
    json_append("],");
}

static void build_meta(void)
{
    float values[6];
    uint32_t i, key, config_id, config_hash;
    char name[16];
    const machine_cfg_t *m = &meta_config.machine;
    const rl_torque_param_t *p = &meta_config.torque;
    key = lock();
    meta_config = active_config;
    config_id = diag.config_id;
    config_hash = diag.config_hash;
    diag.meta_session = diag.session;
    diag.meta_restart = 0u;
    unlock(key);
    diag.meta_len = 0u;
    diag.meta_offset = 0u;
    diag.meta_id++;
    json_append("{\"protocol_version\":1,\"firmware\":%s,", S2R_BUILD_JSON);
    json_append("\"config_id\":%lu,\"machine\":\"%s\",\"machine_id\":%u,", (unsigned long)config_id, m->name, meta_config.machine_id);
    json_append("\"uart\":%d,\"baud\":1152000,\"format\":\"8N1\",\"policy_period_us\":%u,\"control_period_us\":%u,", (int)MACHINE_VOFA_PORT, (unsigned)(1000000u / S2R_POLICY_HZ), (unsigned)S2R_CONTROL_PERIOD_US);
    json_append("\"rate_profile\":%u,\"rates_hz\":{\"POLICY\":%u,\"CONTROL\":%u,\"IMU\":%u,\"HEALTH\":%u},\"history_period_us\":%u,", (unsigned)S2R_RATE_LOW, (unsigned)S2R_POLICY_HZ, (unsigned)S2R_CONTROL_HZ, (unsigned)S2R_IMU_HZ, (unsigned)S2R_HEALTH_HZ, (unsigned)S2R_HISTORY_PERIOD_US);
    json_append("\"period_semantics\":\"policy tick from TIM6 divided by MACHINE_POLICY_DIV; all periods sampled by commTask (1 ms quantization), actual period in HEALTH\",");
    json_append("\"source\":\"imcalib/Telemetry decoupled sampler, read-only; existing control code unchanged\",");
    json_append("\"sampled_at\":\"commTask 1 kHz\",\"control_frame\":\"period S2R_CONTROL_PERIOD_US of last actuation state; internal loop timing not observable\",");
    json_append("\"record\":{\"buffer_bytes\":%u,\"hz\":%u,\"dump_period_us\":%u,\"trigger\":\"auto: session begin (arm) starts, session end + tail stops, then slow dump; debugger writes override\",\"flags\":[\"RECORDING\",\"REPLAYING\"],\"event_codes\":{\"RECORD_START\":9,\"RECORD_STOP\":10,\"DUMP_END\":11}},",
        (unsigned)S2R_RECORD_BYTES, (unsigned)(1000000u / S2R_RECORD_PERIOD_US), (unsigned)S2R_DUMP_PERIOD_US);
    json_append("\"unavailable\":[\"action_raw\",\"tau_virtual_raw_fw\",\"gas_tau_shank_fw\",\"tau_motor_unclipped\",\"motor_send_ok_mask\",\"can_enqueue_us\",\"current_motor\",\"infer_exec_us\"],");
    json_append("\"derived\":[\"motor_clamp (request saturation)\",\"motor_rx_us (first-seen time)\",\"imu_rx_us (sample time)\"],");
    json_append("\"clock\":\"MCU Mono_Ns_Get / 1000, uint64 us\",\"boot_id_source\":\"hardware RNG HSI48\",");
    json_append("\"model\":%s,\"geometry_config_sha256\":\"%s\",", S2R_MODEL_JSON, S2R_CONFIG_SHA256);
    json_append("\"observation_size\":25,\"history_size\":125,\"action_size\":6,\"latent_scale\":2,\"forward_axis\":\"+x\",");
    json_append("\"history\":{\"order\":\"oldest_first\",\"includes_current\":true,\"initialization\":\"first obs repeated 5 times\",\"last_action\":\"previous published training action\"},");
    json_append("\"warmup_steps\":%u,\"obs_clip\":%.9g,\"action_clip\":%.9g,", RL_WARMUP_STEPS, (double)RL_OBS_CLIP, (double)RL_ACTION_CLIP);
    json_array("gyro_scale", meta_config.observation.gyro_scale, 3u);
    json_array("command_scale", meta_config.observation.command_scale, 3u);
    json_array("joint_vel_scale", meta_config.observation.joint_vel_scale, 6u);
    json_array("obs_default", meta_config.observation.obs_dof_pos, 4u);
    json_array("pd_default_fw", p->dof_pos, 6u);
    json_array("kp", p->p_gains, 6u);
    json_array("kd", p->d_gains, 6u);
    json_array("wheel_pid_left", p->wheel_pid[0], 3u);
    json_array("wheel_pid_right", p->wheel_pid[1], 3u);
    json_array("rl_zero", m->rl.zero, 4u);
    for (i = 0u; i < 6u; i++)
    {
        values[i] = (float)m->rl.sign[i];
    }
    json_array("rl_sign", values, 6u);
    json_append("\"runtime_config_crc32\":\"%08lx\",\"config_crc_scope\":\"MCU configuration snapshot bytes; build/source hash supplies ABI\",", (unsigned long)config_hash);
    json_append("\"selected_model\":%lu,\"pid_columns\":[\"kp\",\"ki\",\"kd_discrete\",\"max_output\",\"integral_limit\",\"deadband\",\"max_error\",\"wrap\",\"mode\",\"kd_velocity\"],", (unsigned long)meta_config.model);
    for (i = 0u; i < 6u; i++)
    {
        (void)snprintf(name, sizeof(name), "pid_%u", (unsigned)i);
        json_array(name, meta_config.pid[i], 10u);
    }
    json_append("\"pid_state_reset\":\"boot or model select; no automatic reset on ordinary disable\",\"output_ramp\":null,");
    json_append("\"observation_order\":[\"gyro_xyz*scale\",\"projected_gravity_xyz\",\"command_vx_yaw_height*scale\",\"q_L4-default\",\"dq_V6*scale\",\"last_action_V6\"],");
    json_append("\"V6\":[\"lf0\",\"lf1\",\"lfwheel\",\"rf0\",\"rf1\",\"rfwheel\"],\"L4\":[\"lf0\",\"lf1\",\"rf0\",\"rf1\"],\"M6\":[\"DM_FL\",\"DM_BL\",\"DM_FR\",\"DM_BR\",\"wheel_left\",\"wheel_right\"],");
    json_append("\"feedback_rates_hz\":null,\"imu_poll_us\":1000,\"motor_poll_us\":1000,\"imu_quaternion_direction\":\"body_to_world per observation inverse rotation\",");
    json_append("\"dji_command_conversion\":{\"raw_max\":[%u,%u],\"full_torque_nm\":[%.9g,%.9g],\"standard_ratio\":[%.9g,%.9g],\"source\":\"existing driver configured estimate, not measured current\"},", DJI_CURRENT_MAX_M2006, DJI_CURRENT_MAX_M3508, (double)DJI_NM_FULL_M2006, (double)DJI_NM_FULL_M3508, (double)DJI_RATIO_STD_M2006, (double)DJI_RATIO_STD_M3508);
    json_append("\"dji_can\":[[%u,%u],[%u,%u]],", dji_motor_config[0].feedback_id, dji_motor_config[0].control_id, dji_motor_config[1].feedback_id, dji_motor_config[1].control_id);
    for (i = 0u; i < 2u; i++)
    {
        values[0] = meta_config.legs[i].lu;
        values[1] = meta_config.legs[i].lg;
        values[2] = meta_config.legs[i].offset_phi0;
        values[3] = (float)meta_config.legs[i].configured;
        (void)snprintf(name, sizeof(name), "leg_config_%u", (unsigned)i);
        json_array(name, values, 4u);
    }
    json_append("\"leg_config_columns\":[\"lu\",\"lg\",\"offset_phi0\",\"configured\"],");
    json_append("\"pd\":{\"leg\":\"Kp*wrap(target-q)-Kd*dq\",\"pos_scale\":0.5,\"wheel_vel_scale\":10,\"virtual_limits\":[40,40,3.9,40,40,3.9]},");
    json_append("\"motor_limits\":[%.9g,%.9g],\"mit_ranges\":[%.9g,%.9g,%.9g],", (double)m->dm_trq_clamp, (double)m->dji_trq_clamp, (double)m->dm_pos_max, (double)m->dm_vel_max, (double)m->dm_trq_max);
    json_append("\"dm\":[");
    for (i = 0u; i < 4u; i++)
    {
        json_append("%s{\"rx_id\":%u,\"tx_id\":%u,\"bus\":%u,\"fb_sign\":%d,\"out_sign\":%d,\"zero\":%.9g}", i ? "," : "", dm_motor_config[i].feedback_id, dm_motor_config[i].control_id, m->dm_bus[i], m->dm_sign[i].fb, m->dm_sign[i].out, (double)m->dm_zero[i]);
    }
    json_append("],\"wheel_source_indices\":[1,0],\"dji_bus\":%u,\"dji_type\":%u,\"gear_ratio\":%.9g,\"wheel_radius\":%.9g,", m->dji_bus, m->dji_type, (double)m->dji_gear_ratio, (double)m->wheel_r);
    json_append("\"wheel_sign\":[[%d,%d],[%d,%d]],\"motor_units\":\"driver logical output axis, rad/rad_s/Nm\",", m->dji_sign[0].fb, m->dji_sign[0].out, m->dji_sign[1].fb, m->dji_sign[1].out);
    json_append("\"feedback\":{\"dm_torque\":\"driver MIT estimate\",\"wheel_torque\":null,\"motor_current_A\":null,\"accel_includes_gravity\":null},");
    json_array("gas_force_n", m->gas_spring_force_n, 2u);
    json_append("\"gas_sign\":[%d,%d],\"leg_lengths\":[%.9g,%.9g],", m->gas_comp_sign[0], m->gas_comp_sign[1], (double)m->leg_lu, (double)m->leg_lg);
    json_append("\"imu\":{\"quat_order\":\"wxyz\",\"quat_src\":[%u,%u,%u],\"quat_sign\":[%d,%d,%d],\"gyro_sign\":[%d,%d,%d],\"filter\":\"quaternion normalization, no additional telemetry filter\",\"rx_time\":\"commTask sampling time\"},", m->imu.quat_src[0], m->imu.quat_src[1], m->imu.quat_src[2], m->imu.quat_sign[0], m->imu.quat_sign[1], m->imu.quat_sign[2], m->imu.gyr_sign[0], m->imu.gyr_sign[1], m->imu.gyr_sign[2]);
    json_append("\"mass_kg\":null,\"payload\":null,\"ground_friction\":null,\"world_position\":null,\"true_linear_velocity\":null}");
}

static void health(uint64_t now)
{
    uint32_t i, key;
    record_init(&comm_record, S2R_HEALTH, 100u, now);
    key = lock();
    diag.h[0] = (uint32_t)(now - diag.health_time);
    diag.h[19] = queue.dropped;
    diag.h[20] = diag.busy_total;
    diag.h[21] = queue.peak_bytes;
    diag.h[24] = queue.generated;
    for (i = 0u; i < 25u; i++)
    {
        S2R_Put_U32(comm_record.payload + i * 4, diag.h[i]);
    }
    memset(diag.h, 0, sizeof(diag.h));
    queue.peak_bytes = queue.bytes;
    diag.health_time = now;
    unlock(key);
    emit(&comm_record, 0u);
}

uint8_t S2R_Pump(void)
{
    uint64_t now;
    uint32_t key, count;
    uint8_t desired;

    if (!diag.initialized)
    {
        init_once();
    }
    now = S2R_Now_Us();
    desired = s2r_diagnostic_requested ? 1u : 0u;
    if (desired != diag.enabled && !robot_state.motor_enabled && !diag.session
        && (VOFA_UART)->gState == HAL_UART_STATE_READY)
    {
        key = lock();
        queue.dropped += queue.count + (diag.pending ? 1u : 0u);
        queue.count = queue.read_idx = queue.write_idx = 0u;
        queue.bytes = 0u;
        diag.pending = 0u;
        diag.enabled = desired;
        diag.meta_restart = 1u;
        unlock(key);
    }
    if (!diag.enabled)
    {
        return 0u;
    }
    if (!diag.boot)
    {
        return 1u;
    }
    record_tick(now);
    S2R_Source_Tick();
    config_check();
    if (diag.fault != ctrl_fault)
    {
        event(ctrl_fault ? 4u : 5u, ctrl_fault ? 3u : 0u, ctrl_fault);
        diag.fault = ctrl_fault;
    }
    if (diag.meta_restart && diag.config_id)
    {
        build_meta();
        diag.meta_time = now;
    }
    else if (diag.meta_offset == diag.meta_len && now - diag.meta_time >= S2R_META_REPEAT_US)
    {
        diag.meta_offset = 0u;
        diag.meta_time = now;
    }
    if (!(s2r_init_error & 2u) && diag.meta_offset < diag.meta_len && now - diag.meta_chunk_time >= 300000u)
    {
        count = diag.meta_len - diag.meta_offset;
        if (count > 512u)
        {
            count = 512u;
        }
        record_init(&comm_record, S2R_META, (uint16_t)(12u + count), now);
        comm_record.session = diag.meta_session;
        S2R_Put_U32(comm_record.payload, diag.meta_id);
        S2R_Put_U16(comm_record.payload + 4, diag.meta_offset / 512u);
        S2R_Put_U16(comm_record.payload + 6, (diag.meta_len + 511u) / 512u);
        S2R_Put_U32(comm_record.payload + 8, diag.meta_len);
        memcpy(comm_record.payload + 12, meta_json + diag.meta_offset, count);
        emit(&comm_record, 1u);
        diag.meta_offset += (uint16_t)count;
        diag.meta_chunk_time = now;
    }
    if (now - diag.health_time >= S2R_HEALTH_PERIOD_US)
    {
        health(now);
    }
    if ((VOFA_UART)->gState != HAL_UART_STATE_READY)
    {
        diag.busy_total++;
        return 1u;
    }
    if (!diag.pending && now >= diag.tx_ready_us)   /* 帧间强制空隙: 防串口桥缓冲溢出 */
    {
        key = lock();
        count = S2R_Queue_Pop(&queue, &comm_record);
        unlock(key);
        if (!count)
        {
            return 1u;
        }
        diag.pending_len = S2R_Encode(dma_buffer, diag.boot, &comm_record);
        diag.pending = 1u;
    }
    if (!diag.pending)
    {
        return 1u;
    }
    Dma_Cache_Clean_Tx(dma_buffer, diag.pending_len);
    if (HAL_UART_Transmit_DMA(VOFA_UART, dma_buffer, diag.pending_len) == HAL_OK)
    {
        diag.pending = 0u;
        /* 预计发完时刻 + 强制空隙 */
        diag.tx_ready_us = now
            + ((uint64_t)diag.pending_len * 10u * 1000000u / 1152000u)
            + S2R_TX_GAP_US;
    }
    else
    {
        diag.busy_total++;
    }
    return 1u;
}
