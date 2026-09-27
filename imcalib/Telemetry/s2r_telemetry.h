#ifndef S2R_TELEMETRY_H
#define S2R_TELEMETRY_H

#include <stdint.h>
#include "s2r_wire.h"

/* ============================================================================
 * S2R1 整机诊断遥测 (gap 测试)
 * 自包含模块: 只读现有固件状态, 不改动任何现有结构体; 现有代码只在 commTask
 * 保留一个 S2R_Pump() 挂点, 其余全部在本目录内。
 * ==========================================================================*/

#define S2R_MOTOR_NUM       6u      /* 4 髋 + 2 轮 (协议电机序) */
#define S2R_TRACE_NUM       50u     /* CONTROL 前段追踪 float 数 */
#define S2R_OBS_SIZE        25u
#define S2R_HISTORY_SIZE    125u
#define S2R_ACTION_NUM      6u
#define S2R_LATENT_NUM      3u

/* 调试器回退: 0 → 待机切回旧 VOFA (仅电机失能、无会话、UART 就绪时生效) */
extern volatile uint8_t s2r_diagnostic_requested;
extern volatile uint32_t s2r_init_error;    /* bit0 启动 RNG 失败, bit1 META 溢出 */

/* 烧录默认值: 1 = 上电即由 S2R1 诊断遥测占口; 0 = 上电发旧 VOFA (台架调试用)
 * 2026-09-27: 台架查 RL 投入时曾临时置 0, 查完改回 1 (要再看 VOFA 就临时改 0 重编, 或调试器写 s2r_diagnostic_requested=0) */
#ifndef S2R_DIAGNOSTIC_DEFAULT
#define S2R_DIAGNOSTIC_DEFAULT 1u
#endif

/* ============================================================================
 * 速率档: 描述性遥测不需要 100 Hz 时的限流档 (采样仍在 commTask 1 kHz 跑)
 *   0 = 原设定   (待机 ≈58 kB/s, 投入 ≈85 kB/s; 需真 USB-TTL CH340/FT232)
 *   1 = 低速     (≈9.5 kB/s; 实测待机 40 s 零丢帧, 但贴着台架串口桥的天花板)
 *   2 = 稳健     (≈4.3 kB/s; 台架桥留 2 倍余量, 推荐)
 *   3 = 极低     (≈1.6 kB/s; 只验链路/看趋势)
 * 实测: 同一条 PowerDebugger (VID 303A) 桥, 6.25 kB/s 干净, 33.6 kB/s 丢一半字节,
 * 所以台架采集请留余量; 换真 USB-TTL 后改回 0。
 * ==========================================================================*/
#ifndef S2R_RATE_LOW
#define S2R_RATE_LOW   2u
#endif

#if S2R_RATE_LOW == 0u
#define S2R_CONTROL_PERIOD_US   10000u      /* CONTROL 100 Hz */
#define S2R_POLICY_PERIOD_US    0u          /* 0 = 每次推理都发 */
#define S2R_IMU_PERIOD_US       20000u      /* IMU 50 Hz 上限 */
#define S2R_HEALTH_PERIOD_US    100000u     /* HEALTH 10 Hz */
#define S2R_HISTORY_PERIOD_US   500000u     /* HISTORY 2 Hz */
#define S2R_META_REPEAT_US      5000000u    /* META 清单重发间隔 */
#define S2R_POLICY_HZ           100u
#define S2R_CONTROL_HZ          100u
#define S2R_IMU_HZ              50u
#define S2R_HEALTH_HZ           10u
#define S2R_HISTORY_HZ          2u
#elif S2R_RATE_LOW == 1u
#define S2R_CONTROL_PERIOD_US   100000u     /* CONTROL 10 Hz */
#define S2R_POLICY_PERIOD_US    100000u     /* POLICY 10 Hz */
#define S2R_IMU_PERIOD_US       200000u     /* IMU 5 Hz */
#define S2R_HEALTH_PERIOD_US    200000u     /* HEALTH 5 Hz */
#define S2R_HISTORY_PERIOD_US   1000000u    /* HISTORY 1 Hz */
#define S2R_META_REPEAT_US      30000000u
#define S2R_POLICY_HZ           10u
#define S2R_CONTROL_HZ          10u
#define S2R_IMU_HZ              5u
#define S2R_HEALTH_HZ           5u
#define S2R_HISTORY_HZ          1u
#elif S2R_RATE_LOW == 2u
#define S2R_CONTROL_PERIOD_US   200000u     /* CONTROL 5 Hz */
#define S2R_POLICY_PERIOD_US    200000u     /* POLICY 5 Hz */
#define S2R_IMU_PERIOD_US       500000u     /* IMU 2 Hz */
#define S2R_HEALTH_PERIOD_US    1000000u    /* HEALTH 1 Hz */
#define S2R_HISTORY_PERIOD_US   5000000u    /* HISTORY 0.2 Hz */
#define S2R_META_REPEAT_US      20000000u   /* META 20 s 重发一轮 */
#define S2R_POLICY_HZ           5u
#define S2R_CONTROL_HZ          5u
#define S2R_IMU_HZ              2u
#define S2R_HEALTH_HZ           1u
#define S2R_HISTORY_HZ          1u
#elif S2R_RATE_LOW == 3u
#define S2R_CONTROL_PERIOD_US   500000u     /* CONTROL 2 Hz */
#define S2R_POLICY_PERIOD_US    500000u     /* POLICY 2 Hz */
#define S2R_IMU_PERIOD_US       1000000u    /* IMU 1 Hz */
#define S2R_HEALTH_PERIOD_US    2000000u    /* HEALTH 0.5 Hz */
#define S2R_HISTORY_PERIOD_US   10000000u   /* HISTORY 0.1 Hz */
#define S2R_META_REPEAT_US      60000000u
#define S2R_POLICY_HZ           2u
#define S2R_CONTROL_HZ          2u
#define S2R_IMU_HZ              1u
#define S2R_HEALTH_HZ           1u
#define S2R_HISTORY_HZ          1u
#else
#error "S2R_RATE_LOW must be 0, 1, 2 or 3"
#endif

/* ============================================================================
 * 板端短窗录制 (md §10a, 与速率档无关)
 * 触发时按 md 速率 (100 Hz) 把 CONTROL 的完整快照存进片内缓存, 缓存满即停采,
 * 之后以远低于链路上限的速度回放导出; 帧格式/序号/时间戳规则不变, 上位机无需改。
 * ==========================================================================*/
#define S2R_RECORD_BYTES        (192u * 1024u)  /* AXI SRAM 内缓存 (≈4 s @100 Hz) */
#define S2R_RECORD_PERIOD_US    10000u          /* 录制采样周期: md 的 100 Hz */
#define S2R_DUMP_PERIOD_US      150000u         /* 回放节流: ≈3.1 kB/s, 留足链路余量 */
/* 帧间强制空隙: 每帧发完后至少空这么久再发下一帧。
 * 台架实测: 队列有积压时固件会以线速连着推好几帧, 串口桥缓冲溢出 → 0.9 的帧丢掉;
 * 留出空隙后瞬时速率被压到 ~85 kB/s, 远低于线速但足够本项目所有档位。 */
#ifndef S2R_TX_GAP_US
#define S2R_TX_GAP_US           1500u
#endif
/* 触发方式: 1 = 固件自动 (建会话即开录, 会话结束再录 200 ms 尾巴后自动导出, 无需调试器);
 *           0 = 只认调试器写的 s2r_record_requested */
#ifndef S2R_RECORD_AUTO
#define S2R_RECORD_AUTO         1u
#endif
#define S2R_RECORD_TAIL_US      200000u         /* 失能后继续录的尾巴 */

/* 调试器写: 1 = 开始录制 (清缓存); 0 = 提前停止 → 自动回放 */
extern volatile uint8_t s2r_record_requested;
/* 调试器写: 1 = 把缓存里的这一轮再回放一次 (补丢帧) */
extern volatile uint8_t s2r_replay_requested;
/* 只读: 0 空闲 / 1 录制中 / 2 回放中; 已录制帧数 */
extern volatile uint8_t s2r_record_state;
extern volatile uint32_t s2r_record_frames;

/* ============================================================================
 * 采样数据类型 (由 s2r_source.c 填充, 与协议 CONTROL 布局一一对应)
 * ==========================================================================*/
typedef struct {
    uint64_t now_us;                        /* 本拍模块时间 */
    uint64_t motor_rx_us[S2R_MOTOR_NUM];    /* 首次见到新帧的时刻, 0 = 未收到 */
    uint32_t state_seq;                     /* 采样拍数 */
    uint32_t imu_seq;                       /* IMU 有效帧数 */
    uint32_t send_mask;                     /* 提交成功位 (本分支驱动未暴露, 恒 0) */
    uint32_t used_seq;                      /* 执行层本拍实际消费的策略序号 */
    uint8_t  rl_valid;                      /* RL 力矩链路本拍有效 */
    float    values[S2R_TRACE_NUM];         /* CONTROL 前段追踪量 */
    uint32_t wrap_mask;
    uint32_t motor_clamp;
    uint32_t virtual_clamp;
    float    q_motor[S2R_MOTOR_NUM];        /* 关节角反馈 */
    float    dq_motor[S2R_MOTOR_NUM];       /* 关节速度反馈 */
    float    tau_feedback[S2R_MOTOR_NUM];   /* 力矩反馈 (轮无反馈 = NaN) */
    float    tau_request[S2R_MOTOR_NUM];    /* 已下发力矩请求 */
} s2r_control_sample_t;

/* ============================================================================
 * 对外接口 (通信任务每拍调用)
 * ==========================================================================*/
uint8_t  S2R_Pump(void);        /* 返回 1 = 已占用遥测口, 调用方不要发旧 VOFA */
uint64_t S2R_Now_Us(void);      /* 模块时钟 (Mono_Ns_Get / 1000) */

/* ============================================================================
 * 适配层接口 (仅 s2r_source.c 调用)
 * ==========================================================================*/
uint32_t S2R_Published_Seq(void);
void S2R_Session_Update(uint8_t engaged);
void S2R_History_Reset(void);
void S2R_Observation(uint8_t valid, uint8_t pushed, uint64_t sample_us);
void S2R_Policy_Begin(const float obs[S2R_OBS_SIZE],
                      const float history[S2R_HISTORY_SIZE],
                      const float command[3]);
void S2R_Policy_End(uint8_t ok, const float published[S2R_ACTION_NUM],
                    const float latent[S2R_LATENT_NUM]);
void S2R_Imu_Record(uint32_t imu_seq, uint64_t rx_us);
void S2R_Control(const s2r_control_sample_t *sample);

#endif
