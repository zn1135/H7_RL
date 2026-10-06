"""Exercise the actual VOFA packing, stop capture and DMA busy path on a host."""
import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import unittest

from test_gas_spring_integration import ROOT, production_enum, production_function, read


FIXTURE = r"""
#include <assert.h>
#include <stdint.h>
#include <math.h>
#include <string.h>
#include "machine_config.h"
#define DR16_SW_UP 1u
#define DR16_SW_DOWN 2u
#define DR16_SW_MID 3u
#define FAULT_NONE 0u
#define CONTROL_TIME_VOFA_ENABLE 0
#define DMA_CACHE_LINE_SIZE 32u
static struct { uint8_t rc_enable, motor_enabled, fallen; } robot_state;
static struct { uint8_t online, s1; } rc_command;
static struct { uint8_t online, pitch_world_valid; float pitch_world, euler_rad[3], gyro_rad_s[3]; } imu_state;
static struct { struct { uint8_t online[4]; uint32_t last_rx_tick[4]; } dm;
                struct { uint8_t online[2]; } dji; uint32_t timestamp_ms; } motor_state;
static struct { uint8_t err_raw, rx_seen; } dm_motor_feedback[4];
static struct { struct { uint8_t valid; float virtual_leg_length, virtual_leg_angle, d_virtual_leg_length, d_virtual_leg_angle; } output; } leg_l, leg_r;
static struct { uint8_t phase, fault, enabled, need; float elapsed, stable;
                float length_cmd[2], angle_cmd[2], angle_speed_cmd[2], force[2], tp[2], raw_dm[4], support, blend;
                struct { float p; } angle_pos_pid[2], angle_speed_pid[2]; } standup_control;
static struct { float leg_len_tgt[2], target[10], x[10], u[4]; uint8_t valid, gain_valid; } lqr_state;
static struct { float F[2], Tp[2]; } leg_balance;
static uint32_t ctrl_fault;
static uint8_t gas_spring_only_enabled, output_debug_dm_sent, output_debug_dji_sent;
static volatile float rl_output_dm_cmd_nm[4];
static volatile float rl_output_wheel_cmd_nm[2];
static machine_cfg_t fixture_machine;
const machine_cfg_t *const machine = &fixture_machine;
static uint8_t usb_lock, usb_allowed, dma_ready = 1;
static uint8_t lqr_engaged;
static unsigned enables, disables, wheel_stops, send_count;
static uint8_t *dma_buffer;
static uint16_t dma_length;
static const uint8_t tail[4] = {0, 0, 0x80, 0x7f};
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
static uint8_t JointUsb_ModeLock(void) { return usb_lock; }
static uint8_t JointUsb_EnableAllowed(void) { return usb_allowed; }
static uint8_t DR16_Online(void) { return rc_command.online; }
static uint8_t Dm_Is_Enabled(uint8_t i) { return motor_state.dm.online[i] && dm_motor_feedback[i].err_raw == 1; }
static uint8_t output_task_lqr_engaged(void) { return lqr_engaged; }
static uint8_t output_task_rl_engaged(void) { return 0; }
static int Dm_All_Enable(void) { enables++; return 0; }
static int Dm_All_Disable(void)
{
    assert(vofa_stop_diag.latched);
    disables++;
    rl_output_dm_cmd_nm[0] = rl_output_dm_cmd_nm[1] = 0;
    return 0;
}
static int Dji_All_Stop(void) { wheel_stops++; return 0; }
static void Dm_Enable_Watchdog(void) {}
static void Dm_Disable_Watchdog(void) {}
static uint8_t Vofa_Transport_Ready(void) { return dma_ready; }
static uint8_t Vofa_Transport_Send(uint8_t *data, uint16_t len)
{
    assert(dma_ready); dma_buffer = data; dma_length = len;
    send_count++; dma_ready = 0; return 1;
}
typedef struct { uint32_t ErrorCode; } FDCAN_HandleTypeDef;
typedef struct { uint32_t LastErrorCode, DataLastErrorCode, BusOff, ErrorPassive, Warning,
    ProtocolException, RxFDFflag, RxBRSflag, RxESIflag; } FDCAN_ProtocolStatusTypeDef;
typedef struct { uint32_t TxErrorCnt, RxErrorCnt, RxErrorPassive, ErrorLogging; } FDCAN_ErrorCountersTypeDef;
static FDCAN_HandleTypeDef hfdcan1 = {0x12345678u};
static FDCAN_ProtocolStatusTypeDef hw_protocol = {3,6,0,1,1,0,1,1,0};
static FDCAN_ErrorCountersTypeDef hw_counters = {128,4,0,7};
static uint32_t fifo_free = 32;
static int HAL_FDCAN_GetProtocolStatus(const FDCAN_HandleTypeDef *h, FDCAN_ProtocolStatusTypeDef *p)
{ (void)h; *p=hw_protocol; return 0; }
static int HAL_FDCAN_GetErrorCounters(const FDCAN_HandleTypeDef *h, FDCAN_ErrorCountersTypeDef *p)
{ (void)h; *p=hw_counters; return 0; }
static uint32_t HAL_FDCAN_GetTxFifoFreeLevel(const FDCAN_HandleTypeDef *h)
{ (void)h; return fifo_free; }
static uint32_t Can_Bus_Rx_Count(uint8_t bus) { assert(bus==1);return 42; }
static uint32_t Can_Bus_Last_Rx_Id(uint8_t bus) { assert(bus==1);return 0x12; }
static void frame(float out[VOFA_MAX_CH])
{
    dma_ready = 1; Robot_Control_Send_Vofa();
    assert(dma_length == 39 * sizeof(float) + 4);
    assert(memcmp(dma_buffer + 39 * sizeof(float), tail, 4) == 0);
    memcpy(out, dma_buffer, 39 * sizeof(float));
}
static void setup(void)
{
    unsigned i;
    rc_command.online = robot_state.rc_enable = imu_state.online = 1;
    imu_state.pitch_world = .3f; imu_state.pitch_world_valid = 1;
    imu_state.euler_rad[0] = .1f; imu_state.euler_rad[1] = .2f;
    imu_state.gyro_rad_s[1] = .4f;
    leg_l.output.d_virtual_leg_angle = .8f; leg_r.output.d_virtual_leg_angle = -.1f;
    rc_command.s1 = DR16_SW_MID;
    output_debug_dm_sent = output_debug_dji_sent = 1;
    fixture_machine.rl.configured = 1;
    leg_l.output.valid = leg_r.output.valid = 1;
    leg_l.output.virtual_leg_length = .15f; leg_r.output.virtual_leg_length = .16f;
    leg_l.output.virtual_leg_angle = -.1f; leg_r.output.virtual_leg_angle = .2f;
    for (i = 0; i < 4; i++) { motor_state.dm.online[i] = 1; dm_motor_feedback[i].err_raw = 1; dm_motor_feedback[i].rx_seen=1; }
    motor_state.dji.online[0] = motor_state.dji.online[1] = 1;
    standup_control.phase = standup_control.enabled = standup_control.need = 1;
    standup_control.elapsed = 2.5f; standup_control.stable = .03125f;
    standup_control.length_cmd[0] = standup_control.length_cmd[1] = .15f;
    standup_control.support = .25f; standup_control.blend = .125f;
    standup_control.angle_cmd[0] = -.06f; standup_control.angle_cmd[1] = .06f;
    standup_control.angle_speed_cmd[0] = 3; standup_control.angle_speed_cmd[1] = -3;
    standup_control.angle_pos_pid[0].p = 40; standup_control.angle_speed_pid[0].p = 8;
    lqr_state.valid = lqr_state.gain_valid = 1;
    lqr_state.leg_len_tgt[0] = .21f; lqr_state.leg_len_tgt[1] = .22f;
    lqr_state.target[LQR_X_THL] = .4f; lqr_state.target[LQR_X_THR] = .5f;
    lqr_state.x[LQR_X_DS] = .3f; lqr_state.x[LQR_X_DTHB] = -.5f;
    lqr_state.u[LQR_U_BL] = -11; lqr_state.u[LQR_U_BR] = -12;
    leg_balance.F[0] = 111; leg_balance.F[1] = 112;
    leg_balance.Tp[0] = 11; leg_balance.Tp[1] = 12;
    rl_output_wheel_cmd_nm[0] = 2; rl_output_wheel_cmd_nm[1] = -2;
    leg_l.output.d_virtual_leg_length = -.01f; leg_r.output.d_virtual_leg_length = -.02f;
    standup_control.force[0] = -20; standup_control.force[1] = -30;
    for (i = 0; i < 4; i++) { standup_control.raw_dm[i] = 21.0f + i; }
    motor_state.timestamp_ms = 100;
    motor_state.dm.last_rx_tick[0] = 98; motor_state.dm.last_rx_tick[1] = 97;
    motor_state.dm.last_rx_tick[2] = 95; motor_state.dm.last_rx_tick[3] = 94;
    rl_output_dm_cmd_nm[0] = 12.5f; rl_output_dm_cmd_nm[1] = -9.5f;
    rl_output_dm_cmd_nm[2] = 3; rl_output_dm_cmd_nm[3] = -4;
    Robot_Enable_Update(); assert(robot_state.motor_enabled && enables == 1);
}
"""

CHECKS = {
    "packing_and_busy": r"""
int main(void)
{
    float out[VOFA_MAX_CH]; unsigned i, count; uint8_t previous[160];
    const float age[4]={2,3,5,6};
    setup();frame(out);
    assert(out[0]==255 && out[1]==253 && out[2]==0);
    assert(out[3]==1 && out[4]==0 && out[5]==2500 && out[6]==31.25f);
    for(i=0;i<4;i++) { assert(out[7+i]==age[i]);assert(out[11+i]==1);assert(out[21+i]==rl_output_dm_cmd_nm[i]); }
    assert(out[15]==32 && out[16]==128 && out[17]==4 && out[18]==3 && out[19]==6 && out[20]==102);
    assert(out[25]==.2f && out[26]==.4f && out[27]==.15f && out[28]==.16f && out[29]==.3f && out[30]==223);
    assert(out[31]==42 && out[32]==0x12 && out[33]==0x5678 && out[34]==0x1234 && out[35]==100);
    assert(out[36]==-.1f && out[37]==.2f && out[38]==7);
    memcpy(previous,dma_buffer,dma_length);count=send_count;
    standup_control.phase=4;standup_control.fault=4;fifo_free=0;
    hw_protocol.LastErrorCode=hw_protocol.DataLastErrorCode=7;
    Robot_Control_Send_Vofa();assert(send_count==count && memcmp(previous,dma_buffer,dma_length)==0);
    frame(out);assert(out[3]==4 && out[4]==4 && out[15]==0 && out[18]==3 && out[19]==6);
    dm_motor_feedback[3].rx_seen=0;frame(out);assert(out[10]==-1);
    leg_l.output.virtual_leg_angle=INFINITY;frame(out);assert(isnan(out[36]));
    return 0;
}
""",
    "offline_capture_and_rearm": r"""
int main(void)
{
    float out[VOFA_MAX_CH]; unsigned i;
    setup(); motor_state.timestamp_ms = 112;
    motor_state.dm.online[0] = motor_state.dm.online[1] = 0; ctrl_fault = 4;
    Robot_Enable_Update(); assert(!robot_state.motor_enabled && disables == 1 && wheel_stops == 1);
    frame(out); assert(out[0] == 243 && vofa_stop_diag.channel[0] == 14 && vofa_stop_diag.channel[1] == 15);
    assert(vofa_stop_diag.channel[4] == 4 && vofa_stop_diag.channel[5] == 12.5f && vofa_stop_diag.channel[6] == -9.5f && vofa_stop_diag.channel[7] == 97);
    ctrl_fault = 0; motor_state.timestamp_ms = 120;
    for (i = 0; i < 2; i++) { motor_state.dm.online[i] = 1; motor_state.dm.last_rx_tick[i] = 120; }
    Robot_Enable_Update(); assert(robot_state.motor_enabled && enables == 2);
    frame(out); assert(out[0] == 255 && vofa_stop_diag.channel[0] == 14 && vofa_stop_diag.channel[4] == 4 && vofa_stop_diag.channel[7] == 97);
    rc_command.s1 = DR16_SW_DOWN; robot_state.rc_enable = 0;
    Robot_Enable_Update(); frame(out); assert(vofa_stop_diag.channel[7] == 97);
    rc_command.s1 = DR16_SW_MID; robot_state.rc_enable = 1;
    Robot_Enable_Update(); frame(out);
    assert(vofa_stop_diag.channel[0] == 0 && vofa_stop_diag.channel[1] == 0 && vofa_stop_diag.channel[4] == 0 && vofa_stop_diag.channel[7] == 0);
    return 0;
}
""",
    "protection_and_pitch_capture": r"""
int main(void)
{
    float out[VOFA_MAX_CH];
    setup(); dm_motor_feedback[0].err_raw = 9; dm_motor_feedback[1].err_raw = 10;
    ctrl_fault = 4; robot_state.fallen = 1; output_debug_dm_sent = 0;
    Robot_Enable_Update(); frame(out);
    assert(out[0] == 255 && vofa_stop_diag.channel[2] == 9 && vofa_stop_diag.channel[3] == 10);
    assert(vofa_stop_diag.channel[4] == 4 && vofa_stop_diag.channel[7] == 19 && disables == 1);
    dm_motor_feedback[0].err_raw = dm_motor_feedback[1].err_raw = 1;
    ctrl_fault = 0; robot_state.fallen = 0; output_debug_dm_sent = 1;
    Robot_Enable_Update(); frame(out); assert(vofa_stop_diag.channel[2] == 9 && vofa_stop_diag.channel[3] == 10 && vofa_stop_diag.channel[7] == 19);
    return 0;
}
""",
    "startup_and_tick_wrap": r"""
int main(void)
{
    float out[VOFA_MAX_CH];
    ctrl_fault = 4; Robot_Enable_Update(); assert(!vofa_stop_diag.latched && disables == 0);
    ctrl_fault = 0; setup();
    motor_state.timestamp_ms = 3;
    motor_state.dm.last_rx_tick[0] = UINT32_MAX - 5;
    motor_state.dm.last_rx_tick[1] = UINT32_MAX - 7;
    Robot_Enable_Update(); frame(out); assert(vofa_stop_diag.channel[0] == 9 && vofa_stop_diag.channel[1] == 11 && vofa_stop_diag.channel[7] == 0);
    return 0;
}
""",
}


class VofaStopDiagnosticsTest(unittest.TestCase):
    def test_production_capture_and_frames(self):
        compiler = os.environ.get("CC") or shutil.which("gcc")
        if compiler is None:
            self.skipTest("no host gcc; set CC")
        env = os.environ.copy()
        env["PATH"] = str(Path(compiler).parent) + os.pathsep + env.get("PATH", "")
        source = read("imcalib/task/task_comm.c")
        diag = re.search(r"typedef struct \{\s*float channel\[8\];.*?static vofa_stop_diag_t[^;]*;", source, re.S).group()
        capacity = re.search(r"#define VOFA_MAX_CH\s+\d+u", read("imcalib/user-lib/Vofa_send.h")).group()
        prefix = "#include <stdint.h>\n" + capacity + "\n" + production_enum(read("imcalib/user-lib/dm.h"), "dm_motor_idx_t")
        prefix += '\n'.join(re.findall(r'enum\s*\{.*?\};', read('imcalib/Algorithm/lqr_balance.h'), re.S))
        prefix += '\n' + '\n'.join(re.findall(r'#define DJI_MOTOR_\w+\s+\d+u', read('imcalib/user-lib/dji.h'))) + '\n'
        prefix += re.search(r'#define LEG_2PI\s+[^\n]+', read('imcalib/Algorithm/leg_solver.h')).group() + '\n'
        fixture = FIXTURE
        split = fixture.index("static void frame(")
        functions = "\n".join(production_function(source, name) for name in (
            "Robot_Control_Vofa_Diag_Update", "Robot_Enable_Update", "Robot_Control_Send_Vofa"))
        functions = production_function(read("imcalib/user-lib/pid.c"), "Angle_Wrap_180") + "\n" + production_function(read("imcalib/user-lib/Vofa_send.c"), "Vofa_Send") + "\n" + functions
        content = prefix + "\n" + diag + "\n" + fixture[:split] + "\n" + functions + "\n" + fixture[split:]
        with tempfile.TemporaryDirectory(prefix="vofa-stop-") as temporary:
            path = Path(temporary)
            for name, checks in CHECKS.items():
                test = path / (name + ".c")
                test.write_text(content + checks, encoding="utf-8")
                for machine_id in (0, 1):
                    with self.subTest(case=name, machine=machine_id):
                        exe = path / (name + str(machine_id) + ".exe")
                        result = subprocess.run([compiler, "-std=c99", "-Wall", "-Wextra", "-Werror",
                            f"-DMACHINE_DEFAULT={machine_id}", "-I", str(ROOT / "imcalib/user-lib"),
                            str(test), "-o", str(exe)], capture_output=True, text=True,
                            encoding="utf-8", errors="replace", env=env)
                        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                        result = subprocess.run([str(exe)], capture_output=True, text=True,
                                                encoding="utf-8", errors="replace", env=env)
                        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
