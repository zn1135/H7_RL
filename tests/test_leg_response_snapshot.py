"""Run the firmware's ordinary VOFA sender against deterministic host doubles."""

from pathlib import Path
import os
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "imcalib/task/task_comm.c"
PREAMBLE = r"""
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "leg_solver.h"
#include "rl_observation.h"
#define DM_MOTOR_NUM 4
typedef struct { int8_t dm_front, dm_rear; uint8_t configured; } leg_map_t;
typedef struct {
    float pos_zero_rad[4];
    float trq_nm[4];
    float vel_rad_s[4];
    uint64_t parsed_rx_ns[4];
    uint8_t online[4];
} dm_state_t;
static struct { dm_state_t dm; } motor_state;
static leg_map_t leg_map_l = {0, 1, 1}, leg_map_r = {2, 3, 1};
static leg_state_t leg_l, leg_r;
static volatile float rl_output_dm_cmd_nm[4];
static volatile uint8_t output_debug_dm_sent, vofa_leg_response_side;
static struct { float height_cmd; } input_command;
static struct { rl_observation_state_t observation; rl_observation_param_t param; } rl_control;
static struct { float joint_target[4]; uint64_t updated_ns; uint8_t valid; } rl_output_diag;
static struct { uint8_t active, inhibited; } leg_response_test;
static uint8_t active_side, fault_mask, torque_output_enabled, rl_engaged;
static uint32_t ctrl_fault;
static uint64_t now_ns = 100000000;
static unsigned lock_depth, inject_preemption, send_count;
static float received[24];
static void overwrite_output(void)
{
    rl_output_dm_cmd_nm[0] = 90.0f;
    rl_output_dm_cmd_nm[1] = 80.0f;
    output_debug_dm_sent = 0;
    leg_l.output.virtual_leg_length = 9.0f;
    leg_l.output.vshank_jac[0] = 9.0f;
    rl_output_diag.joint_target[0] = 9.0f;
    rl_control.observation.obs[RL_OBS_CMD_VX] = 9.0f;
    active_side = 1;
}
static void vTaskSuspendAll(void) { ++lock_depth; }
static int xTaskResumeAll(void)
{
    assert(lock_depth == 1);
    --lock_depth;
    if (inject_preemption) { overwrite_output(); }
    return 0;
}
static uint64_t Mono_Ns_Get(void) { assert(lock_depth == 1); return now_ns; }
static uint8_t output_leg_test_side(void) { return active_side; }
static uint8_t output_task_rl_engaged(void) { return rl_engaged; }
static uint8_t Dm_Has_Fault(uint8_t index)
{
    if (inject_preemption && !lock_depth) { overwrite_output(); }
    return (fault_mask >> index) & 1;
}
static uint8_t RL_Joint_Map(float pos[4], float vel[6])
{
    assert(!"diagnostic sender must not call RL_Joint_Map");
    return 0;
}
static uint8_t Vofa_Send(const float *data, uint8_t n)
{
    assert(lock_depth == 0);
    assert(n == 24);
    memcpy(received, data, sizeof(received));
    ++send_count;
    return 1;
}
"""
MODE_STUBS = r"""
static struct { uint8_t motor_enabled; } robot_state;
static uint8_t s2r_active, trace_active, joint_lock, joint_stream, transport_changed;
static unsigned trace_pumps, trace_discards, transport_allowed;
static void Dm_Parse(void) {}
static void Dji_Parse(void) {}
static void Motor_State_Update(void) {}
static void Leg_State_Update(void) {}
static void WS2812_RainbowBlink(void) {}
static void Remote_Control_Update(void) {}
static void JointUsb_Process(void) {}
static void Robot_Fallen_Update(void) {}
static void Robot_Fault_Update(void) {}
static void Robot_Enable_Update(void) {}
static void JointUsb_Pump(void) {}
static uint8_t JointUsb_ModeLock(void) { return joint_lock; }
static uint8_t JointUsb_StreamRequested(void) { return joint_stream; }
static uint8_t S2R_Pump(void) { return s2r_active; }
static void Vofa_Trace_Discard(void) { ++trace_discards; }
static uint8_t Vofa_Trace_Pump(void) { ++trace_pumps; return trace_active; }
static uint8_t Vofa_Transport_Update(uint8_t allowed)
{
    transport_allowed = allowed;
    return transport_changed;
}
"""
HARNESS = r"""
int main(int argc, char **argv)
{
    unsigned i, scenario, expected_flags = 1023;
    unsigned side;
    assert(argc == 2);
    scenario = (unsigned)atoi(argv[1]);
    if (scenario >= 100) {
        s2r_active = scenario == 102;
        trace_active = scenario == 101 || scenario == 102;
        joint_lock = scenario == 103;
        joint_stream = scenario == 104;
        robot_state.motor_enabled = scenario == 105;
        transport_changed = scenario == 106;
        comm_task_body();
        comm_task_body();
        assert(send_count == ((scenario == 101 || scenario == 102) ? 0 : 1));
        assert(trace_pumps == (scenario == 102 ? 0 : 2));
        assert(trace_discards == ((scenario == 102 || scenario == 106) ? 2 : 0));
        if (scenario != 102) {
            assert(transport_allowed == (scenario >= 103 && scenario <= 105 ? 0 : 1));
        }
        return 0;
    }
    for (i = 0; i < 4; ++i) {
        motor_state.dm.online[i] = 1;
        motor_state.dm.parsed_rx_ns[i] = now_ns - 2000000;
        motor_state.dm.pos_zero_rad[i] = (float)i + 0.125f;
        motor_state.dm.trq_nm[i] = (float)i + 0.75f;
        motor_state.dm.vel_rad_s[i] = (float)i + 1.25f;
    }
    leg_l.output.valid = leg_r.output.valid = 1;
    leg_l.output.virtual_leg_length = leg_r.output.virtual_leg_length = 0.25f;
    leg_l.output.virtual_leg_angle = 0.125f;
    leg_r.output.virtual_leg_angle = -0.125f;
    leg_l.output.thigh_angle = 0.25f;
    leg_l.output.virtual_shank_angle = 0.5f;
    leg_r.output.thigh_angle = -0.25f;
    leg_r.output.virtual_shank_angle = -0.5f;
    leg_l.output.vshank_jac[0] = 2.0f;
    leg_l.output.vshank_jac[1] = 0.5f;
    leg_r.output.vshank_jac[0] = -3.0f;
    leg_r.output.vshank_jac[1] = -0.75f;
    rl_output_diag.joint_target[0] = 10.25f;
    rl_output_diag.joint_target[1] = -8.5f;
    rl_output_diag.joint_target[2] = -10.25f;
    rl_output_diag.joint_target[3] = 8.5f;
    rl_output_diag.updated_ns = now_ns - 2000000;
    rl_output_diag.valid = 1;
    rl_control.observation.valid = 1;
    rl_control.param.configured = 1;
    rl_control.param.command_scale[0] = 2.0f;
    rl_control.param.command_scale[1] = 0.25f;
    rl_control.param.command_scale[2] = 5.0f;
    rl_control.observation.obs[RL_OBS_CMD_VX] = 3.0f;
    rl_control.observation.obs[RL_OBS_CMD_YAW_RATE] = -0.5f;
    rl_control.observation.obs[RL_OBS_CMD_HEIGHT] = 1.25f;
    input_command.height_cmd = 9.0f;
    rl_output_dm_cmd_nm[0] = 5.0f;
    rl_output_dm_cmd_nm[1] = 4.0f;
    rl_output_dm_cmd_nm[2] = -5.0f;
    rl_output_dm_cmd_nm[3] = -4.0f;
    output_debug_dm_sent = 1;
    robot_state.motor_enabled = torque_output_enabled = rl_engaged = 1;
    leg_response_test.active = 1;
    if (scenario == 1) { output_debug_dm_sent = 0; expected_flags &= ~32u; }
    if (scenario == 2) { leg_l.output.valid = 0; expected_flags &= ~4u; }
    if (scenario == 3) { motor_state.dm.parsed_rx_ns[0] = now_ns - 10000001; expected_flags &= ~5u; }
    if (scenario == 4) { fault_mask = 2; expected_flags &= ~6u; }
    if (scenario == 5) { inject_preemption = 1; }
    if (scenario == 6) { vofa_leg_response_side = 1; }
    if (scenario == 7) { active_side = vofa_leg_response_side = 1; }
    if (scenario == 8) { motor_state.dm.parsed_rx_ns[0] = 0; expected_flags &= ~5u; }
    if (scenario == 9) { motor_state.dm.parsed_rx_ns[0] = now_ns + 1; expected_flags &= ~5u; }
    if (scenario == 10) { motor_state.dm.parsed_rx_ns[0] = now_ns - 10000000; }
    if (scenario == 11) { rl_output_diag.updated_ns = now_ns - 10000001; expected_flags &= ~8u; }
    if (scenario == 12) { rl_output_diag.updated_ns = now_ns + 1; expected_flags &= ~8u; }
    if (scenario == 13) { active_side = vofa_leg_response_side = 2; expected_flags &= ~15u; }
    if (scenario == 14) { rl_output_diag.joint_target[0] = NAN; expected_flags &= ~8u; }
    if (scenario == 15) { rl_output_diag.valid = 0; expected_flags &= ~8u; }
    if (scenario == 16) { rl_control.observation.valid = 0; expected_flags &= ~16u; }
    if (scenario == 17) { rl_control.param.command_scale[1] = 0.0f; expected_flags &= ~16u; }
    if (scenario == 18) { rl_control.param.command_scale[1] = NAN; expected_flags &= ~16u; }
    if (scenario == 19) { rl_control.observation.obs[RL_OBS_CMD_HEIGHT] = NAN; expected_flags &= ~16u; }
    if (scenario == 20) { torque_output_enabled = 0; expected_flags &= ~256u; }
    if (scenario == 21) { robot_state.motor_enabled = 0; expected_flags &= ~64u; }
    if (scenario == 22) { rl_engaged = 0; expected_flags &= ~128u; }
    if (scenario == 23) { leg_response_test.active = 0; expected_flags &= ~512u; }
    if (scenario == 24) { leg_response_test.inhibited = 1; expected_flags |= 1024u; }
    if (scenario == 25) { ctrl_fault = 1; expected_flags |= 2048u; }
    if (scenario == 26) { motor_state.dm.trq_nm[0] = NAN; expected_flags &= ~5u; }
    if (scenario == 27) { motor_state.dm.pos_zero_rad[1] = NAN; expected_flags &= ~6u; }
    if (scenario == 28) { leg_l.output.virtual_leg_angle = NAN; expected_flags &= ~4u; }
    if (scenario == 29) { motor_state.dm.parsed_rx_ns[0] = rl_output_diag.updated_ns = now_ns; }
    if (scenario == 30) { rl_output_diag.updated_ns = now_ns - 10000000; }
    if (scenario == 31) { rl_control.param.configured = 0; expected_flags &= ~16u; }
    if (scenario == 32) { rl_control.param.command_scale[0] = -2.0f; }
    if (scenario == 33) { leg_l.output.vshank_jac[1] = NAN; expected_flags &= ~4u; }
    if (scenario == 34) { motor_state.dm.vel_rad_s[0] = NAN; expected_flags &= ~5u; }
    if (scenario == 35) { motor_state.dm.online[1] = 0; expected_flags &= ~6u; }
    side = active_side;
    Robot_Control_Send_Vofa();
    assert(send_count == 0);
    Robot_Control_Send_Vofa();
    assert(send_count == 1);
    assert(received[0] == 100.0f);
    assert((unsigned)received[15] == expected_flags);
    assert(received[1] == (float)side);
    assert(received[2] == ((expected_flags & 16) ? (scenario == 32 ? -1.5f : 1.5f) : 0.0f));
    assert(received[3] == ((expected_flags & 16) ? -2.0f : 0.0f));
    assert(received[4] == ((expected_flags & 16) ? 0.25f : 0.0f));
    assert(received[5] == ((expected_flags & 8) ? (side == 1 ? -10.25f : 10.25f) : 0.0f));
    assert(received[6] == ((expected_flags & 4) ? (side == 1 ? -0.25f : 0.25f) : 0.0f));
    assert(received[7] == ((expected_flags & 8) ? (side == 1 ? 8.5f : -8.5f) : 0.0f));
    assert(received[8] == ((expected_flags & 4) ? (side == 1 ? -0.5f : 0.5f) : 0.0f));
    assert(received[9] == ((expected_flags & 4) ? 0.25f : 0.0f));
    assert(received[10] == ((expected_flags & 4) ? (side == 1 ? -0.125f : 0.125f) : 0.0f));
    assert(received[11] == (side > 1 ? 0.0f : (side == 1 ? -5.0f : 5.0f)));
    assert(received[12] == (side > 1 ? 0.0f : (side == 1 ? -4.0f : 4.0f)));
    assert(received[13] == ((expected_flags & 1) ? (side == 1 ? 2.75f : 0.75f) : 0.0f));
    assert(received[14] == ((expected_flags & 2) ? (side == 1 ? 3.75f : 1.75f) : 0.0f));
    assert(received[16] == ((expected_flags & 1) ? (side == 1 ? 2.125f : 0.125f) : 0.0f));
    assert(received[17] == ((expected_flags & 2) ? (side == 1 ? 3.125f : 1.125f) : 0.0f));
    assert(received[18] == ((expected_flags & 1) ? (side == 1 ? 3.25f : 1.25f) : 0.0f));
    assert(received[19] == ((expected_flags & 2) ? (side == 1 ? 4.25f : 2.25f) : 0.0f));
    assert(received[20] == ((expected_flags & 4) ? (side == 1 ? -0.75f : 0.5f) : 0.0f));
    assert(received[21] == ((expected_flags & 4) ? (side == 1 ? -3.0f : 2.0f) : 0.0f));
    if (scenario == 8 || scenario == 9 || scenario == 13) {
        assert(received[22] == -1.0f);
    } else if (scenario == 3 || scenario == 10) {
        assert(fabsf(received[22] - 10.0f) < 0.00001f);
    } else {
        assert(received[22] == (scenario == 29 ? 0.0f : 2.0f));
    }
    assert(received[23] == (side > 1 ? -1.0f : 2.0f));
    return 0;
}
"""


class LegResponseSnapshotTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = os.environ.get("CC") or shutil.which("gcc") or shutil.which("clang")
        if compiler is None:
            raise unittest.SkipTest("no native C compiler; set CC")
        source = SOURCE.read_text(encoding="utf-8")
        begin = source.index("static void Robot_Control_Send_Vofa(void)")
        cls.temp = tempfile.TemporaryDirectory(prefix="leg-response-snapshot-")
        cls.addClassCleanup(cls.temp.cleanup)
        folder = Path(cls.temp.name)
        host = folder / "snapshot.c"
        host.write_text(PREAMBLE + MODE_STUBS + source[begin:] + HARNESS, encoding="utf-8")
        cls.executable = folder / ("snapshot.exe" if os.name == "nt" else "snapshot")
        cls.environment = os.environ.copy()
        cls.environment["PATH"] = str(Path(compiler).resolve().parent) + os.pathsep + cls.environment["PATH"]
        subprocess.run([compiler, "-std=c99", "-O1", "-I" + str(ROOT / "imcalib/Algorithm"),
                        str(host), "-lm", "-o", str(cls.executable)], check=True, env=cls.environment)

    def run_scenario(self, scenario):
        result = subprocess.run([str(self.executable), str(scenario)], capture_output=True,
                                text=True, env=self.environment)
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_snapshot_validity_and_coherence(self):
        for scenario in range(36):
            with self.subTest(scenario=scenario):
                self.run_scenario(scenario)

    def test_comm_protocol_priority_and_transport_switch_guards(self):
        for scenario in range(100, 107):
            with self.subTest(scenario=scenario):
                self.run_scenario(scenario)


if __name__ == "__main__":
    unittest.main()
