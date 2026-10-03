"""Host-test the real RC/observation/policy-task path with a network test double.

This verifies data plumbing, warmup and action publication, not networkzn1 behavior.
"""

from pathlib import Path
import os
import re
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]


def production_type(source, name):
    match = re.search(r"typedef\s+struct\s*\{[^}]*\}\s*" + name + r"\s*;", source)
    if match is None:
        raise AssertionError("missing production type: " + name)
    return match.group()


def production_function(source, name):
    match = re.search(r"^(?:int16_t|float)\s+" + name + r"\([^)]*\)\s*\{", source, re.M)
    if match is None:
        raise AssertionError("missing production function: " + name)
    depth, end = 1, match.end()
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[match.start():end]


HEADERS = r"""
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "rc_command.h"
#include "leg_solver.h"
#include "imu_state.h"
#include "machine_config.h"
#include "rl_policy.h"
#define DM_MOTOR_NUM 4
#define DJI_MOTOR_NUM 2
#define DJI_MOTOR_WHEEL_LFT 0
#define DJI_MOTOR_WHEEL_RGT 1
"""


STUBS = r"""
static imu_state_t imu_state;
static motor_state_t motor_state;
static leg_state_t leg_l, leg_r;
static action_state_t action_state;
static input_command_t input_command;
static rc_command_t rc_command;
static struct {
    rl_observation_state_t observation;
    rl_observation_param_t param;
    rl_policy_t policy;
} rl_control;
static uint8_t fixture_engaged, network_success;
static uint8_t gas_spring_only_enabled;
static unsigned network_calls, trace_calls, network_init_calls;
static uint32_t fixture_tick;
static float captured_obs[RL_OBS_SIZE], captured_history[RL_OBS_HISTORY_SIZE];
static const float network_action[RL_ACTION_SIZE] = {0.125f, -0.25f, 0.375f,
                                                    -0.5f, 0.625f, -0.75f};
static uint32_t __get_PRIMASK(void) { return 0; }
static void __disable_irq(void) {}
static void __set_PRIMASK(uint32_t value) { (void)value; }
static uint32_t HAL_GetTick(void) { return fixture_tick; }
static uint64_t Mono_Ns_Get(void) { return (uint64_t)fixture_tick * 1000000; }
static uint8_t output_task_rl_engaged(void) { return fixture_engaged; }
uint8_t RL_Policy_Init(rl_policy_t *policy)
{
    network_init_calls++;
    policy->ready = 1;
    return 1;
}
uint8_t RL_Policy_Run(rl_policy_t *policy, const rl_observation_state_t *observation,
                      float action[RL_ACTION_SIZE])
{
    /* Capture the actual task input; deliberately do not run the learned model. */
    network_calls++;
    assert(observation->valid && observation->history_ready);
    memcpy(captured_obs, observation->obs, sizeof(captured_obs));
    memcpy(captured_history, observation->history, sizeof(captured_history));
    if (!network_success)
    {
        return 0;
    }
    memcpy(action, network_action, sizeof(network_action));
    policy->run_us = 77;
    return 1;
}
static void Vofa_Trace_Record(const imu_state_t *imu,
                              const rl_observation_state_t *observation,
                              const float action[RL_ACTION_SIZE], uint64_t time_us,
                              uint8_t engaged, uint8_t ready, float inference_us)
{
    (void)imu; (void)observation; (void)action; (void)time_us;
    (void)engaged; (void)ready; (void)inference_us;
    trace_calls++;
}
"""


HARNESS = r"""
static void near(float actual, float expected)
{
    if (fabsf(actual - expected) > 1.0e-5f)
    {
        fprintf(stderr, "actual=%g expected=%g\n", actual, expected);
        assert(0);
    }
}
static void rc(int16_t yaw_raw, int16_t velocity_raw, int16_t height_raw,
               int16_t left_y_raw, uint8_t online)
{
    dr16_t remote = {0};
    remote.ch0 = yaw_raw;
    remote.ch1 = velocity_raw;
    remote.wheel = height_raw;
    remote.ch3 = left_y_raw;
    remote.s1 = DR16_SW_UP;
    remote.s2 = DR16_SW_MID;
    remote.online = online;
    Rc_Command_Update(&rc_command, &remote);
}
static void setup(void)
{
    unsigned i;
    memset(&rl_control, 0, sizeof(rl_control));
    memset(&imu_state, 0, sizeof(imu_state));
    memset(&motor_state, 0, sizeof(motor_state));
    memset(&leg_l, 0, sizeof(leg_l));
    memset(&leg_r, 0, sizeof(leg_r));
    memset(&action_state, 0, sizeof(action_state));
    memset(&input_command, 0, sizeof(input_command));
    RL_Observation_Param_Init(&rl_control.param);
    RL_Observation_Init(&rl_control.observation);
    imu_state.online = 1;
    imu_state.quat[0] = 1;
    imu_state.gyro_rad_s[0] = 0.4f;
    imu_state.gyro_rad_s[1] = -0.8f;
    imu_state.gyro_rad_s[2] = 1.2f;
    leg_l.output.valid = leg_r.output.valid = 1;
    leg_l.output.thigh_angle = machine->rl.zero[0] + 0.2f;
    leg_l.output.virtual_shank_angle = machine->rl.zero[1] - 0.3f;
    leg_r.output.thigh_angle = machine->rl.zero[2] + 0.4f;
    leg_r.output.virtual_shank_angle = machine->rl.zero[3] - 0.5f;
    leg_l.input.d_hip_f = 1;
    leg_l.output.d_virtual_shank_angle = 2;
    leg_r.input.d_hip_f = 3;
    leg_r.output.d_virtual_shank_angle = 4;
    for (i = 0; i < DM_MOTOR_NUM; i++) { motor_state.dm.online[i] = 1; }
    for (i = 0; i < DJI_MOTOR_NUM; i++)
    {
        motor_state.dji.online[i] = 1;
        motor_state.dji.vel_rad_s[i] = (float)(5 + i);
    }
    fixture_engaged = 0;
    network_success = 1;
    network_calls = trace_calls = network_init_calls = 0;
    fixture_tick = 1000;
    warmup_cnt = 0;
    rc(0, 0, 0, 0, 1);
    assert(machine->rl.configured);
    assert(RL_OBS_SIZE == 25 && RL_OBS_HISTORY_SIZE == 125 && RL_ACTION_SIZE == 6);
    assert(RL_OBS_CMD_VX == 6 && RL_OBS_CMD_YAW_RATE == 7 && RL_OBS_CMD_HEIGHT == 8);
}
static void commands(float vx, float yaw, float height)
{
    float command[3];
    RL_Command_From_Rc(command);
    near(command[0], vx); near(command[1], yaw); near(command[2], height);
    near(input_command.vx_cmd, vx);
    near(input_command.yaw_cmd, yaw);
    near(input_command.height_cmd, height);
}
static void build(void)
{
    float command[3];
    imu_state_t used_imu;
    uint64_t time_us;
    RL_Command_From_Rc(command);
    assert(RL_Control_Update_Observation(command, &used_imu, &time_us));
    assert(time_us == (uint64_t)fixture_tick * 1000);
}
static void all_actions_zero(void)
{
    unsigned i;
    for (i = 0; i < RL_ACTION_SIZE; i++) { near(action_state.a[i], 0); }
}
static void warmup(void)
{
    unsigned i;
    fixture_engaged = 1;
    for (i = 0; i < RL_WARMUP_STEPS; i++)
    {
        fixture_tick += 10;
        ctrl_task_body();
        assert(network_calls == 0);
        all_actions_zero();
    }
    assert(warmup_cnt == RL_WARMUP_STEPS);
}
int main(int argc, char **argv)
{
    unsigned i, frame;
    assert(argc == 2);
    setup();
    if (strcmp(argv[1], "gas_only_no_inference") != 0)
    {
        assert(RL_CMD_VX_MAX == 1.0f && RL_CMD_YAW_MAX == 3.0f);
    }
    if (strcmp(argv[1], "command_range") == 0)
    {
        float command[3];
        fixture_engaged = 1;
        rc(-660, -660, -660, -660, 1); commands(-1.0f, -3.0f, RL_CMD_HEIGHT_INIT - 0.003f);
        rc(0, 0, 0, 0, 1); commands(0, 0, RL_CMD_HEIGHT_INIT - 0.003f);
        rc(-660, -660, -660, -660, 1);
        for (i = 0; i < 100; i++) { RL_Command_From_Rc(command); }
        commands(-1.0f, -3.0f, RL_CMD_HEIGHT_MIN);
        rc(660, 660, 660, 660, 1); commands(1.0f, 3.0f, RL_CMD_HEIGHT_MIN + 0.003f);
        for (i = 0; i < 100; i++) { RL_Command_From_Rc(command); }
        rc(1000, 1000, 1000, 1000, 1); commands(1.0f, 3.0f, RL_CMD_HEIGHT_MAX);
        rc(-1000, -1000, -1000, -1000, 1);
        for (i = 0; i < 100; i++) { RL_Command_From_Rc(command); }
        rc(-1000, -1000, -1000, -1000, 1); commands(-1.0f, -3.0f, RL_CMD_HEIGHT_MIN);
    }
    else if (strcmp(argv[1], "deadband_and_offline") == 0)
    {
        fixture_engaged = 1;
        rc(20, 10, 20, 20, 1); commands(0, 0, RL_CMD_HEIGHT_INIT);
        near(rc_command.vel, 0); near(rc_command.yaw, 0); near(rc_command.len, 0);
        rc(21, 11, 21, 21, 1);
        assert(rc_command.vel > 0 && rc_command.yaw < 0 && rc_command.len > 0);
        commands(11.0f / 660.0f, 21.0f / 660.0f * 3.0f,
            RL_CMD_HEIGHT_INIT + 21.0f / 660.0f * 0.003f);
        rc(660, 660, 660, 660, 0); commands(0, 0, RL_CMD_HEIGHT_INIT);
        assert(!rc_command.online && !rc_command.s1 && !rc_command.s2);
    }
    else if (strcmp(argv[1], "observation_and_history") == 0)
    {
        float baseline[RL_OBS_SIZE], snapshots[6][RL_OBS_SIZE];
        build();
        memcpy(baseline, rl_control.observation.obs, sizeof(baseline));
        near(baseline[6], 0); near(baseline[7], 0); near(baseline[8], RL_CMD_HEIGHT_INIT * 5.0f);
        for (frame = 0; frame < RL_OBS_HISTORY_FRAMES; frame++)
        {
            assert(!memcmp(&rl_control.observation.history[frame * RL_OBS_SIZE],
                           baseline, sizeof(baseline)));
        }
        for (frame = 0; frame < 6; frame++)
        {
            imu_state.gyro_rad_s[0] = 0.4f + (float)frame * 0.1f;
            leg_r.output.thigh_angle = machine->rl.zero[2] + 0.4f + (float)frame * 0.02f;
            build();
            memcpy(snapshots[frame], rl_control.observation.obs, sizeof(baseline));
            near(snapshots[frame][0], 0.1f + (float)frame * 0.025f);
            near(snapshots[frame][9], -0.34f - (float)frame * 0.02f);
            for (i = 0; i < RL_OBS_SIZE; i++)
            {
                if (i != 0 && i != 9) { near(snapshots[frame][i], baseline[i]); }
            }
        }
        for (frame = 0; frame < RL_OBS_HISTORY_FRAMES; frame++)
        {
            assert(!memcmp(&rl_control.observation.history[frame * RL_OBS_SIZE],
                           snapshots[frame + 1], sizeof(baseline)));
        }
        assert(network_calls == 0);
    }
    else if (strcmp(argv[1], "inference_publication") == 0)
    {
        const float expected_reference[6] = {0.5f, -0.625f, -0.75f, -0.125f, 0.25f, 0.375f};
        rc(660, 660, 660, 0, 1);
        ctrl_task_init();
        assert(network_init_calls == 1);
        warmup();
        fixture_tick += 10; ctrl_task_body();
        assert(network_calls == 1 && action_state.updated && action_state.rl_ready);
        assert(action_state.last_ok_tick == fixture_tick);
        near(captured_obs[6], 2.0f); near(captured_obs[7], 0.75f);
        near(captured_obs[8], RL_CMD_HEIGHT_MAX * 5.0f);
        near(captured_obs[9], -0.34f); near(captured_obs[10], 0.40f);
        near(captured_obs[11], 0.14f); near(captured_obs[12], -0.20f);
        for (frame = 0; frame < RL_OBS_HISTORY_FRAMES; frame++)
        {
            assert(!memcmp(&captured_history[frame * RL_OBS_SIZE], captured_obs,
                           sizeof(captured_obs)));
        }
        for (i = 0; i < RL_ACTION_SIZE; i++)
        {
            near(captured_obs[19 + i], 0);
            near(action_state.a[i], (float)machine->rl.sign[i] * expected_reference[i]);
            near(rl_control.observation.last_action[i], network_action[i]);
        }
        ctrl_task_body();
        assert(network_calls == 2);
        for (i = 0; i < RL_ACTION_SIZE; i++)
        {
            near(captured_obs[19 + i], network_action[i]);
            near(captured_history[100 + 19 + i], network_action[i]);
        }
    }
    else if (strcmp(argv[1], "command_changes") == 0)
    {
        warmup();
        rc(0, 660, 660, 0, 1); ctrl_task_body();
        near(captured_obs[6], 2.0f); near(captured_obs[7], 0);
        near(captured_obs[8], (RL_CMD_HEIGHT_INIT + 0.003f) * 5.0f);
        for (i = 0; i < 4; i++) { ctrl_task_body(); }
        near(captured_obs[8], (RL_CMD_HEIGHT_INIT + 0.015f) * 5.0f);
        rc(0, -660, -660, 0, 1); ctrl_task_body();
        near(captured_obs[6], -2.0f); near(captured_obs[7], 0);
        near(captured_obs[8], (RL_CMD_HEIGHT_INIT + 0.012f) * 5.0f);
        near(captured_history[100 + 6], -2.0f);
        near(captured_history[100 + 8], (RL_CMD_HEIGHT_INIT + 0.012f) * 5.0f);
        near(captured_history[75 + 6], 2.0f);
        near(captured_history[75 + 8], (RL_CMD_HEIGHT_INIT + 0.015f) * 5.0f);
        rc(0, 0, 0, 0, 1); ctrl_task_body();
        near(captured_obs[6], 0); near(captured_obs[7], 0);
        near(captured_obs[8], (RL_CMD_HEIGHT_INIT + 0.012f) * 5.0f);
        for (i = 0; i < 20; i++) { ctrl_task_body(); }
        commands(0, 0, RL_CMD_HEIGHT_INIT + 0.012f);
        fixture_engaged = 0; ctrl_task_body();
        near(rl_control.observation.obs[8], RL_CMD_HEIGHT_INIT * 5.0f);
        commands(0, 0, RL_CMD_HEIGHT_INIT);
        fixture_engaged = 1; rc(0, 0, 660, 0, 1);
        commands(0, 0, RL_CMD_HEIGHT_INIT + 0.003f);
    }
    else if (strcmp(argv[1], "preview_no_inference") == 0)
    {
        rc(-660, 660, -660, 0, 1); ctrl_task_body();
        assert(network_calls == 0 && trace_calls == 1 && warmup_cnt == 0);
        assert(!action_state.rl_ready && !rl_control.observation.history_ready);
        all_actions_zero();
        near(rl_control.observation.obs[6], 2.0f);
        near(rl_control.observation.obs[7], -0.75f);
        near(rl_control.observation.obs[8], RL_CMD_HEIGHT_INIT * 5.0f);
    }
    else if (strcmp(argv[1], "invalid_source") == 0)
    {
        warmup(); motor_state.dm.online[3] = 0; ctrl_task_body();
        assert(network_calls == 0 && !action_state.rl_ready && !warmup_cnt);
        assert(!rl_control.observation.valid && !rl_control.observation.history_ready);
        all_actions_zero();
    }
    else if (strcmp(argv[1], "gas_only_no_inference") == 0)
    {
        warmup();
        gas_spring_only_enabled = 1;
        for (i = 0; i < RL_WARMUP_STEPS + 2; i++)
        {
            ctrl_task_body();
        }
        assert(network_calls == 0 && warmup_cnt == 0);
        assert(!action_state.rl_ready && !rl_control.observation.history_ready);
        all_actions_zero();
        gas_spring_only_enabled = 0;
        warmup(); ctrl_task_body();
        assert(network_calls == 1 && action_state.rl_ready);
    }
    else if (strcmp(argv[1], "network_failure") == 0)
    {
        warmup(); network_success = 0; ctrl_task_body();
        assert(network_calls == 1 && !action_state.rl_ready); all_actions_zero();
    }
    else if (strcmp(argv[1], "joint_roles") == 0)
    {
        float pos[4], vel[6];
        const float expected_pos[4] = {-0.4f, 0.5f, 0.2f, -0.3f};
        const float expected_vel[6] = {-3, -4, 6, 1, 2, -5};
        assert(RL_Joint_Map(pos, vel));
        for (i = 0; i < 4; i++) { near(pos[i], expected_pos[i]); }
        for (i = 0; i < 6; i++) { near(vel[i], expected_vel[i]); }
    }
    else if (strcmp(argv[1], "default_pose") == 0)
    {
        leg_l.output.thigh_angle = machine->rl.zero[0] + 0.06f;
        leg_r.output.thigh_angle = machine->rl.zero[2] + 0.06f;
        leg_l.output.virtual_shank_angle = machine->rl.zero[1] - 0.10f;
        leg_r.output.virtual_shank_angle = machine->rl.zero[3] - 0.10f;
        build();
        for (i = 9; i < 13; i++) { near(rl_control.observation.obs[i], 0); }
    }
    else { assert(!"unknown scenario"); }
    puts(argv[1]); return 0;
}
"""


class RlCommandInputTest(unittest.TestCase):
    def test_real_rc_observation_and_publication_with_network_double(self):
        self.run_scenarios(("command_range", "deadband_and_offline", "observation_and_history",
                            "inference_publication", "command_changes", "preview_no_inference", "invalid_source",
                            "network_failure", "joint_roles", "default_pose"))

    def test_gas_only_suppresses_inference_and_restores_warmup(self):
        self.run_scenarios(("gas_only_no_inference",))

    def run_scenarios(self, scenarios):
        compiler = os.environ.get("CC") or shutil.which("gcc")
        if compiler is None:
            self.skipTest("no native C compiler; set CC to host gcc")
        header = (ROOT / "imcalib/task/inc/robot_control.h").read_text(encoding="utf-8")
        source = (ROOT / "imcalib/task/task_policy.c").read_text(encoding="utf-8")
        source = re.sub(r"^#include[^\n]*", "", source, flags=re.M)
        parts = [HEADERS]
        for name in ("dm_motor_state_t", "dji_motor_state_t", "motor_state_t",
                     "action_state_t", "input_command_t"):
            parts.append(production_type(header, name))
        parts.extend([STUBS, production_function(
            (ROOT / "imcalib/user-lib/dr16.c").read_text(encoding="utf-8"), "DR16_Deadline"),
            production_function((ROOT / "imcalib/user-lib/pid.c").read_text(encoding="utf-8"),
                                "Angle_Wrap_180"), source, HARNESS])
        environment = os.environ.copy()
        if os.name == "nt":
            resolved = shutil.which(compiler) or compiler
            environment["PATH"] = (str(Path(resolved).resolve().parent) + os.pathsep
                                   + environment.get("PATH", ""))
        with tempfile.TemporaryDirectory(prefix="rl-command-input-") as temporary:
            folder = Path(temporary)
            host = folder / "rl_command_input.c"
            executable = folder / ("rl_command_input.exe" if os.name == "nt" else "rl_command_input")
            host.write_text("\n\n".join(parts), encoding="utf-8")
            (folder / "main.h").write_text("#include <stdint.h>\n", encoding="utf-8")
            (folder / "ai_platform.h").write_text(
                "typedef void *ai_handle;\ntypedef struct { void *data; } ai_buffer;\n",
                encoding="utf-8")
            command = [compiler, "-std=c99", "-Wall", "-Wextra", "-Werror",
                       "-I" + str(folder), "-I" + str(ROOT / "imcalib/user-lib"),
                       "-I" + str(ROOT / "imcalib/Algorithm"), str(host),
                       str(ROOT / "imcalib/user-lib/rc_command.c"),
                       str(ROOT / "imcalib/user-lib/machine_config.c"),
                       str(ROOT / "imcalib/Algorithm/rl_observation.c"),
                       "-lm", "-o", str(executable)]
            result = subprocess.run(command, capture_output=True, text=True, env=environment)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            for scenario in scenarios:
                with self.subTest(scenario=scenario):
                    result = subprocess.run([str(executable), scenario], capture_output=True,
                                            text=True, env=environment)
                    self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


if __name__ == "__main__":
    unittest.main()
