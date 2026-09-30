"""Compile the real output gate with host drivers; no controller is reimplemented."""

from pathlib import Path
import os
import re
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]


def function(source, name):
    match = re.search(r"^(?:static\s+)?(?:inline\s+)?(?:void|uint8_t)\s+" + name
                      + r"\s*\([^)]*\)\s*\{", source, re.M)
    if match is None:
        raise AssertionError("missing production function: " + name)
    depth = 1
    end = match.end()
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[match.start():end]


def typedef(source, name):
    match = re.search(r"typedef\s+(?:struct|enum)\s*\{[^}]*\}\s*"
                      + name + r"\s*;", source)
    if match is None:
        raise AssertionError("missing production type: " + name)
    return match.group()


STUBS = r"""
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#define DM_MOTOR_NUM 4
#define DJI_MOTOR_NUM 2
#define DM_MOTOR_LEG_F_LFT 0
#define DM_MOTOR_LEG_B_LFT 1
#define DM_MOTOR_LEG_F_RGT 2
#define DM_MOTOR_LEG_B_RGT 3
#define DJI_MOTOR_WHEEL_LFT 0
#define DJI_MOTOR_WHEEL_RGT 1
typedef enum { HAL_OK, HAL_ERROR } HAL_StatusTypeDef;
static float sent_dm[DM_MOTOR_NUM], sent_wheel[DJI_MOTOR_NUM];
static HAL_StatusTypeDef dm_result, wheel_result;
static unsigned dm_calls, wheel_calls;
static uint8_t joint_usb_lock;
static uint8_t JointUsb_ModeLock(void) { return joint_usb_lock; }
static HAL_StatusTypeDef Dm_Send_Torque(const float *value)
{
    memcpy(sent_dm, value, sizeof(sent_dm));
    dm_calls++;
    return dm_result;
}
static HAL_StatusTypeDef Dm_Send_Zero(void)
{
    memset(sent_dm, 0, sizeof(sent_dm));
    dm_calls++;
    return dm_result;
}
static HAL_StatusTypeDef Dji_Send_Wheel_Torque(float left, float right)
{
    sent_wheel[0] = left;
    sent_wheel[1] = right;
    wheel_calls++;
    return wheel_result;
}
static HAL_StatusTypeDef Dji_All_Stop(void)
{
    return Dji_Send_Wheel_Torque(0.0f, 0.0f);
}
"""


INTEGRATION_STUBS = r"""
static struct {
    struct { float pos_target[6]; } torque_state;
} rl_control;
static uint8_t stream_requested, sample_dm_ok;
static unsigned sample_calls;
static uint64_t sample_queue_ns;
static torque_output_t sampled_torque;
static uint64_t Mono_Ns_Get(void) { return 12345; }
static uint8_t JointUsb_StreamRequested(void) { return stream_requested; }
static void JointUsb_ActuationTick(const torque_output_t *value,
                                   uint64_t queue_ns, uint8_t dm_ok)
{
    sampled_torque = *value;
    sample_queue_ns = queue_ns;
    sample_dm_ok = dm_ok;
    sample_calls++;
}
"""


HARNESS = r"""
static torque_output_t command = {{10, 20, 30, 40}, {50, 60}, 1};
static void reset(void)
{
    leg_response_test = (leg_response_test_t){0};
    rl_output_diag = (rl_output_diag_t){0};
    memset(&rl_control, 0, sizeof(rl_control));
    vofa_leg_response_side = 0;
    robot_state = (robot_state_t){0};
    leg_map_l = (leg_map_t){0, 1, 1};
    leg_map_r = (leg_map_t){2, 3, 1};
    ctrl_strategy = CTRL_STRATEGY_RL;
    torque_output_enabled = 1;
    joint_usb_lock = 0;
    dm_result = wheel_result = HAL_OK;
    dm_calls = wheel_calls = 0;
    stream_requested = sample_calls = 0;
    command.valid = 1;
}
static void arm(uint8_t side)
{
    vofa_leg_response_side = side;
    leg_response_test.requested = 1;
    output_leg_test_update();
    assert(leg_response_test.active == 1);
    assert(leg_response_test.side == side);
    assert(leg_response_test.inhibited == 0);
    assert(output_leg_test_side() == side);
    robot_state.motor_enabled = 1;
}
static void tick(void)
{
    torque_output_t before = command;
    output_leg_test_update();
    output_dispatch(&command);
    assert(memcmp(&before, &command, sizeof(command)) == 0);
}
static void expect(float a, float b, float c, float d, float left, float right)
{
    float wanted[4] = {a, b, c, d};
    unsigned i;
    assert(dm_calls > 0 && wheel_calls > 0);
    for (i = 0; i < DM_MOTOR_NUM; i++)
    {
        assert(sent_dm[i] == wanted[i]);
        assert(rl_output_dm_cmd_nm[i] == wanted[i]);
    }
    assert(sent_wheel[0] == left && sent_wheel[1] == right);
    assert(rl_output_wheel_cmd_nm[0] == left);
    assert(rl_output_wheel_cmd_nm[1] == right);
    assert(output_debug_dm_sent == (dm_result == HAL_OK));
    assert(output_debug_dji_sent == (wheel_result == HAL_OK));
}
static void zero(void) { expect(0, 0, 0, 0, 0, 0); }
static void expect_sample(void)
{
    unsigned i;
    assert(sample_calls == 1 && sample_queue_ns == 12345);
    assert(sampled_torque.valid == 1);
    assert(sample_dm_ok == output_debug_dm_sent);
    for (i = 0; i < DM_MOTOR_NUM; i++)
    {
        assert(sampled_torque.dm[i] == sent_dm[i]);
    }
    for (i = 0; i < DJI_MOTOR_NUM; i++)
    {
        assert(sampled_torque.dji[i] == sent_wheel[i]);
    }
}
static void diagnostic_targets(void)
{
    /* Wheel entries must not appear in the four joint targets. */
    float targets[6] = {11, 12, 999, 13, 14, 888};
    memcpy(rl_control.torque_state.pos_target, targets, sizeof(targets));
}
static void expect_targets(void)
{
    assert(rl_output_diag.valid == 1);
    assert(rl_output_diag.updated_ns == 12345);
    assert(rl_output_diag.joint_target[0] == 11);
    assert(rl_output_diag.joint_target[1] == 12);
    assert(rl_output_diag.joint_target[2] == 13);
    assert(rl_output_diag.joint_target[3] == 14);
}
int main(int argc, char **argv)
{
    unsigned i;
    assert(argc == 2);
    assert(leg_response_test.requested == 1 && !leg_response_test.active
           && !leg_response_test.side && !leg_response_test.inhibited);
    assert(vofa_leg_response_side == 0);
    if (strcmp(argv[1], "startup_left") == 0)
    {
        leg_map_l = (leg_map_t){0, 1, 1};
        leg_map_r = (leg_map_t){2, 3, 1};
        torque_output_enabled = 1;
        ctrl_strategy = CTRL_STRATEGY_RL;
        assert(robot_state.motor_enabled == 0);
        output_leg_test_update();
        assert(leg_response_test.active == 1 && leg_response_test.side == 0
               && leg_response_test.inhibited == 0);
        robot_state.motor_enabled = 1;
        tick();
        expect(10, 20, 0, 0, 0, 0);
        puts(argv[1]);
        return 0;
    }
    reset();
    if (strcmp(argv[1], "left") == 0)
    {
        arm(0); tick(); expect(10, 20, 0, 0, 0, 0);
    }
    else if (strcmp(argv[1], "right") == 0)
    {
        arm(1); tick(); expect(0, 0, 30, 40, 0, 0);
    }
    else if (strcmp(argv[1], "mapping") == 0)
    {
        leg_map_l = (leg_map_t){3, 1, 1};
        arm(0); tick(); expect(0, 20, 0, 40, 0, 0);
    }
    else if (strcmp(argv[1], "off") == 0)
    {
        tick(); expect(10, 20, 30, 40, 50, 60);
        robot_state.motor_enabled = 1;
        vofa_leg_response_side = 1;
        tick(); expect(10, 20, 30, 40, 50, 60);
        assert(!leg_response_test.inhibited);
    }
    else if (strcmp(argv[1], "change_side") == 0)
    {
        arm(0);
        vofa_leg_response_side = 1;
        tick(); zero();
        assert(leg_response_test.inhibited && leg_response_test.side == 0);
        assert(output_leg_test_side() == 0);
        vofa_leg_response_side = 0;
        tick(); zero();
        robot_state.motor_enabled = 0;
        output_leg_test_update();
        robot_state.motor_enabled = 1;
        tick(); expect(10, 20, 0, 0, 0, 0);
    }
    else if (strcmp(argv[1], "stop_while_enabled") == 0)
    {
        arm(0);
        leg_response_test.requested = 0;
        tick(); zero();
        assert(leg_response_test.active && leg_response_test.inhibited);
        leg_response_test.requested = 1;
        tick(); zero();
        robot_state.motor_enabled = 0;
        leg_response_test.requested = 0;
        tick(); expect(10, 20, 30, 40, 50, 60);
        assert(!leg_response_test.active && !leg_response_test.inhibited);
    }
    else if (strcmp(argv[1], "start_while_enabled") == 0)
    {
        output_leg_test_update();
        robot_state.motor_enabled = 1;
        leg_response_test.requested = 1;
        tick(); zero();
        assert(!leg_response_test.active && leg_response_test.inhibited);
        leg_response_test.requested = 0;
        tick(); zero();
    }
    else if (strcmp(argv[1], "invalid_config") == 0)
    {
        leg_response_test.requested = 2;
        tick(); zero();
        assert(leg_response_test.inhibited);
        reset();
        leg_response_test.requested = 1;
        vofa_leg_response_side = 2;
        tick(); zero();
        assert(leg_response_test.inhibited);
        reset();
        arm(0);
        leg_response_test.requested = 255;
        tick(); zero();
        assert(leg_response_test.inhibited);
        reset();
        arm(0);
        vofa_leg_response_side = 255;
        tick(); zero();
        assert(leg_response_test.inhibited);
    }
    else if (strcmp(argv[1], "invalid_mapping") == 0)
    {
        leg_map_t invalid[] = {{0, 1, 0}, {-1, 1, 1}, {0, -1, 1},
                               {4, 1, 1}, {0, 4, 1}, {1, 1, 1}};
        for (i = 0; i < sizeof(invalid) / sizeof(invalid[0]); i++)
        {
            reset(); arm(0); leg_map_l = invalid[i]; tick(); zero();
            reset(); arm(1); leg_map_r = invalid[i]; tick(); zero();
        }
    }
    else if (strcmp(argv[1], "invalid_command") == 0)
    {
        arm(0); tick(); expect(10, 20, 0, 0, 0, 0);
        command.valid = 0;
        tick(); zero();
        command.valid = 1;
        torque_output_enabled = 0;
        tick(); zero();
    }
    else if (strcmp(argv[1], "driver_failure") == 0)
    {
        arm(1);
        dm_result = HAL_ERROR;
        tick(); expect(0, 0, 30, 40, 0, 0);
        dm_result = HAL_OK;
        wheel_result = HAL_ERROR;
        tick(); expect(0, 0, 30, 40, 0, 0);
        command.valid = 0;
        tick(); zero();
        dm_result = HAL_ERROR;
        wheel_result = HAL_OK;
        tick(); zero();
    }
    else if (strcmp(argv[1], "jid1") == 0)
    {
        arm(0);
        joint_usb_lock = 1;
        ctrl_strategy = CTRL_STRATEGY_JOINT_USB;
        tick(); expect(10, 20, 30, 40, 50, 60);
        leg_response_test.inhibited = 1;
        tick(); expect(10, 20, 30, 40, 50, 60);
        torque_output_enabled = 0;
        tick(); zero();
    }
    else if (strcmp(argv[1], "other_strategy") == 0)
    {
        arm(0);
        ctrl_strategy = CTRL_STRATEGY_LQR;
        tick(); zero();
        ctrl_strategy = CTRL_STRATEGY_DISABLE;
        tick(); zero();
        ctrl_strategy = CTRL_STRATEGY_JOINT_USB;
        tick(); zero();
        reset();
        ctrl_strategy = CTRL_STRATEGY_LQR;
        tick(); expect(10, 20, 30, 40, 50, 60);
    }
    else if (strcmp(argv[1], "stream_filtered") == 0)
    {
        arm(1);
        stream_requested = 1;
        integration_dispatch(command);
        expect(0, 0, 30, 40, 0, 0);
        expect_sample();
    }
    else if (strcmp(argv[1], "stream_zero") == 0)
    {
        arm(0);
        stream_requested = 1;
        torque_output_enabled = 0;
        integration_dispatch(command);
        zero();
        expect_sample();
    }
    else if (strcmp(argv[1], "diagnostic_target_mapping") == 0)
    {
        arm(1);
        diagnostic_targets();
        integration_dispatch(command);
        expect(0, 0, 30, 40, 0, 0);
        expect_targets();
        assert(!stream_requested && sample_calls == 0);
    }
    else if (strcmp(argv[1], "diagnostic_invalid_next_tick") == 0)
    {
        diagnostic_targets();
        integration_dispatch(command);
        expect_targets();
        command.valid = 0;
        rl_control.torque_state.pos_target[0] = 999;
        integration_dispatch(command);
        zero();
        assert(!rl_output_diag.valid);
        assert(rl_output_diag.joint_target[0] == 11);
    }
    else if (strcmp(argv[1], "diagnostic_non_rl") == 0)
    {
        ctrl_strategy_t strategies[] = {CTRL_STRATEGY_LQR, CTRL_STRATEGY_DISABLE,
                                        CTRL_STRATEGY_JOINT_USB};
        for (i = 0; i < sizeof(strategies) / sizeof(strategies[0]); i++)
        {
            reset();
            diagnostic_targets();
            integration_dispatch(command);
            expect_targets();
            ctrl_strategy = strategies[i];
            joint_usb_lock = (uint8_t)(ctrl_strategy == CTRL_STRATEGY_JOINT_USB);
            integration_dispatch(command);
            assert(!rl_output_diag.valid);
        }
    }
    else if (strcmp(argv[1], "diagnostic_zero_output") == 0)
    {
        arm(0);
        diagnostic_targets();
        torque_output_enabled = 0;
        integration_dispatch(command);
        zero();
        expect_targets();
    }
    else { assert(!"unknown scenario"); }
    puts(argv[1]);
    return 0;
}
"""


class LegResponseOutputTest(unittest.TestCase):
    def test_production_output_gate(self):
        source = (ROOT / "imcalib/task/task_actuation.c").read_text(encoding="utf-8")
        header = (ROOT / "imcalib/task/inc/robot_control.h").read_text(encoding="utf-8")
        torque = (ROOT / "imcalib/Algorithm/torque_output.h").read_text(encoding="utf-8")
        parts = [STUBS]
        for name in ("leg_response_test_t", "rl_output_diag_t", "robot_state_t",
                     "leg_map_t", "ctrl_strategy_t"):
            parts.append(typedef(header, name))
        parts.extend([typedef(torque, "torque_output_t"),
                      "robot_state_t robot_state; leg_map_t leg_map_l, leg_map_r;",
                      "volatile ctrl_strategy_t ctrl_strategy;",
                      "uint8_t torque_output_enabled;",
                      "volatile uint8_t output_debug_dm_sent, output_debug_dji_sent;"])
        parts.extend([function(torque, "Torque_Output_Clear"), INTEGRATION_STUBS])
        for name in ("leg_response_test", "vofa_leg_response_side",
                     "rl_output_dm_cmd_nm", "rl_output_wheel_cmd_nm", "rl_output_diag"):
            match = re.search(r"^volatile\s+[^;\n]*\b" + name + r"\b[^;\n]*;", source, re.M)
            self.assertIsNotNone(match, "missing production global: " + name)
            parts.append(match.group())
        for name in ("output_leg_test_update", "output_leg_test_side", "output_dispatch",
                     "output_rl_diag_update"):
            parts.append(function(source, name))
        body = function(source, "output_task_body")
        self.assertIn("output_leg_test_update();", body)
        # Execute the production dispatch/telemetry block without stubbing the controllers.
        start = body.rfind("{", 0, body.index("output_dispatch(&torque)"))
        block = body[start:body.rfind("}")]
        declarations = []
        for name in ("sent_torque", "i"):
            match = re.search(r"(?:torque_output_t|uint8_t)\s+" + name + r"\s*;", body)
            self.assertIsNotNone(match, "missing production local: " + name)
            declarations.append(match.group())
        parts.append("static void integration_dispatch(torque_output_t torque)\n{\n"
                     + "\n".join(declarations) + "\n" + block + "\n}")
        parts.append(HARNESS)
        compiler = os.environ.get("CC") or shutil.which("gcc")
        if compiler is None:
            self.skipTest("no native C compiler; set CC to host gcc")
        environment = os.environ.copy()
        if os.name == "nt":
            compiler_path = shutil.which(compiler) or compiler
            environment["PATH"] = (str(Path(compiler_path).resolve().parent)
                                   + os.pathsep + environment.get("PATH", ""))
        with tempfile.TemporaryDirectory(prefix="leg-response-output-") as temporary:
            folder = Path(temporary)
            host = folder / "output_gate.c"
            executable = folder / ("output_gate.exe" if os.name == "nt" else "output_gate")
            host.write_text("\n\n".join(parts), encoding="utf-8")
            result = subprocess.run([compiler, "-std=c99", "-Wall", "-Wextra", "-Werror",
                                     str(host), "-o", str(executable)], capture_output=True,
                                    text=True, env=environment)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            for scenario in ("startup_left", "left", "right", "mapping", "off", "change_side",
                             "stop_while_enabled", "start_while_enabled", "invalid_config",
                             "invalid_mapping", "invalid_command", "driver_failure",
                             "jid1", "other_strategy", "stream_filtered", "stream_zero",
                             "diagnostic_target_mapping", "diagnostic_invalid_next_tick",
                             "diagnostic_non_rl", "diagnostic_zero_output"):
                with self.subTest(scenario=scenario):
                    result = subprocess.run([str(executable), scenario],
                                            capture_output=True, text=True, env=environment)
                    self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


if __name__ == "__main__":
    unittest.main()
