"""Execute the production arbiter, spring solver and dispatch on a host."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

from test_gas_spring_integration import (
    ROOT, HEADERS, STUBS, ARM_MATH_SHIM, production_enum, production_function, read, without_includes,
)


FIXTURE = r"""
#define DR16_SW_UP 1
#define DR16_SW_DOWN 2
#define DR16_SW_MID 3
static volatile uint8_t gas_spring_only_enabled = 1;
static leg_state_t leg_l, leg_r;
static rc_command_t rc_command;
static struct { uint8_t rc_enable, motor_enabled, fallen; } robot_state;
static uint32_t ctrl_fault;
#define FAULT_NONE 0
static struct { struct { float vel_rad_s[2]; } dji; } motor_state;
static imu_state_t imu_state;
static struct { uint8_t rl_ready; float a[6]; } action_state;
static struct {
    struct { unsigned selected_model; } policy;
    rl_torque_param_t torque_param[1];
    rl_torque_state_t torque_state;
} rl_control;
static lqr_state_t lqr_state;
static leg_balance_t leg_balance;
static ctrl_strategy_t ctrl_strategy;
static unsigned rl_calls, lqr_calls, usb_calls;
static uint8_t usb_lock;
static int htim6;
static int HAL_TIM_Base_Start_IT(int *timer) { (void)timer; return HAL_OK; }
static void Error_Handler(void) { assert(0); }
static uint64_t Mono_Ns_Get(void) { return 1; }
static uint8_t JointUsb_ModeLock(void) { return usb_lock; }
static uint8_t JointUsb_PhysicalPermit(void) { return 1; }
static uint8_t JointUsb_EnableAllowed(void) { return 1; }
static int Dm_All_Enable(void) { return HAL_OK; }
static int Dm_All_Disable(void) { return HAL_OK; }
static void Dm_Enable_Watchdog(void) {}
static void Dm_Disable_Watchdog(void) {}
static uint8_t JointUsb_StreamRequested(void) { return 0; }
static void JointUsb_Compute(torque_output_t *t) { (void)t; usb_calls++; }
static void JointUsb_ActuationTick(const torque_output_t *t, uint64_t ns, uint8_t ok)
{ (void)t; (void)ns; (void)ok; }
#define LQR_State_Update(a,b,c,d,e,f) ((void)(a),(void)(b),(void)(c),(void)(d),(void)(e),(void)(f),0)
#define LQR_Enable_Latch(a,b,c) ((void)(a),(void)(b),(void)(c),1)
#define Leg_Balance_Reset(a) ((void)(a))
#define LQR_Target_Update(a,b,c) ((void)(a),(void)(b),(void)(c),0)
#define LQR_Control_Update(a) ((void)(a),++lqr_calls)
#define Leg_Balance_Compute(a,b,c,d,e,f) ((void)(a),(void)(b),(void)(c),(void)(d),(void)(e),(void)(f),0)
#define RL_Torque_Compute(a,b,c,d,e,f,g) ((void)(a),(void)(b),(void)(c),(void)(d),(void)(e),(void)(f),(void)(g),++rl_calls,0)
"""

CHECK = r"""
static void tick(uint8_t expect_output)
{
    unsigned i;
    float delta[2];
    const leg_state_t *leg[2] = {&leg_l, &leg_r};
    output_task_body();
    assert(!rl_calls && !lqr_calls && !usb_calls);
    assert(!output_task_rl_engaged() && !output_task_lqr_engaged());
    assert(fixture_sent_wheel[0] == 0 && fixture_sent_wheel[1] == 0);
    for (i = 0; i < 4; i++)
    {
        float expected = 0;
        if (expect_output)
        {
            assert(Leg_Force_Map_Forward(leg[i/2], -Leg_SpringF(leg[i/2]->output.virtual_leg_length), 0, delta));
            expected = clampf(delta[i%2], -machine->dm_trq_clamp, machine->dm_trq_clamp);
        }
        assert(fabsf(fixture_sent_dm[i] - expected) < 0.0001f);
    }
}
int main(void)
{
    unsigned s1, s2;
    uint8_t available = GAS_SPRING_COMP_ENABLE && MACHINE_DEFAULT == MACHINE_ID_BIG_WHEELLEG;
    robot_state.rc_enable = 1;
    Robot_Enable_Update(); assert(robot_state.motor_enabled);
    ctrl_fault = 1; Robot_Enable_Update(); assert(!robot_state.motor_enabled);
    ctrl_fault = 0; robot_state.fallen = 1;
    Robot_Enable_Update(); assert(!robot_state.motor_enabled);
    robot_state.fallen = 0; usb_lock = 1;
    Robot_Enable_Update(); assert(!robot_state.motor_enabled);
    gas_spring_only_enabled = 0;
    Robot_Enable_Update(); assert(robot_state.motor_enabled);
    gas_spring_only_enabled = 1;
    Robot_Enable_Update(); assert(!robot_state.motor_enabled);
    usb_lock = 0;
    fixture_machine.dm_trq_clamp = 40;
    fixture_machine.rl.configured = fixture_machine.lqr_configured = 1;
    Leg_Init(&leg_l);
    leg_l.config.configured = 1;
    leg_l.config.lu = 0.21f; leg_l.config.lg = 0.25f;
    leg_l.input.hip_f = 2; leg_l.input.hip_b = 0.4f;
    leg_r = leg_l; leg_r.input.hip_f = 2.2f; leg_r.input.hip_b = 0.65f;
    assert(Leg_Solve(&leg_l) && Leg_Solve(&leg_r));
    robot_state.motor_enabled = torque_output_enabled = rc_command.online = 1;
    for (s1 = 1; s1 <= 3; s1++)
    {
        rc_command.s1 = s1;
        robot_state.rc_enable = strategy_rc_enable(&rc_command);
        assert(robot_state.rc_enable == (available && s1 == DR16_SW_UP));
        for (s2 = 1; s2 <= 3; s2++)
        {
            rc_command.s2 = s2;
            tick(available && s1 == DR16_SW_UP && s2 == DR16_SW_MID);
        }
    }
    rc_command.s1 = DR16_SW_UP; rc_command.s2 = DR16_SW_MID;
    robot_state.rc_enable = strategy_rc_enable(&rc_command);
    if (available)
    {
        tick(1); assert(ctrl_strategy == CTRL_STRATEGY_GAS_SPRING);
        fixture_machine.dm_trq_clamp = 0.1f; tick(1);
        leg_r.output.force_valid = 0; tick(0); leg_r.output.force_valid = 1;
        leg_l.output.virtual_leg_length = NAN; tick(0); assert(Leg_Solve(&leg_l));
        torque_output_enabled = 0; tick(0); torque_output_enabled = 1;
        robot_state.motor_enabled = 0; tick(0); robot_state.motor_enabled = 1;
        robot_state.rc_enable = 0; tick(0); robot_state.rc_enable = 1;
    }
    usb_lock = 1; tick(0); assert(ctrl_strategy == CTRL_STRATEGY_DISABLE); usb_lock = 0;
    rc_command.online = 0; tick(0); assert(!strategy_rc_enable(&rc_command));
    assert(!strategy_rc_enable(NULL));
    rc_command.online = 1; robot_state.rc_enable = 1; gas_spring_only_enabled = 0;
    assert(strategy_from_remote(&rc_command) == CTRL_STRATEGY_RL);
    rc_command.s1 = DR16_SW_MID; assert(strategy_from_remote(&rc_command) == CTRL_STRATEGY_LQR);
    usb_lock = 1; assert(strategy_from_remote(&rc_command) == CTRL_STRATEGY_JOINT_USB);
    return 0;
}
"""


class GasSpringBenchTest(unittest.TestCase):
    def test_real_actuation_and_arbiter(self):
        compiler = os.environ.get("CC") or shutil.which("gcc")
        if compiler is None:
            self.skipTest("no native C compiler; set CC to host gcc")
        environment = os.environ.copy()
        environment["PATH"] = str(Path(compiler).parent) + os.pathsep + environment.get("PATH", "")
        headers = HEADERS.replace("int unused; } rc_command_t", "uint8_t online, s1, s2; } rc_command_t")
        headers = headers.replace("int unused; } imu_state_t", "uint8_t online; } imu_state_t")
        actuation = without_includes(read("imcalib/task/task_actuation.c"))
        actuation = actuation.replace("volatile float rl_output_dm_cmd_nm[DM_MOTOR_NUM];", "")
        actuation = actuation.replace("volatile float rl_output_wheel_cmd_nm[DJI_MOTOR_NUM];", "")
        content = [headers,
                   production_enum(read("imcalib/user-lib/dm.h"), "dm_motor_idx_t"),
                   "#define DJI_MOTOR_NUM 2\n#define DJI_MOTOR_WHEEL_LFT 0\n#define DJI_MOTOR_WHEEL_RGT 1",
                   without_includes(read("imcalib/Algorithm/torque_output.h")),
                   without_includes(read("imcalib/Algorithm/rl_torque.h")),
                   without_includes(read("imcalib/Algorithm/lqr_balance.h")),
                   without_includes(read("imcalib/Algorithm/leg_balance.h")),
                   production_enum(read("imcalib/task/inc/robot_control.h"), "ctrl_strategy_t"),
                   STUBS, FIXTURE, without_includes(read("imcalib/Algorithm/gas_spring.c")),
                   actuation, production_function(read("imcalib/task/task_comm.c"), "Robot_Enable_Update"), CHECK]
        with tempfile.TemporaryDirectory(prefix="gas-bench-") as temporary:
            folder = Path(temporary)
            (folder / "arm_math.h").write_text(ARM_MATH_SHIM, encoding="utf-8")
            source = folder / "bench.c"
            source.write_text("\n".join(content), encoding="utf-8")
            for machine_id in (0, 1):
                for enabled in (0, 1):
                    with self.subTest(machine=machine_id, compensation=enabled):
                        exe = folder / (f"bench_{machine_id}_{enabled}" + (".exe" if os.name == "nt" else ""))
                        result = subprocess.run([compiler, "-std=c99", "-Wall", "-Wextra", "-Werror",
                            "-DLEG_TRIG_LIBM=1", f"-DMACHINE_DEFAULT={machine_id}",
                            f"-DGAS_SPRING_COMP_ENABLE={enabled}", "-I", str(folder),
                            "-I", str(ROOT / "imcalib/Algorithm"), "-I", str(ROOT / "imcalib/user-lib"),
                            str(source), str(ROOT / "imcalib/Algorithm/leg_solver.c"), "-lm", "-o", str(exe)],
                            capture_output=True, text=True, env=environment)
                        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                        result = subprocess.run([str(exe)], capture_output=True, text=True, env=environment)
                        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
