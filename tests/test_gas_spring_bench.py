"""Execute the production arbiter, spring solver and dispatch on a host."""
import os
from pathlib import Path
import re
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
static leg_state_t leg_l, leg_r;
static rc_command_t rc_command;
static struct { uint8_t rc_enable, motor_enabled, fallen; } robot_state;
static uint32_t ctrl_fault;
#define FAULT_NONE 0
#define Robot_Control_Vofa_Diag_Update(request) ((void)(request))
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
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
static uint8_t JointUsb_EnableAllowed(void) { return 1; }
static int Dm_All_Enable(void) { return HAL_OK; }
static int Dm_All_Disable(void) { return HAL_OK; }
static void Dm_Enable_Watchdog(void) {}
static void Dm_Disable_Watchdog(void) {}
static uint8_t JointUsb_StreamRequested(void) { return 0; }
static void JointUsb_Compute(torque_output_t *t) { (void)t; usb_calls++; }
static void JointUsb_ActuationTick(const torque_output_t *t, uint64_t ns, uint8_t ok)
{ (void)t; (void)ns; (void)ok; }
#define LQR_Gain_Compatible() (machine->lqr_configured)
#define LQR_Ready() (machine->lqr_configured)
#define LQR_State_Update(a,b,c,d,e,f) ((void)(e),(void)(f),(a)->valid=(b)->online && (c)->output.valid && (d)->output.valid,(a)->valid)
#define LQR_Enable_Latch(a,b,c) ((void)(a),(void)(b),(void)(c),1)
#define LQR_Init(a) ((void)(a))
#define LQR_Velocity_Apply(a,b,c) ((void)(a),(void)(b),(void)(c))
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
    assert(!output_task_lqr_engaged());
    if (ctrl_strategy == CTRL_STRATEGY_COMPENSATION) { assert(!output_task_rl_engaged()); }
    assert(fixture_sent_wheel[0] == 0 && fixture_sent_wheel[1] == 0);
    for (i = 0; i < 4; i++)
    {
        float expected = 0;
        if (expect_output)
        {
            gravity_leg_result_t gravity;
            assert(Gravity_Comp_Compute(machine->gravity, leg[i/2]->output.virtual_leg_length,
                leg[i/2]->output.virtual_leg_angle - imu_state.pitch_world, &gravity));
            assert(Leg_Force_Map_Forward(leg[i/2], -Leg_SpringF(leg[i/2]->output.virtual_leg_length), gravity.torque, delta));
            expected = clampf(delta[i%2], -machine->dm_trq_clamp, machine->dm_trq_clamp);
        }
        assert(fabsf(fixture_sent_dm[i] - expected) < 0.0001f);
    }
}
int main(void)
{
    unsigned s1, s2;
    uint8_t available = GAS_SPRING_COMP_ENABLE && MACHINE_DEFAULT == MACHINE_ID_BIG_WHEELLEG;
    rc_command.online = 1;
    robot_state.rc_enable = 1;
    Robot_Enable_Update(); assert(robot_state.motor_enabled);
    ctrl_fault = 1; Robot_Enable_Update(); assert(!robot_state.motor_enabled);
    ctrl_fault = 0; robot_state.fallen = 1;
    Robot_Enable_Update(); assert(!robot_state.motor_enabled);
    robot_state.fallen = 0; usb_lock = 1;
    Robot_Enable_Update(); assert(robot_state.motor_enabled);
    usb_lock = 0;
    fixture_machine = machine_table[MACHINE_DEFAULT];
    fixture_machine.dm_trq_clamp = 40;
    fixture_machine.rl.configured = fixture_machine.lqr_configured = 1;
    Leg_Init(&leg_l);
    leg_l.config.configured = 1;
    leg_l.config.lu = 0.21f; leg_l.config.lg = 0.25f;
    leg_l.input.hip_f = 2; leg_l.input.hip_b = 0.4f;
    leg_r = leg_l; leg_r.input.hip_f = 2.2f; leg_r.input.hip_b = 0.65f;
    assert(Leg_Solve(&leg_l) && Leg_Solve(&leg_r));
    robot_state.motor_enabled = torque_output_enabled = rc_command.online = imu_state.pitch_world_valid = 1;
    for (s1 = 1; s1 <= 3; s1++)
    {
        rc_command.s1 = s1;
        for (s2 = 1; s2 <= 3; s2++)
        {
            rc_command.s2 = s2;
            robot_state.rc_enable = strategy_rc_enable(&rc_command);
            if (s1 == DR16_SW_MID && s2 == DR16_SW_UP)
            {
                assert(robot_state.rc_enable == available);
            }
            tick(available && s1 == DR16_SW_MID && s2 == DR16_SW_UP);
        }
    }
    rc_command.s1 = DR16_SW_MID; rc_command.s2 = DR16_SW_UP;
    robot_state.rc_enable = strategy_rc_enable(&rc_command);
    if (available)
    {
        tick(1); assert(ctrl_strategy == CTRL_STRATEGY_COMPENSATION);
        assert(compensation_debug.gravity.valid && compensation_debug.bench);
        fixture_machine.lqr_configured = 0; tick(1); fixture_machine.lqr_configured = 1;
        fixture_machine.dm_trq_clamp = 0.1f; tick(1);
        assert(compensation_debug.saturated);
        imu_state.pitch_world_valid = 0; tick(0); imu_state.pitch_world_valid = 1;
        leg_r.output.force_valid = 0; tick(0); leg_r.output.force_valid = 1;
        leg_l.output.virtual_leg_length = NAN; tick(0); assert(Leg_Solve(&leg_l));
        torque_output_enabled = 0; tick(0); torque_output_enabled = 1;
        robot_state.motor_enabled = 0; tick(0); robot_state.motor_enabled = 1;
        robot_state.rc_enable = 0; tick(0); robot_state.rc_enable = 1;
    }
    usb_lock = 1; tick(0); assert(ctrl_strategy == CTRL_STRATEGY_DISABLE); usb_lock = 0;
    rc_command.online = 0; tick(0); assert(!strategy_rc_enable(&rc_command));
    assert(!strategy_rc_enable(NULL));
    rc_command.online = 1; robot_state.rc_enable = 1;
    rc_command.s1 = DR16_SW_UP; rc_command.s2 = DR16_SW_MID;
    assert(strategy_from_remote(&rc_command) == CTRL_STRATEGY_RL);
    rc_command.s1 = DR16_SW_MID; assert(strategy_from_remote(&rc_command) == CTRL_STRATEGY_LQR);
    usb_lock = 1; assert(strategy_from_remote(&rc_command) == CTRL_STRATEGY_DISABLE);
    rc_command.s1 = DR16_SW_UP; rc_command.s2 = DR16_SW_DOWN;
    assert(strategy_from_remote(&rc_command) == CTRL_STRATEGY_JOINT_USB);
    return 0;
}
"""


NORMAL_CHECK = r"""
static void normal_tick(unsigned expected)
{
    unsigned i;
    robot_state.rc_enable=strategy_rc_enable(&rc_command);
    Robot_Enable_Update();
    rl_calls=lqr_calls=usb_calls=0;
    output_task_body();
    if (rl_calls!=(expected==2) || lqr_calls!=(expected==1)) {
        fprintf(stderr,"s1=%u s2=%u expected=%u rl=%u lqr=%u valid=%u fault=%u online=%u\n",
            rc_command.s1,rc_command.s2,expected,rl_calls,lqr_calls,lqr_state.valid,ctrl_fault,imu_state.online);
        assert(0);
    }
    assert(!usb_calls);
    for(i=0;i<4;i++) { assert(fixture_sent_dm[i]==(float)expected); }
    for(i=0;i<2;i++) { assert(fabsf(fixture_sent_wheel[i]-0.1f*expected)<0.0001f); }
}
int main(void)
{
    unsigned s1,s2,expected;
    fixture_machine=machine_table[MACHINE_DEFAULT];
    torque_output_enabled=rc_command.online=imu_state.online=1;
    leg_l.output.valid=leg_r.output.valid=1;
    lqr_state.valid=action_state.rl_ready=1;
    for(s1=1;s1<=3;s1++)
    {
        for(s2=1;s2<=3;s2++)
        {
            rc_command.s1=s1; rc_command.s2=s2;
            expected=0;
            if(s2==DR16_SW_MID && s1==DR16_SW_MID && machine->lqr_configured) { expected=1; }
            if(s2==DR16_SW_MID && s1==DR16_SW_UP && machine->rl.configured) { expected=2; }
            normal_tick(expected);
            if(s1==DR16_SW_MID && s2!=DR16_SW_UP) { assert(ctrl_strategy==CTRL_STRATEGY_LQR); }
            if(s1==DR16_SW_MID && s2==DR16_SW_UP) {
                assert(ctrl_strategy==(GAS_SPRING_COMP_ENABLE && machine->gravity && machine->spring
                    ? CTRL_STRATEGY_COMPENSATION : CTRL_STRATEGY_DISABLE));
            }
            if(s1==DR16_SW_UP && machine->rl.configured) { assert(ctrl_strategy==CTRL_STRATEGY_RL); }
            if(s1==DR16_SW_DOWN) { assert(ctrl_strategy==CTRL_STRATEGY_DISABLE); }
        }
    }
    rc_command.s1=DR16_SW_MID; rc_command.s2=DR16_SW_MID;
    rc_command.online=0; normal_tick(0); rc_command.online=1;
    ctrl_fault=1; normal_tick(0); ctrl_fault=0;
    robot_state.fallen=1; normal_tick(0); robot_state.fallen=0;
    imu_state.online=0; normal_tick(0); imu_state.online=1;
    leg_r.output.valid=0; normal_tick(0); leg_r.output.valid=1;
    fixture_machine.lqr_configured=0; normal_tick(0); fixture_machine.lqr_configured=1;
    if(machine->rl.configured)
    {
        rc_command.s1=DR16_SW_UP;
        action_state.rl_ready=0; normal_tick(0); action_state.rl_ready=1;
        normal_tick(2);
    }
    rc_command.s1=DR16_SW_MID; normal_tick(1);
    return 0;
}
"""

CADENCE_CHECK = r"""
static void step(void)
{
    fixture_ns += 1000000;
    Policy_Tick_Div();
    output_task_body();
}
static void assert_zero(void)
{
    unsigned i;
    for(i=0;i<4;i++) { assert(fixture_sent_dm[i]==0); }
    for(i=0;i<2;i++) { assert(fixture_sent_wheel[i]==0); }
}
int main(void)
{
    unsigned i, compute_before, sends_before, expected, fault_case;
    fixture_machine=machine_table[MACHINE_DEFAULT];
    fixture_machine.rl.configured=1;
    assert(MACHINE_TICK_DT==0.001f && MACHINE_LQR_DT==0.001f);
    assert(fabsf(MACHINE_POLICY_DT-0.01f)<1e-7f);
    assert(MACHINE_POLICY_DIV==10);
    assert(MACHINE_RL_CTRL_DIV==(MACHINE_DEFAULT==0?2u:1u));
    assert(fabsf(MACHINE_RL_CTRL_DT-(MACHINE_DEFAULT==0?0.002f:0.001f))<1e-7f);
    robot_state.rc_enable=robot_state.motor_enabled=rc_command.online=1;
    torque_output_enabled=imu_state.online=action_state.rl_ready=1;
    leg_l.output.valid=leg_r.output.valid=lqr_state.valid=1;
    rc_command.s1=rc_command.s2=DR16_SW_MID;
    Robot_Enable_Update();
    for(i=0;i<100;i++)
    {
        compute_before=lqr_calls; sends_before=fixture_dm_transmits;
        step(); assert(lqr_calls==compute_before+1 && fixture_dm_transmits==sends_before+1);
        if(i) { assert(control_time_debug.period_us==1000 && control_time_debug.output_period_us==1000); }
    }
    assert(policy_releases==10 && control_time_debug.sequence==100);
    rc_command.s1=DR16_SW_UP;
    for(i=0;i<100;i++)
    {
        expected=(i%MACHINE_RL_CTRL_DIV)==0;
        compute_before=rl_calls; sends_before=fixture_dm_transmits;
        step();
        assert(rl_calls==compute_before+expected && fixture_dm_transmits==sends_before+expected);
        assert(fixture_sent_dm[0]==2 && fixture_sent_wheel[0]==0.2f);
        if(expected && i) { assert(control_time_debug.output_period_us==1000*MACHINE_RL_CTRL_DIV); }
    }
    assert(policy_releases==20);
    for(fault_case=0;fault_case<9;fault_case++)
    {
        rc_command.s1=DR16_SW_MID; step();
        rc_command.s1=DR16_SW_UP; step();
        compute_before=rl_calls; sends_before=fixture_dm_transmits;
        switch(fault_case)
        {
        case 0: ctrl_fault=4; break;
        case 1: robot_state.fallen=1; break;
        case 2: robot_state.motor_enabled=0; break;
        case 3: action_state.rl_ready=0; break;
        case 4: leg_r.output.valid=0; break;
        case 5: rc_command.s2=DR16_SW_UP; break;
        case 6: rc_command.online=0; break;
        case 7: torque_output_enabled=0; break;
        default: robot_state.rc_enable=0; break;
        }
        step(); assert(rl_calls==compute_before && fixture_dm_transmits==sends_before+1); assert_zero();
        ctrl_fault=robot_state.fallen=0;
        robot_state.rc_enable=robot_state.motor_enabled=action_state.rl_ready=1;
        leg_r.output.valid=rc_command.online=torque_output_enabled=1;
        rc_command.s2=DR16_SW_MID;
        step(); assert(rl_calls==compute_before+1 && fixture_sent_dm[0]==2);
    }
    rc_command.s1=DR16_SW_MID; step();
    rc_command.s1=DR16_SW_UP; fixture_fail_output=1;
    sends_before=fixture_dm_transmits; compute_before=rl_calls;
    step(); assert_zero(); step(); assert_zero();
    assert(rl_calls==compute_before+2 && fixture_dm_transmits==sends_before+2);
    fixture_fail_output=0; step(); assert(fixture_sent_dm[0]==2);
    rc_command.s1=DR16_SW_MID; sends_before=fixture_dm_transmits;
    step(); assert(fixture_sent_dm[0]==1 && fixture_dm_transmits==sends_before+1);
    assert(control_time_debug.period_us==1000);
    return 0;
}
"""

DECODER_CHECK = r"""
int main(void)
{
    rc_control_mode_t mode;
    unsigned left,right,expected_enable,available;
    fixture_machine=machine_table[MACHINE_DEFAULT];
    Robot_Enable_Update();assert(!robot_state.motor_enabled);
    rc_command.online=1;
    available=GAS_SPRING_COMP_ENABLE && machine->spring && machine->gravity;
    for(left=1;left<=3;left++) {
        for(right=1;right<=3;right++) {
            rc_command.s1=left;rc_command.s2=right;
            mode=Control_Mode_Decode(&rc_command);
            assert(mode.usb_permit==(left==DR16_SW_UP && right==DR16_SW_DOWN));
            assert(mode.usb_reset==(left==DR16_SW_DOWN));
            if(left==DR16_SW_MID && right==DR16_SW_UP) {
                assert(mode.strategy==(available?CTRL_STRATEGY_COMPENSATION:CTRL_STRATEGY_DISABLE));
                assert(mode.engage);expected_enable=available;
            } else if(left==DR16_SW_MID) {
                assert(mode.strategy==CTRL_STRATEGY_LQR);
                assert(mode.engage==(right==DR16_SW_MID));expected_enable=machine->lqr_configured;
            } else if(left==DR16_SW_UP) {
                assert(mode.strategy==CTRL_STRATEGY_RL);
                assert(mode.engage==(right==DR16_SW_MID));expected_enable=machine->rl.configured;
            } else {
                assert(mode.strategy==CTRL_STRATEGY_DISABLE && !mode.engage);expected_enable=0;
            }
            assert(mode.rc_enable==expected_enable && strategy_rc_enable(&rc_command)==expected_enable);
            rc_command.online=0;mode=Control_Mode_Decode(&rc_command);
            assert(mode.strategy==CTRL_STRATEGY_DISABLE && !mode.rc_enable && !mode.engage && !mode.usb_permit);
            rc_command.online=1;
        }
    }
    mode=Control_Mode_Decode(NULL);
    assert(mode.strategy==CTRL_STRATEGY_DISABLE && !mode.rc_enable && !mode.engage && !mode.usb_permit && !mode.usb_reset);
    fixture_machine.lqr_configured=0;rc_command.s1=DR16_SW_MID;rc_command.s2=DR16_SW_UP;
    mode=Control_Mode_Decode(&rc_command);assert(mode.rc_enable==available);
    return 0;
}
"""


class GasSpringBenchTest(unittest.TestCase):
    def test_real_actuation_and_arbiter(self):
        self.build_and_run(FIXTURE, CHECK)

    def test_default_rl_lqr_switches_and_output_gates(self):
        self.assertNotIn("gas_spring_only_enabled", read("imcalib/task/robot_control.c"))
        self.build_and_run(self.normal_fixture(), NORMAL_CHECK)

    def test_strategy_cadence_and_stop_on_skipped_rl_tick(self):
        fixture = self.normal_fixture()
        fixture = fixture.replace("static uint64_t Mono_Ns_Get(void) { return 1; }",
            "static uint64_t fixture_ns;\nstatic uint64_t Mono_Ns_Get(void) { return fixture_ns; }")
        fixture = fixture.replace("static uint8_t fixture_output(torque_output_t *t, float value)\n{\n    unsigned i;",
            "static uint8_t fixture_fail_output;\nstatic uint8_t fixture_output(torque_output_t *t, float value)\n{\n    unsigned i;\n    if(fixture_fail_output && value==2) { return 0; }")
        self.build_and_run(fixture, CADENCE_CHECK, instrument=True)

    def test_single_decoder_truth_table(self):
        self.build_and_run(FIXTURE, DECODER_CHECK)

    def normal_fixture(self):
        fixture = FIXTURE
        fixture = fixture.replace(
            "#define LQR_Target_Update(a,b,c) ((void)(a),(void)(b),(void)(c),0)",
            "#define LQR_Target_Update(a,b,c) ((void)(a),(void)(b),(void)(c),1)")
        fixture = fixture.replace(
            "#define LQR_Control_Update(a) ((void)(a),++lqr_calls)",
            "#define LQR_Control_Update(a) ((a)->gain_valid=1,++lqr_calls)")
        fixture = fixture.replace(
            "#define Leg_Balance_Compute(a,b,c,d,e,f) ((void)(a),(void)(b),(void)(c),(void)(d),(void)(e),(void)(f),0)",
            "#define Leg_Balance_Compute(a,b,c,d,e,f) ((void)(a),(void)(b),(void)(c),(void)(d),(void)(e),fixture_output(f,1))")
        fixture = fixture.replace(
            "#define RL_Torque_Compute(a,b,c,d,e,f,g) ((void)(a),(void)(b),(void)(c),(void)(d),(void)(e),(void)(f),(void)(g),++rl_calls,0)",
            "#define RL_Torque_Compute(a,b,c,d,e,f,g) ((void)(a),(void)(b),(void)(c),(void)(d),(void)(e),(void)(f),++rl_calls,fixture_output(g,2))")
        fixture += r"""
static uint8_t fixture_output(torque_output_t *t, float value)
{
    unsigned i;
    for (i=0;i<4;i++) { t->dm[i]=value; }
    t->dji[0]=t->dji[1]=value*0.1f;
    return 1;
}
"""
        return fixture

    def build_and_run(self, fixture, check, instrument=False):
        compiler = os.environ.get("CC") or shutil.which("gcc")
        if compiler is None:
            self.skipTest("no native C compiler; set CC to host gcc")
        environment = os.environ.copy()
        environment["PATH"] = str(Path(compiler).parent) + os.pathsep + environment.get("PATH", "")
        headers = HEADERS.replace("int unused; } rc_command_t", "uint8_t online, s1, s2; float vel, yaw, len; } rc_command_t")
        headers = headers.replace("typedef struct { int unused; } imu_state_t;", '#include "imu_state.h"\n#include "slip.h"')
        actuation = without_includes(read("imcalib/task/task_actuation.c"))
        actuation = actuation.replace("volatile float rl_output_dm_cmd_nm[DM_MOTOR_NUM];", "")
        actuation = actuation.replace("volatile float rl_output_wheel_cmd_nm[DJI_MOTOR_NUM];", "")
        stubs = STUBS
        timing = ""
        if instrument:
            stubs = "static unsigned fixture_dm_transmits;\n" + stubs.replace(
                "return fixture_dm_status;", "fixture_dm_transmits++; return fixture_dm_status;")
            timing = "#define CONTROL_TIME_VOFA_ENABLE 1\n" + re.search(
                r"typedef struct \{[^}]*\} control_time_debug_t;", read("imcalib/task/inc/robot_control.h")).group()
            timing += "\nstatic volatile control_time_debug_t control_time_debug;\n"
            timing += "static unsigned policy_releases; static int policy_tick_sem_handle;\n"
            timing += "static int osSemaphoreRelease(int h) { (void)h; policy_releases++; return 0; }\n"
            timing += production_function(read("imcalib/task/robot_control.c"), "Policy_Tick_Div")
        content = [headers,
                   without_includes(read("imcalib/user-lib/machine_config.c")).split("const machine_cfg_t *const machine")[0],
                   production_enum(read("imcalib/user-lib/dm.h"), "dm_motor_idx_t"),
                   "#define DJI_MOTOR_NUM 2\n#define DJI_MOTOR_WHEEL_LFT 0\n#define DJI_MOTOR_WHEEL_RGT 1",
                   without_includes(read("imcalib/Algorithm/torque_output.h")),
                   without_includes(read("imcalib/Algorithm/rl_torque.h")),
                   without_includes(read("imcalib/Algorithm/lqr_balance.h")),
                   without_includes(read("imcalib/Algorithm/leg_balance.h")),
                   without_includes(read("imcalib/Algorithm/standup.h")),
                   production_enum(read("imcalib/task/inc/robot_control.h"), "ctrl_strategy_t"),
                   re.search(r"typedef struct \{\s*ctrl_strategy_t strategy;[^}]*\} rc_control_mode_t;",
                       read("imcalib/task/inc/robot_control.h")).group(),
                   timing, stubs, fixture, without_includes(read("imcalib/Algorithm/gas_spring.c")),
                   without_includes(read("imcalib/Algorithm/gravity_comp.c")),
                   without_includes(read("imcalib/Algorithm/standup.c")),
                   actuation,
                   "static inline ctrl_strategy_t strategy_from_remote(const rc_command_t *cmd) { "
                   "rc_control_mode_t mode=Control_Mode_Decode(cmd); return strategy_from_mode(&mode); }",
                   production_function(read("imcalib/task/task_comm.c"), "Robot_Enable_Update"), check]
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
                            str(source), str(ROOT / "imcalib/Algorithm/leg_solver.c"), str(ROOT / "imcalib/Algorithm/slip.c"), str(ROOT / "imcalib/user-lib/pid.c"), "-lm", "-o", str(exe)],
                            capture_output=True, text=True, encoding="utf-8", errors="replace", env=environment)
                        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                        result = subprocess.run([str(exe)], capture_output=True, text=True, encoding="utf-8", errors="replace", env=environment)
                        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
