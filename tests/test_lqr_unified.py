"""Execute both machines' common LQR sources and replay the old small controller.

Hardware input is a fixture. Geometry, estimators, PID, gains, force mapping and
spring composition are production C. libm replaces CMSIS in host tests.
"""
import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import unittest

from test_gas_spring_integration import ARM_MATH_SHIM, without_includes

ROOT = Path(__file__).resolve().parents[1]


def read(path):
    return (ROOT / path).read_text(encoding="utf-8")


PREFIX = r"""
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "arm_math.h"
#include "machine_config.h"
#include "leg_solver.h"
#include "simple-function.h"
#include "kalman.h"
#include "pid.h"
#include "imu_state.h"
#include "lqr_gain_table.h"
typedef struct { int unused; } dr16_t;
#define DM_MOTOR_NUM 4
#define DJI_MOTOR_NUM 2
#define DJI_MOTOR_WHEEL_LFT 0
#define DJI_MOTOR_WHEEL_RGT 1
"""

CHECK = r"""
static void near(float a, float b)
{
    if (!isfinite(a) || !isfinite(b) || fabsf(a-b)>0.00001f)
    {
        fprintf(stderr,"%g != %g\n",a,b); assert(0);
    }
}
static leg_state_t legs[2];
static imu_state_t imu;
static rc_command_t remote;
static const float wheels[2]={0.4f,-0.2f};
static void setup(void)
{
    unsigned i;
    memset(legs,0,sizeof(legs)); memset(&imu,0,sizeof(imu));
    memset(&remote,0,sizeof(remote)); remote.online=1;
    imu.online=1; imu.quat[0]=1; imu.acc_g[2]=1;
    imu.euler_rad[ATTITUDE_PITCH]=0.02f;
    imu.euler_rad[ATTITUDE_ROLL]=-0.01f;
    imu.gyro_rad_s[1]=0.03f;
    for(i=0;i<2;i++)
    {
        legs[i].config.lu=machine->leg_lu; legs[i].config.lg=machine->leg_lg;
        legs[i].config.offset_phi0=machine->leg_off_phi0[i];
        legs[i].config.configured=1;
        legs[i].input.hip_f=2.65f+0.05f*i; legs[i].input.hip_b=0.45f;
        legs[i].input.d_hip_f=0.01f; legs[i].input.d_hip_b=-0.01f;
        assert(Leg_Solve(&legs[i]) && legs[i].output.force_valid);
    }
}
static void metadata(void)
{
    lqr_gain_desc_t d=*LQR_Gain_Info(); machine_cfg_t cfg=*machine;
    float k[40], edge[40]; unsigned i;
    assert(d.machine_id==MACHINE_DEFAULT && LQR_Gain_Compatible());
    assert(MACHINE_TIM6_PERIOD==999 && MACHINE_LQR_DT==0.001f);
    near(machine->lqr.dt,MACHINE_LQR_DT);
#if MACHINE_DEFAULT == MACHINE_ID_BIG_WHEELLEG
    {
        lowpass1d_t lp;
        Lowpass_Init(&lp,machine->lqr.lpf_alpha[0]);
        Lowpass_Update(&lp,1.0f);
        near(Lowpass_Update(&lp,1.0f),0.51f);
        near(machine->lqr.leg_len[0].kd*machine->lqr.dt,25.0f);
        near(machine->lqr.roll.kd*machine->lqr.dt,0.1f);
        near(machine->lqr.kf_q/machine->lqr.dt,7.0f);
    }
#endif
    assert(LQR_Gain_Check(&d,&cfg,MACHINE_DEFAULT,MACHINE_CTRL_DT)==LQR_GAIN_OK);
    d.machine_id=2;
    assert(LQR_Gain_Check(&d,&cfg,MACHINE_DEFAULT,MACHINE_CTRL_DT)==LQR_GAIN_BAD_MACHINE);
    d=*LQR_Gain_Info(); d.state_schema=0;
    assert(LQR_Gain_Check(&d,&cfg,MACHINE_DEFAULT,MACHINE_CTRL_DT)==LQR_GAIN_BAD_SCHEMA);
    d=*LQR_Gain_Info(); d.dt*=2;
    assert(LQR_Gain_Check(&d,&cfg,MACHINE_DEFAULT,MACHINE_CTRL_DT)==LQR_GAIN_BAD_PERIOD);
    d=*LQR_Gain_Info(); cfg.wheel_r*=1.1f;
    assert(LQR_Gain_Check(&d,&cfg,MACHINE_DEFAULT,MACHINE_CTRL_DT)==LQR_GAIN_BAD_GEOMETRY);
    cfg=*machine; cfg.lqr.leg_len_init[0]=0;
    assert(LQR_Gain_Check(&d,&cfg,MACHINE_DEFAULT,MACHINE_CTRL_DT)==LQR_GAIN_BAD_DOMAIN);
    assert(!LQR_Gain_Eval(NAN,0.2f,k,0));
    for(i=0;i<40;i++) { assert(k[i]==0); }
    assert(LQR_Gain_Eval(-100,100,k,0));
    assert(LQR_Gain_Eval(d.len_min,d.len_max,edge,0));
    for(i=0;i<40;i++) { assert(k[i]==edge[i]); }
    assert(LQR_Ready()==machine->lqr_configured);
}
static void chain(void)
{
    lqr_state_t s; leg_balance_t balance; torque_output_t torque;
    unsigned i;
    setup(); LQR_Init(&s); Leg_Balance_Init(&balance);
    assert(LQR_State_Update(&s,&imu,&legs[0],&legs[1],wheels,MACHINE_CTRL_DT));
    assert(LQR_Enable_Latch(&s,&legs[0],&legs[1]));
    remote.vel=0.1f; remote.yaw=-0.1f;
    assert(LQR_Target_Update(&s,&remote,MACHINE_CTRL_DT));
    LQR_Control_Update(&s); assert(s.gain_valid);
    Torque_Output_Clear(&torque);
    assert(Leg_Balance_Compute(&balance,&s,&legs[0],&legs[1],MACHINE_CTRL_DT,&torque));
    for(i=0;i<4;i++) { assert(isfinite(torque.dm[i])); assert(fabsf(torque.dm[i])<=lqr_debug.trq_max_hip); }
    near(balance.Tp[0],-s.u[LQR_U_BL]); near(balance.Tp[1],-s.u[LQR_U_BR]);
    for(i=0;i<2;i++) { near(torque.dji[i],s.u[i]); }
    lqr_debug.len_pid_enable=0; lqr_debug.hip_enable=0;
    assert(Leg_Balance_Compute(&balance,&s,&legs[0],&legs[1],MACHINE_CTRL_DT,&torque));
    near(balance.F[0],machine->lqr.support_force[0]);
    near(balance.F[1],machine->lqr.support_force[1]);
    near(balance.Tp[0],0); near(balance.Tp[1],0);
}
static void length_history_startup(void)
{
    lqr_state_t state;
    leg_balance_t balance;
    torque_output_t torque;
    float last_error;
    float error;

    setup(); LQR_Init(&state); Leg_Balance_Init(&balance);
    assert(!balance.len_history_ready);
    balance.len_prime_enable = 0;
    state.leg_len_tgt[0] = legs[0].output.virtual_leg_length - 0.04f;
    state.leg_len_tgt[1] = legs[1].output.virtual_leg_length + 0.03f;
    state.u[LQR_U_WL] = 0.6f; state.u[LQR_U_WR] = -0.7f;
    state.u[LQR_U_BL] = 0.8f; state.u[LQR_U_BR] = -0.9f;

    assert(Leg_Balance_Compute(&balance,&state,&legs[0],&legs[1],MACHINE_CTRL_DT,&torque));
    near(balance.leg_len[0].dout,balance.leg_len[0].d*balance.leg_len[0].err[NOW]);
    near(balance.leg_len[1].dout,balance.leg_len[1].d*balance.leg_len[1].err[NOW]);
    Leg_Balance_Reset(&balance);
    balance.len_prime_enable = 1;

    legs[0].output.valid = 0;
    assert(!Leg_Balance_Compute(&balance,&state,&legs[0],&legs[1],MACHINE_CTRL_DT,&torque));
    assert(balance.leg_len[0].err[LAST] == 0.0f);
    assert(!balance.len_history_ready);
    legs[0].output.valid = 1;
    state.leg_len_tgt[0] = NAN;
    assert(!Leg_Balance_Compute(&balance,&state,&legs[0],&legs[1],MACHINE_CTRL_DT,&torque));
    assert(!balance.len_history_ready);
    state.leg_len_tgt[0] = legs[0].output.virtual_leg_length - 0.04f;
    assert(Leg_Balance_Compute(&balance,&state,&legs[0],&legs[1],MACHINE_CTRL_DT,&torque));
    assert(balance.len_history_ready);
    near(balance.leg_len[0].dout,0.0f); near(balance.leg_len[1].dout,0.0f);
    near(balance.leg_len[0].pout,balance.leg_len[0].p*balance.leg_len[0].err[NOW]);
    near(balance.leg_len[1].pout,balance.leg_len[1].p*balance.leg_len[1].err[NOW]);
    near(balance.Tp[0],-state.u[LQR_U_BL]); near(balance.Tp[1],-state.u[LQR_U_BR]);
    near(torque.dji[0],state.u[LQR_U_WL]); near(torque.dji[1],state.u[LQR_U_WR]);

    last_error = balance.leg_len[0].err[LAST];
    legs[0].input.hip_f += 0.001f; assert(Leg_Solve(&legs[0]));
    error = state.leg_len_tgt[0] - legs[0].output.virtual_leg_length;
    assert(fabsf(error-last_error)>0.000001f);
    assert(Leg_Balance_Compute(&balance,&state,&legs[0],&legs[1],MACHINE_CTRL_DT,&torque));
    near(balance.leg_len[0].dout,balance.leg_len[0].d*(error-last_error));

    Leg_Balance_Reset(&balance);
    assert(balance.len_prime_enable && !balance.len_history_ready);
    state.leg_len_tgt[0] -= 0.02f; state.leg_len_tgt[1] += 0.01f;
    assert(Leg_Balance_Compute(&balance,&state,&legs[0],&legs[1],MACHINE_CTRL_DT,&torque));
    near(balance.leg_len[0].dout,0.0f); near(balance.leg_len[1].dout,0.0f);

    balance.len_prime_enable = 0;
    Leg_Balance_Reset(&balance);
    assert(!balance.len_prime_enable && !balance.len_history_ready);
    assert(Leg_Balance_Compute(&balance,&state,&legs[0],&legs[1],MACHINE_CTRL_DT,&torque));
    near(balance.leg_len[0].dout,balance.leg_len[0].d*balance.leg_len[0].err[NOW]);
    near(balance.leg_len[1].dout,balance.leg_len[1].d*balance.leg_len[1].err[NOW]);
}

static void command_yaw_direction(void)
{
    lqr_state_t state;
    unsigned i;
    setup(); LQR_Init(&state);
    state.valid=1; state.len[0]=state.len[1]=0.18f;
    state.x[LQR_X_S]=machine->lqr.pos_target;
    state.x[LQR_X_THL]=machine->lqr.leg_trim[0];
    state.x[LQR_X_THR]=machine->lqr.leg_trim[1];
    state.x[LQR_X_THB]=machine->lqr.pitch_trim;
    remote.yaw=0.3f;
    assert(LQR_Target_Update(&state,&remote,MACHINE_CTRL_DT));
    assert(state.target[LQR_X_DPHI]>0);
    LQR_Control_Update(&state);
    assert(state.gain_valid && state.u[LQR_U_WL]<0 && state.u[LQR_U_WR]>0);
    remote.yaw=-0.3f;
    assert(LQR_Target_Update(&state,&remote,MACHINE_CTRL_DT));
    assert(state.target[LQR_X_DPHI]<0);
    LQR_Control_Update(&state);
    assert(state.gain_valid && state.u[LQR_U_WL]>0 && state.u[LQR_U_WR]<0);
    remote.yaw=0;
    assert(LQR_Target_Update(&state,&remote,MACHINE_CTRL_DT));
    near(state.target[LQR_X_DPHI],0);
    LQR_Control_Update(&state);
    for(i=0;i<4;i++) { near(state.u[i],0); }
}
static void invalid(void)
{
    lqr_state_t s; float bad[2]={NAN,0}; unsigned i;
    setup(); LQR_Init(&s);
    remote.vel=NAN; assert(!LQR_Target_Update(&s,&remote,MACHINE_CTRL_DT)); remote.vel=0;
    assert(!LQR_State_Update(&s,&imu,&legs[0],&legs[1],bad,MACHINE_CTRL_DT));
    imu.gyro_rad_s[1]=INFINITY;
    assert(!LQR_State_Update(&s,&imu,&legs[0],&legs[1],wheels,MACHINE_CTRL_DT));
    imu.gyro_rad_s[1]=0.03f;
    assert(LQR_State_Update(&s,&imu,&legs[0],&legs[1],wheels,MACHINE_CTRL_DT));
    s.target[9]=NAN; LQR_Control_Update(&s); assert(!s.gain_valid);
    for(i=0;i<4;i++) { assert(s.u[i]==0); }
    s.target[9]=0; LQR_Control_Update(&s); assert(s.gain_valid);
    s.valid=0; LQR_Control_Update(&s); assert(!s.gain_valid);
    for(i=0;i<4;i++) { assert(s.u[i]==0); }
}
static void geometry_power(void)
{
    leg_state_t plus, minus;
    float tau[2], derivative, expected;
    unsigned side, joint;
    setup();
    for(side=0;side<2;side++)
    {
        for(joint=0;joint<2;joint++)
        {
            plus=minus=legs[side];
            if(joint==0) { plus.input.hip_f+=0.001f; minus.input.hip_f-=0.001f; }
            else { plus.input.hip_b+=0.001f; minus.input.hip_b-=0.001f; }
            assert(Leg_Solve(&plus) && Leg_Solve(&minus));
            derivative=(plus.output.virtual_leg_length-minus.output.virtual_leg_length)/0.002f;
            assert(fabsf(derivative-legs[side].output.leg_jac[0][joint])<0.0003f);
            derivative=(plus.output.virtual_leg_angle-minus.output.virtual_leg_angle)/0.002f;
            assert(fabsf(derivative-legs[side].output.leg_jac[1][joint])<0.0003f);
        }
        assert(Leg_Force_Map_Forward(&legs[side],25.0f,1.2f,tau));
        expected=25.0f*legs[side].output.d_virtual_leg_length+1.2f*legs[side].output.d_virtual_leg_angle;
        near(tau[0]*legs[side].input.d_hip_f+tau[1]*legs[side].input.d_hip_b,expected);
    }
}
static void world_leg_conversion(void)
{
    lqr_state_t state;
    leg_state_t left, right;
    imu_state_t attitude;
    float initial[2];
    const float wheel[2]={0,0};
    unsigned i;
    setup(); left=legs[0]; right=legs[1]; attitude=imu;
    attitude.euler_rad[LQR_IMU_PITCH_IDX]=0;
    attitude.gyro_rad_s[1]=0;
    LQR_Init(&state);
    assert(LQR_State_Update(&state,&attitude,&left,&right,wheel,MACHINE_CTRL_DT));
    near(state.x[LQR_X_THL],left.output.virtual_leg_angle);
    near(state.x[LQR_X_THR],right.output.virtual_leg_angle);
    initial[0]=state.x[LQR_X_THL]; initial[1]=state.x[LQR_X_THR];
    /* 机身旋转、世界系腿方向不变。 */
    left.input.hip_f-=0.08f; left.input.hip_b-=0.08f;
    right.input.hip_f-=0.08f; right.input.hip_b-=0.08f;
    left.input.d_hip_f=left.input.d_hip_b=-0.2f;
    right.input.d_hip_f=right.input.d_hip_b=-0.2f;
    assert(Leg_Solve(&left) && Leg_Solve(&right));
    attitude.euler_rad[LQR_IMU_PITCH_IDX]=0.08f;
    attitude.gyro_rad_s[1]=0.2f;
    state.lpf_omg_pitch.out=0.2f;
    assert(LQR_State_Update(&state,&attitude,&left,&right,wheel,MACHINE_CTRL_DT));
    near(state.x[LQR_X_THL],initial[0]); near(state.x[LQR_X_THR],initial[1]);
    near(state.x[LQR_X_DTHL],0); near(state.x[LQR_X_DTHR],0);
    for(i=0;i<2;i++) { near(state.len[i],legs[i].output.virtual_leg_length); }
}
#if MACHINE_DEFAULT == MACHINE_ID_SMALL_WHEELLEG
static void replay(void)
{
    lqr_state_t old, now; leg_balance_t bo,bn; torque_output_t to,tn;
    rc_command_t old_remote;
    unsigned t,i;
    setup(); Baseline_LQR_Init(&old); LQR_Init(&now); lqr_debug.legacy_gain=1;
    Baseline_Leg_Balance_Init(&bo); Leg_Balance_Init(&bn);
    bn.len_prime_enable=0; /* Strict historical startup comparison. */
    Baseline_LQR_Enable_Latch(&old,&legs[0],&legs[1]);
    LQR_Enable_Latch(&now,&legs[0],&legs[1]);
    for(t=0;t<100;t++)
    {
        remote.vel=t<50?0.05f:0; remote.yaw=t%3?0:0.03f; remote.len=t<20?0.02f:0;
        old_remote=remote; old_remote.yaw=-remote.yaw;
        Baseline_LQR_Target_Update(&old,&old_remote,MACHINE_CTRL_DT);
        LQR_Target_Update(&now,&remote,MACHINE_CTRL_DT);
        assert(Baseline_LQR_State_Update(&old,&imu,&legs[0],&legs[1],wheels,MACHINE_CTRL_DT));
        assert(LQR_State_Update(&now,&imu,&legs[0],&legs[1],wheels,MACHINE_CTRL_DT));
        Baseline_LQR_Control_Update(&old); LQR_Control_Update(&now); assert(now.gain_valid);
        for(i=0;i<10;i++)
        {
            float basis=(i>=4 && i<=7)?-1.0f:1.0f;
            near(old.x[i],basis*now.x[i]); near(old.target[i],basis*now.target[i]);
        }
        for(i=0;i<4;i++) { near(old.u[i],now.u[i]); }
        Torque_Output_Clear(&to); Torque_Output_Clear(&tn);
        assert(Baseline_Leg_Balance_Compute(&bo,&old,&legs[0],&legs[1],MACHINE_CTRL_DT,&to));
        assert(Leg_Balance_Compute(&bn,&now,&legs[0],&legs[1],MACHINE_CTRL_DT,&tn));
        for(i=0;i<4;i++) { near(to.dm[i],tn.dm[i]); }
        for(i=0;i<2;i++) { near(to.dji[i],tn.dji[i]); }
    }
    lqr_debug.legacy_gain=0; LQR_Control_Update(&now); assert(now.gain_valid && !now.gain_legacy);
    assert(fabsf(old.K[0][4]+now.K[0][4])>0.01f);
}
#endif
int main(int argc,char **argv)
{
    if(argc==4)
    {
        float k[40]; unsigned i;
        assert(LQR_Gain_Eval((float)atof(argv[2]),(float)atof(argv[3]),k,0));
        for(i=0;i<40;i++) { printf("%.9g\n",k[i]); }
        return 0;
    }
    metadata(); chain(); length_history_startup(); command_yaw_direction(); invalid(); geometry_power(); world_leg_conversion();
#if MACHINE_DEFAULT == MACHINE_ID_SMALL_WHEELLEG
    replay();
#endif
    return 0;
}
"""


def baseline():
    text = "\n".join(re.findall(r"^#define\s+[^\n]+", read("tests/fixtures/baseline_lqr_balance.h"), re.M)[1:])
    text += "\n" + "\n".join(re.findall(r"^#define LEG_BALANCE_\w+[^\n]*", read("tests/fixtures/baseline_leg_balance.h"), re.M))
    text += "\n" + read("tools/matlab/baseline/lqr_gain_small_legacy.inc").replace("LQR_K_Small_Legacy", "Baseline_K")
    for filename in ("lqr_balance.c", "leg_balance.c"):
        source = without_includes(read("tests/fixtures/baseline_" + filename))
        # Keep the current user-selected IMU axes on both sides of the replay;
        # this regression isolates the leg-state convention, not calibration.
        current = read("imcalib/Algorithm/lqr_balance.c")
        for macro, value in re.findall(r"^#define (LQR_IMU_\w+)\s+([^\n]+)", current, re.M):
            source = re.sub(r"^#define " + macro + r"\s+[^\n]+", "#define " + macro + " " + value, source, flags=re.M)
        source = re.sub(r"\bLQR_IMU_(\w+)\b", r"BASELINE_LQR_IMU_\1", source)
        for name in ("LQR_Init", "LQR_Enable_Latch", "LQR_Target_Update", "LQR_State_Update",
                     "LQR_Control_Update", "LQR_Wrap_Pi", "LQR_Len_Range", "LQR_Accel_Forward",
                     "Leg_Balance_Init", "Leg_Balance_Reset", "Leg_Balance_Compute", "Leg_Balance_Output"):
            source = re.sub(r"\b" + name + r"\b", "Baseline_" + name, source)
        source = source.replace("LQR_K_WBR", "Baseline_K").replace("lqr_debug", "baseline_debug").replace("baseline_debug_t", "lqr_debug_t")
        text += "\n" + source
    return "#if MACHINE_DEFAULT == MACHINE_ID_SMALL_WHEELLEG\n" + text + "\n#endif\n"


class UnifiedLqrTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = os.environ.get("CC") or shutil.which("gcc")
        if not compiler:
            raise unittest.SkipTest("set CC to a native C compiler")
        cls.temp = tempfile.TemporaryDirectory(prefix="lqr-unified-")
        cls.addClassCleanup(cls.temp.cleanup)
        folder = Path(cls.temp.name)
        cls.environment = os.environ.copy()
        cls.environment["PATH"] = str(Path(compiler).parent) + os.pathsep + cls.environment.get("PATH", "")
        (folder / "arm_math.h").write_text(ARM_MATH_SHIM)
        source = PREFIX
        for path in ("imcalib/user-lib/rc_command.h", "imcalib/Algorithm/torque_output.h",
                     "imcalib/Algorithm/lqr_balance.h", "imcalib/Algorithm/leg_balance.h",
                     "imcalib/Algorithm/gas_spring.h", "imcalib/Algorithm/lqr_balance.c",
                     "imcalib/Algorithm/leg_balance.c", "imcalib/Algorithm/gas_spring.c"):
            source += "\n" + without_includes(read(path))
        source += "\n" + baseline() + "\n" + CHECK
        harness = folder / "controller.c"
        harness.write_text(source, encoding="utf-8")
        cls.executables = {}
        for machine_id in (0, 1):
            executable = folder / (f"lqr_{machine_id}" + (".exe" if os.name == "nt" else ""))
            sources = ["imcalib/user-lib/machine_config.c", "imcalib/Algorithm/lqr_gain_table.c",
                       "imcalib/Algorithm/lqr_gain_small.c", "imcalib/Algorithm/lqr_gain_big.c",
                       "imcalib/Algorithm/leg_solver.c", "imcalib/user-lib/pid.c",
                       "imcalib/user-lib/simple-function.c", "imcalib/user-lib/kalman.c"]
            result = subprocess.run([compiler, "-std=c99", "-Wall", "-Wextra", "-Werror",
                                     "-DLEG_TRIG_LIBM=1", f"-DMACHINE_DEFAULT={machine_id}",
                                     "-I", str(folder), "-I", str(ROOT / "imcalib/Algorithm"),
                                     "-I", str(ROOT / "imcalib/user-lib"), str(harness),
                                     *[str(ROOT / p) for p in sources], "-lm", "-o", str(executable)],
                                    capture_output=True, text=True, encoding="utf-8", errors="replace", env=cls.environment)
            if result.returncode:
                raise AssertionError(result.stdout + result.stderr)
            cls.executables[machine_id] = executable

    def test_common_chain_metadata_failures_and_small_baseline(self):
        for machine_id, executable in self.executables.items():
            with self.subTest(machine=machine_id):
                result = subprocess.run([str(executable)], capture_output=True, text=True, encoding="utf-8", errors="replace", env=self.environment)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_compiled_gain_matches_generated_coefficients_at_asymmetric_lengths(self):
        # An independent float32 evaluator also catches left/right grid reversal
        # and state-major vs output-major export mistakes.
        import struct
        f32 = lambda x: struct.unpack("f", struct.pack("f", x))[0]
        for machine_id, kind in ((0, "big"), (1, "small")):
            l, r = (0.19, 0.27) if machine_id == 0 else (0.15, 0.21)
            output = subprocess.check_output([str(self.executables[machine_id]), "gain", str(l), str(r)],
                                             text=True, env=self.environment)
            actual = [float(x) for x in output.split()]
            coefficients = read(f"imcalib/Algorithm/lqr_gain_{kind}.c")
            values = {"lL": f32(l), "lR": f32(r)}
            values.update(t2=f32(values["lL"]**2), t3=f32(values["lR"]**2), t4=f32(values["lL"]*values["lR"]))
            values.update(t5=f32(values["t2"]*values["lL"]), t6=f32(values["t2"]*values["lR"]),
                          t7=f32(values["lL"]*values["t3"]), t8=f32(values["t3"]*values["lR"]))
            rows = re.findall(r"K_sym\[\s*(\d+)\] = ([^;]+);", coefficients)
            self.assertEqual(len(rows), 40)
            for index, expression in rows:
                first, *terms = re.split(r" ([+-]) ", expression)
                expected = f32(float(first.rstrip("F")))
                for sign, term in zip(terms[::2], terms[1::2]):
                    c, variable = term.split(" * ")
                    product = f32(f32(float(c.rstrip("F"))) * values[variable])
                    expected = f32(expected + (product if sign == "+" else -product))
                self.assertAlmostEqual(actual[int(index)], expected, delta=max(1e-6, abs(expected)*1e-8))


if __name__ == "__main__":
    unittest.main()
