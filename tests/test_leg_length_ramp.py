"""Exercise real length PID inputs with discontinuous upstream targets."""
import unittest
import os
from pathlib import Path
import shutil
import subprocess
import tempfile

from test_lqr_unified import PREFIX
from test_gas_spring_integration import ARM_MATH_SHIM, ROOT, read, without_includes

FIXTURE = r"""
static machine_cfg_t fixture_machine;
const machine_cfg_t *const machine=&fixture_machine;
uint8_t Machine_Id(void) { return MACHINE_DEFAULT; }
static leg_state_t leg_l,leg_r;
static lqr_state_t lqr_state;
static leg_balance_t leg_balance;
static void setup(float angle)
{
    fixture_machine=machine_table[MACHINE_DEFAULT];
    memset(&leg_l,0,sizeof(leg_l));memset(&leg_r,0,sizeof(leg_r));
    leg_l.output.valid=leg_l.output.force_valid=1;
    leg_l.output.virtual_leg_length=.30f;
    leg_l.output.virtual_leg_angle=angle;
    leg_l.output.force_map[0][0]=.05f;leg_l.output.force_map[0][1]=.5f;
    leg_l.output.force_map[1][0]=-.05f;leg_l.output.force_map[1][1]=.5f;
    leg_r=leg_l;
    LQR_Init(&lqr_state);Leg_Balance_Init(&leg_balance);Standup_Init(&standup_control);
}
"""


CHECK = r"""
static void close_length(float actual, float expected)
{
    assert(isfinite(actual) && fabsf(actual-expected)<0.000002f);
}
int main(void)
{
    ramp_t ramp={0};
    torque_output_t torque;
    float previous;
    float step_length;
    unsigned i;

    /* New target moves from measured length, then from previous output. */
    close_length(Ramp_Target_Update(&ramp,.15f,.30f,.3f,.001f),.1503f);
    close_length(Ramp_Target_Update(&ramp,.10f,.30f,.3f,.001f),.1506f);
    close_length(Ramp_Target_Update(&ramp,.10f,.13f,.3f,.002f),.1500f);
    previous=ramp.out;
    assert(isnan(Ramp_Target_Update(&ramp,.15f,NAN,.3f,.001f)));
    assert(isnan(Ramp_Target_Update(&ramp,.15f,.30f,.3f,0)));
    close_length(ramp.out,previous);
    for(i=0;i<1000;i++) { Ramp_Target_Update(&ramp,.10f,.30f,.3f,.001f); }
    close_length(ramp.out,.30f);

    /* Normal controller consumes smoothed input; raw target stays intact. */
    setup(0);
    step_length=machine->lqr.len_rate*MACHINE_LQR_DT;
    leg_l.output.virtual_leg_length=.30f;
    leg_r.output.virtual_leg_length=.15f;
    lqr_state.leg_len_tgt[0]=.15f;
    lqr_state.leg_len_tgt[1]=.30f;
    Torque_Output_Clear(&torque);
    assert(Leg_Balance_Compute(&leg_balance,&lqr_state,&leg_l,&leg_r,MACHINE_LQR_DT,&torque));
    close_length(leg_balance.leg_len[0].set[NOW],.30f-step_length);
    close_length(leg_balance.leg_len[1].set[NOW],.15f+step_length);
    close_length(leg_balance.leg_len[0].dout,0);
    assert(lqr_state.leg_len_tgt[0]==.15f && lqr_state.leg_len_tgt[1]==.30f);
    previous=leg_balance.leg_len[0].set[NOW];
    leg_l.output.virtual_leg_length=.25f;
    assert(Leg_Balance_Compute(&leg_balance,&lqr_state,&leg_l,&leg_r,MACHINE_LQR_DT,&torque));
    close_length(leg_balance.leg_len[0].set[NOW],previous-step_length);
    lqr_state.leg_len_tgt[0]=.31f;
    previous=leg_balance.leg_len[0].set[NOW];
    assert(Leg_Balance_Compute(&leg_balance,&lqr_state,&leg_l,&leg_r,MACHINE_LQR_DT,&torque));
    close_length(leg_balance.leg_len[0].set[NOW],previous+step_length);
    Leg_Balance_Reset(&leg_balance);
    assert(Leg_Balance_Compute(&leg_balance,&lqr_state,&leg_l,&leg_r,MACHINE_LQR_DT,&torque));
    close_length(leg_balance.leg_len[0].set[NOW],.25f+step_length);

    /* Self righting uses the same ramp; PID history priming cannot reset it. */
    setup(standup_param.rear_angle);
    standup_param.length_rate=machine->lqr.len_rate*2.0f;
    step_length=standup_param.length_rate*MACHINE_LQR_DT;
    standup_control.phase=STANDUP_RETRACT;
    standup_control.length_cmd[0]=.15f;
    standup_control.length_cmd[1]=.30f;
    leg_l.output.virtual_leg_length=.30f;
    leg_r.output.virtual_leg_length=.15f;
    assert(Standup_PID_Calculate(&standup_control,
        (const leg_state_t *const[2]){&leg_l,&leg_r},MACHINE_LQR_DT));
    close_length(standup_control.length_pid[0].set[NOW],.30f-step_length);
    close_length(standup_control.length_pid[1].set[NOW],.15f+step_length);
    assert(standup_control.tp[0]==0 && standup_control.tp[1]==0);
    close_length(standup_control.length_pid[0].dout,0);
    previous=standup_control.length_pid[0].set[NOW];
    standup_control.len_history_ready=0;
    leg_l.output.virtual_leg_length=.20f;
    standup_control.phase=STANDUP_SWING;
    assert(Standup_PID_Calculate(&standup_control,
        (const leg_state_t *const[2]){&leg_l,&leg_r},MACHINE_LQR_DT));
    close_length(standup_control.length_pid[0].set[NOW],previous-step_length);
    assert(standup_control.length_cmd[0]==.15f && standup_control.length_cmd[1]==.30f);
    Leg_Balance_Reset(&leg_balance);
    lqr_state.leg_len_tgt[0]=.15f;
    lqr_state.leg_len_tgt[1]=.30f;
    assert(Leg_Balance_Compute(&leg_balance,&lqr_state,&leg_l,&leg_r,MACHINE_LQR_DT,&torque));
    close_length(leg_balance.leg_len[0].set[NOW],.20f-machine->lqr.len_rate*MACHINE_LQR_DT);
    close_length(leg_balance.leg_len[1].set[NOW],.15f+machine->lqr.len_rate*MACHINE_LQR_DT);
    return 0;
}
"""


class LegLengthRampTest(unittest.TestCase):
    def test_real_length_controller_inputs(self):
        self.run_check(CHECK)

    def run_check(self, check):
        compiler=os.environ.get('CC') or shutil.which('gcc')
        if compiler is None:
            self.skipTest('set CC to host gcc')
        env=os.environ.copy()
        env['PATH']=str(Path(compiler).parent)+os.pathsep+env.get('PATH','')
        content=PREFIX
        for path in ('imcalib/user-lib/rc_command.h','imcalib/Algorithm/torque_output.h',
                     'imcalib/Algorithm/lqr_balance.h','imcalib/Algorithm/leg_balance.h',
                     'imcalib/Algorithm/gas_spring.h','imcalib/Algorithm/standup.h'):
            content+='\n'+without_includes(read(path))
        content+='\n'+without_includes(read('imcalib/user-lib/machine_config.c')).split('const machine_cfg_t *const machine')[0]
        for path in ('imcalib/Algorithm/lqr_balance.c','imcalib/Algorithm/leg_balance.c',
                     'imcalib/Algorithm/gas_spring.c','imcalib/Algorithm/gravity_comp.c','imcalib/Algorithm/standup.c'):
            content+='\n'+without_includes(read(path))
        content+='\n'+FIXTURE+check
        with tempfile.TemporaryDirectory(prefix='length-ramp-') as temp:
            folder=Path(temp)
            (folder/'arm_math.h').write_text(ARM_MATH_SHIM,encoding='utf-8')
            source=folder/'check.c';source.write_text(content,encoding='utf-8')
            for machine_id in (0,1):
                with self.subTest(machine=machine_id):
                    exe=folder/f'check{machine_id}.exe'
                    dependencies=('imcalib/Algorithm/lqr_gain_table.c','imcalib/Algorithm/lqr_gain_big.c',
                                  'imcalib/Algorithm/lqr_gain_small.c','imcalib/Algorithm/leg_solver.c',
                                  'imcalib/user-lib/pid.c','imcalib/user-lib/simple-function.c')
                    r=subprocess.run([compiler,'-std=c99','-Wall','-Wextra','-Werror','-DLEG_TRIG_LIBM=1',
                        f'-DMACHINE_DEFAULT={machine_id}','-I',str(folder),'-I',str(ROOT/'imcalib/Algorithm'),
                        '-I',str(ROOT/'imcalib/user-lib'),str(source),*[str(ROOT/p) for p in dependencies],
                        '-lm','-o',str(exe)],capture_output=True,text=True,env=env)
                    self.assertEqual(r.returncode,0,r.stdout+r.stderr)
                    r=subprocess.run([str(exe)],capture_output=True,text=True,env=env)
                    self.assertEqual(r.returncode,0,r.stdout+r.stderr)
