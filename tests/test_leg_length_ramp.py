"""Exercise real length PID inputs with discontinuous upstream targets."""
import unittest

import test_standup


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

    (void)step;
    (void)zero;

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
    return 0;
}
"""


class LegLengthRampTest(unittest.TestCase):
    def test_real_length_controller_inputs(self):
        test_standup.StandupTest().run_cases({"length_ramp": CHECK})
