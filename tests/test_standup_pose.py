"""Real self-righting decisions ignore roll and retain inverted recovery."""
import unittest

import test_leg_length_ramp


CHECK = r"""
static void attitude(imu_state_t *imu, float pitch, float roll)
{
    memset(imu,0,sizeof(*imu));
    imu->online=imu->pitch_world_valid=1;
    imu->pitch_world=imu->euler_rad[1]=pitch;
    imu->euler_rad[0]=roll;
    imu->quat[0]=cosf(pitch*.5f)*cosf(roll*.5f);
    imu->quat[1]=cosf(pitch*.5f)*sinf(roll*.5f);
    imu->quat[2]=sinf(pitch*.5f)*cosf(roll*.5f);
    imu->quat[3]=-sinf(pitch*.5f)*sinf(roll*.5f);
}
int main(void)
{
    const leg_state_t *legs[2]={&leg_l,&leg_r};
    imu_state_t imu;
    torque_output_t torque;
    float rolls[5]={0,.6f,1.3f,1.55f,-1.55f};
    unsigned i,j;
    uint8_t route;

    for(i=0;i<5;i++)
    {
        setup(machine_table[MACHINE_DEFAULT].lqr.leg_trim[0]);
        leg_r.output.virtual_leg_angle=machine->lqr.leg_trim[1];
        attitude(&imu,0,rolls[i]);
        assert(Standup_Pose_Update(&standup_control,&imu)==STANDUP_POSE_NORMAL);
        assert(!Standup_Need(&imu,legs));
        standup_control.phase=STANDUP_SWING;
        standup_control.angle_cmd[0]=machine->lqr.leg_trim[0];
        standup_control.angle_cmd[1]=machine->lqr.leg_trim[1];
        assert(Standup_Ready(&standup_control,&imu,legs));
        assert(standup_control.ready_block==0);
        leg_l.output.virtual_leg_angle+=standup_param.angle_tol+.01f;
        assert(!Standup_Ready(&standup_control,&imu,legs));
        assert(standup_control.ready_block==16);
        leg_l.output.virtual_leg_angle=machine->lqr.leg_trim[0];
        /* Also no roll protection when recovery is disabled. */
        standup_control.recovery_enabled=0;
        Torque_Output_Clear(&torque);
        for(j=0;j<1000 && standup_control.phase!=STANDUP_DONE;j++)
        {
            route=Standup_Update(&standup_control,&imu,&leg_l,&leg_r,1,1,MACHINE_LQR_DT,&torque);
            assert(route!=STANDUP_ROUTE_STOP);
        }
        assert(standup_control.phase==STANDUP_DONE && route==STANDUP_ROUTE_BALANCE);
    }

    setup(0);attitude(&imu,LEG_PI,0);
    Torque_Output_Clear(&torque);
    assert(Standup_Update(&standup_control,&imu,&leg_l,&leg_r,1,1,MACHINE_LQR_DT,&torque)==STANDUP_ROUTE_PREPARE);
    assert(standup_control.pose==STANDUP_POSE_INVERTED && standup_control.upright<0);
    assert(standup_control.phase==STANDUP_SETTLE);
    for(j=0;j<1000 && standup_control.phase!=STANDUP_FLIP;j++)
    {
        assert(Standup_Update(&standup_control,&imu,&leg_l,&leg_r,1,1,MACHINE_LQR_DT,&torque)==STANDUP_ROUTE_PREPARE);
    }
    assert(standup_control.phase==STANDUP_FLIP);
    attitude(&imu,.5f,1.3f);
    Standup_Pose_Update(&standup_control,&imu);
    assert(Standup_Recovery_Ready(&standup_control,&imu,standup_param.recovery_ready_pitch));
    for(j=0;j<1000 && standup_control.phase==STANDUP_FLIP;j++)
    {
        route=Standup_Update(&standup_control,&imu,&leg_l,&leg_r,1,1,MACHINE_LQR_DT,&torque);
        assert(route!=STANDUP_ROUTE_STOP);
    }
    assert(standup_control.recovered && !Standup_Recovering(&standup_control));

    attitude(&imu,standup_param.pitch_max+.1f,0);
    assert(Standup_Pose_Update(&standup_control,&imu)==STANDUP_POSE_INVERTED);
    assert(standup_control.upright>0); /* Original large-pitch entry remains. */
    attitude(&imu,0,0);memset(imu.quat,0,sizeof(imu.quat));
    assert(Standup_Pose_Update(&standup_control,&imu)==STANDUP_POSE_INVALID);
    attitude(&imu,0,1.3f);Torque_Output_Clear(&torque);
    assert(Standup_Update(&standup_control,&imu,&leg_l,&leg_r,0,1,MACHINE_LQR_DT,&torque)==STANDUP_ROUTE_STOP);
    assert(!torque.valid && standup_control.fault==STANDUP_GATED);
    Standup_Reset(&standup_control);
    leg_l.output.valid=0;
    assert(Standup_Update(&standup_control,&imu,&leg_l,&leg_r,1,1,MACHINE_LQR_DT,&torque)==STANDUP_ROUTE_STOP);
    assert(!torque.valid && standup_control.fault==STANDUP_BAD_INPUT);

    setup(0);lqr_state.roll=.2f;
    lqr_state.leg_len_tgt[0]=leg_l.output.virtual_leg_length;
    lqr_state.leg_len_tgt[1]=leg_r.output.virtual_leg_length;
    assert(Leg_Balance_Compute(&leg_balance,&lqr_state,&leg_l,&leg_r,MACHINE_LQR_DT,&torque));
    assert(leg_balance.roll.pos_out!=0 && leg_balance.F[0]<leg_balance.F[1]);
    return 0;
}
"""


class StandupPoseTest(unittest.TestCase):
    def test_roll_ignored_and_inverted_recovery_kept(self):
        test_leg_length_ramp.LegLengthRampTest().run_check(CHECK)
