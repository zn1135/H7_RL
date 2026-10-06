"""Real standalone recovery and unchanged balance chain with scripted feedback."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest
from test_gas_spring_integration import ARM_MATH_SHIM, production_enum, production_function, read, without_includes, ROOT
from test_lqr_unified import PREFIX

FIXTURE = r"""

static machine_cfg_t fixture_machine;
const machine_cfg_t *const machine = &fixture_machine;
uint8_t Machine_Id(void) { return MACHINE_DEFAULT; }
static leg_state_t leg_l, leg_r;
static imu_state_t imu_state;
static lqr_state_t lqr_state;
static leg_balance_t leg_balance;
static rc_command_t rc_command;
static struct { uint8_t rc_enable, motor_enabled, fallen; } robot_state;
static struct { struct { float vel_rad_s[2]; } dji; } motor_state;
static struct { uint8_t rl_ready; float a[6]; } action_state;
static struct { struct { unsigned selected_model; } policy;
    rl_torque_param_t torque_param[1]; rl_torque_state_t torque_state; } rl_control;
static volatile ctrl_strategy_t ctrl_strategy;
static uint32_t ctrl_fault;
static uint8_t torque_output_enabled, gas_spring_only_enabled;
static uint8_t output_debug_dm_sent, output_debug_dji_sent;
static float sent_dm[4], sent_wheel[2];
static unsigned disable_count;
static uint64_t now_ns;
static standup_param_t fixture_defaults;
static uint8_t fixture_defaults_ready;
static int htim6;
#define FAULT_NONE 0u
#define DR16_SW_UP 1u
#define DR16_SW_DOWN 2u
#define DR16_SW_MID 3u
#define HAL_OK 0
#define CONTROL_TIME_VOFA_ENABLE 0
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
#define Robot_Control_Vofa_Diag_Update(request) ((void)(request))
static int HAL_TIM_Base_Start_IT(int *t) { (void)t; return HAL_OK; }
static void Error_Handler(void) { assert(0); }
static uint64_t Mono_Ns_Get(void) { return now_ns; }
static uint8_t JointUsb_ModeLock(void) { return 0; }
static uint8_t JointUsb_PhysicalPermit(void) { return 1; }
static uint8_t JointUsb_EnableAllowed(void) { return 0; }
static uint8_t JointUsb_StreamRequested(void) { return 0; }
static void JointUsb_Compute(torque_output_t *t) { (void)t; }
static void JointUsb_ActuationTick(const torque_output_t *t, uint64_t n, uint8_t ok)
{ (void)t; (void)n; (void)ok; }
static int Dm_All_Enable(void) { return HAL_OK; }
static int Dm_All_Disable(void) { disable_count++; return HAL_OK; }
static void Dm_Enable_Watchdog(void) {}
static void Dm_Disable_Watchdog(void) {}
static int Dm_Send_Zero(void) { memset(sent_dm,0,sizeof(sent_dm)); return HAL_OK; }
static int Dji_All_Stop(void) { memset(sent_wheel,0,sizeof(sent_wheel)); return HAL_OK; }
static int Dm_Send_Torque(const float *value) { memcpy(sent_dm,value,sizeof(sent_dm)); return HAL_OK; }
static int Dji_Send_Wheel_Torque(float l,float r) { sent_wheel[0]=l; sent_wheel[1]=r; return HAL_OK; }
uint8_t RL_Torque_Compute(const leg_state_t *l,const leg_state_t *r,
    const rl_torque_param_t *p,const float v[2],const float a[RL_ACTION_SIZE],rl_torque_state_t *s,torque_output_t *t)
{ (void)l;(void)r;(void)p;(void)v;(void)a;(void)s;(void)t; return 0; }

static void zero(void)
{
    unsigned i;
    for(i=0;i<4;i++) { assert(sent_dm[i]==0); }
    assert(sent_wheel[0]==0 && sent_wheel[1]==0);
}
static void step(void)
{
    now_ns+=1000000;
    robot_state.rc_enable=strategy_rc_enable(&rc_command);
    Robot_Fallen_Update();
    Robot_Enable_Update();
    output_task_body();
}
static void setup(float angle)
{
    if(!fixture_defaults_ready) { fixture_defaults=standup_param;fixture_defaults_ready=1; }
    standup_param=fixture_defaults;
    fixture_machine=machine_table[MACHINE_DEFAULT];
    memset(&leg_l,0,sizeof(leg_l)); memset(&leg_r,0,sizeof(leg_r));
    memset(&imu_state,0,sizeof(imu_state)); memset(&robot_state,0,sizeof(robot_state));
    memset(&rc_command,0,sizeof(rc_command));memset(sent_dm,0,sizeof(sent_dm));memset(sent_wheel,0,sizeof(sent_wheel));
    ctrl_fault=gas_spring_only_enabled=0;torque_output_enabled=1;
    lqr_running=rl_engaged=rl_wait_ticks=0;
    rc_command.online=1;rc_command.s1=rc_command.s2=DR16_SW_MID;
    imu_state.online=1;imu_state.quat[0]=1;imu_state.acc_g[2]=1;
    imu_state.pitch_world_valid=1;
    leg_l.output.valid=leg_l.output.force_valid=1;
    leg_l.output.virtual_leg_length=0.30f;leg_l.output.virtual_leg_angle=angle;
    leg_l.output.force_map[0][0]=0.05f;leg_l.output.force_map[0][1]=0.5f;
    leg_l.output.force_map[1][0]=-0.05f;leg_l.output.force_map[1][1]=0.5f;
    leg_r=leg_l;
    LQR_Init(&lqr_state);Leg_Balance_Init(&leg_balance);Standup_Init(&standup_control);
    assert(standup_control.enabled);
    assert(LQR_Gain_Compatible());
}
static inline void follow(void)
{
    leg_l.output.virtual_leg_length=standup_control.length_cmd[0];
    leg_r.output.virtual_leg_length=standup_control.length_cmd[1];
    if(standup_control.phase==STANDUP_REAR)
    {
        leg_l.output.virtual_leg_angle=Standup_Wrap(leg_l.output.virtual_leg_angle
            +clampf(standup_control.angle_cmd[0]-standup_control.rear_position[0],-.01f,.01f));
        leg_r.output.virtual_leg_angle=Standup_Wrap(leg_r.output.virtual_leg_angle
            +clampf(standup_control.angle_cmd[1]-standup_control.rear_position[1],-.01f,.01f));
    }
    else if(standup_control.phase!=STANDUP_RETRACT)
    {
        leg_l.output.virtual_leg_angle=Standup_Wrap(standup_control.angle_cmd[0]);
        leg_r.output.virtual_leg_angle=Standup_Wrap(standup_control.angle_cmd[1]);
    }
}
"""

CHECKS = {
    'ready_tolerances': r"""
int main(void)
{
    const leg_state_t *legs[2]={&leg_l,&leg_r};
    setup(machine_table[MACHINE_DEFAULT].lqr.leg_trim[0]);
    standup_control.phase=STANDUP_SWING;standup_control.support=1;
    standup_control.length_cmd[0]=standup_param.retract_len;
    standup_control.length_cmd[1]=standup_param.retract_len;
    standup_control.angle_cmd[0]=machine->lqr.leg_trim[0];
    standup_control.angle_cmd[1]=machine->lqr.leg_trim[1];
    leg_l.output.virtual_leg_length=standup_param.retract_len+.039f;
    leg_r.output.virtual_leg_length=standup_param.retract_len+.039f;
    leg_l.output.virtual_leg_angle=machine->lqr.leg_trim[0]+standup_param.angle_tol-.001f;
    leg_r.output.virtual_leg_angle=machine->lqr.leg_trim[1]+standup_param.angle_tol-.001f;
    assert(Standup_Ready(&standup_control,legs));
    leg_r.output.virtual_leg_length=standup_param.retract_len+.041f;
    assert(!Standup_Ready(&standup_control,legs));
    leg_r.output.virtual_leg_length=standup_param.retract_len+.039f;
    leg_r.output.virtual_leg_angle=machine->lqr.leg_trim[1]+standup_param.angle_tol+.001f;
    assert(!Standup_Ready(&standup_control,legs));
    leg_r.output.virtual_leg_angle=machine->lqr.leg_trim[1]+standup_param.angle_tol-.001f;
    imu_state.pitch_world=.4f;imu_state.gyro_rad_s[1]=5.0f;
    leg_l.output.d_virtual_leg_length=leg_r.output.d_virtual_leg_length=2.0f;
    leg_l.output.d_virtual_leg_angle=leg_r.output.d_virtual_leg_angle=5.0f;
    standup_control.support=0;
    assert(Standup_Ready(&standup_control,legs));
    rc_command.s2=DR16_SW_UP;step();zero();
    return 0;
}
""",
    'public_flags_only': r"""
int main(void)
{
    const leg_state_t *legs[2]={&leg_l,&leg_r};
    setup(-1.2f);
    assert(Standup_Input_Valid(&imu_state,legs));
    fixture_machine.lqr.leg_len[0].kp=NAN;
    assert(Standup_Input_Valid(&imu_state,legs)); /* PID tuning is not an input flag. */
    imu_state.online=0;assert(!Standup_Input_Valid(&imu_state,legs));imu_state.online=1;
    imu_state.pitch_world_valid=0;assert(!Standup_Input_Valid(&imu_state,legs));imu_state.pitch_world_valid=1;
    leg_l.output.valid=0;assert(!Standup_Input_Valid(&imu_state,legs));leg_l.output.valid=1;
    leg_r.output.force_valid=0;assert(!Standup_Input_Valid(&imu_state,legs));leg_r.output.force_valid=1;
    assert(!Standup_Input_Valid(NULL,legs));
    rc_command.s2=DR16_SW_UP;step();zero();
    return 0;
}
""",
    'cascade_history_and_ready_hold': r"""
int main(void)
{
    unsigned i;float previous;
    setup(-1.2f);
    fixture_machine.lqr.leg_len[0].ki=10;
    step();
    assert(standup_control.length_pid[0].i==0);
    assert(standup_control.length_pid[0].dout==0 && standup_control.length_pid[1].dout==0);
    previous=standup_control.length_pid[0].err[LAST];step();
    assert(fabsf(standup_control.length_pid[0].dout
        -machine->lqr.leg_len[0].kd*(standup_control.length_pid[0].err[NOW]-previous))<1e-5f);
    setup(-1.2f);
    leg_l.output.virtual_leg_length=leg_r.output.virtual_leg_length=0.15f;
    standup_param.angle_pos_kd=50;standup_param.angle_speed_kd=10;
    step();assert(standup_control.phase==STANDUP_SWING);
    assert(standup_control.angle_pos_pid[0].dout==0 && standup_control.angle_speed_pid[0].dout==0);
    setup(-1.2f);
    leg_l.output.virtual_leg_length=leg_r.output.virtual_leg_length=0.15f;
    step();assert(standup_control.phase==STANDUP_SWING);
    standup_control.angle_cmd[0]=machine->lqr.leg_trim[0]+0.8f;
    leg_l.output.d_virtual_leg_angle=standup_param.angle_speed_max+0.5f;step();
    assert(standup_control.angle_speed_cmd[0]==standup_param.angle_speed_max);
    assert(standup_control.tp[0]<0); /* Brake above commanded speed. */
    setup(-1.2f);step();
    for(i=0;i<4000;i++)
    {
        follow();step();
        if(standup_control.phase==STANDUP_SWING && standup_control.stable>=0.02f) { break; }
    }
    assert(i<4000 && standup_control.phase==STANDUP_SWING);
    leg_l.output.virtual_leg_angle=machine->lqr.leg_trim[0]+standup_param.angle_tol+.01f;step();
    assert(standup_control.stable==0 && standup_control.phase==STANDUP_SWING && !lqr_running);
    for(i=0;i<20;i++) { follow();step(); }
    assert(standup_control.phase==STANDUP_SWING);
    for(i=0;i<500 && standup_control.phase!=STANDUP_DONE;i++) { follow();step(); }
    assert(standup_control.phase==STANDUP_DONE);
    rc_command.s2=DR16_SW_UP;step();zero();
    return 0;
}
""",
    'trigger_thresholds': r"""
int main(void)
{
    const leg_state_t *legs[2]={&leg_l,&leg_r};unsigned side;int sign;
    setup(machine_table[MACHINE_DEFAULT].lqr.leg_trim[0]);
    leg_r.output.virtual_leg_angle=machine->lqr.leg_trim[1];
    assert(fabsf(standup_param.trigger_angle-50.0f*LEG_PI/180.0f)<1e-6f);
    assert(fabsf(standup_param.trigger_pitch-40.0f*LEG_PI/180.0f)<1e-6f);
    for(sign=-1;sign<=1;sign+=2)
    {
        for(side=0;side<2;side++)
        {
            (side==0 ? &leg_l : &leg_r)->output.virtual_leg_angle=
                machine->lqr.leg_trim[side]+sign*49.9f*LEG_PI/180.0f;
            assert(!Standup_Need(&imu_state,legs));
            (side==0 ? &leg_l : &leg_r)->output.virtual_leg_angle=
                machine->lqr.leg_trim[side]+sign*50.1f*LEG_PI/180.0f;
            assert(Standup_Need(&imu_state,legs));
            (side==0 ? &leg_l : &leg_r)->output.virtual_leg_angle=machine->lqr.leg_trim[side];
        }
        imu_state.pitch_world=sign*39.9f*LEG_PI/180.0f;
        assert(!Standup_Need(&imu_state,legs));
        imu_state.pitch_world=sign*40.1f*LEG_PI/180.0f;
        assert(Standup_Need(&imu_state,legs));
        imu_state.pitch_world=0;
    }
    rc_command.s2=DR16_SW_UP;step();zero();
    return 0;
}
""",
    'world_judge_and_control': r"""
int main(void)
{
    const float angles[]={0,1.0f,1.4f,1.401f,1.2f,1.0f,0.999f,LEG_PI,-LEG_PI};
    const unsigned expected[]={0,0,0,1,1,1,0,1,1};
    const leg_state_t *legs[2]={&leg_l,&leg_r};
    standup_ctx_t first,second;unsigned i;
    setup(-1.2f);
    for(i=0;i<sizeof(expected)/sizeof(expected[0]);i++)
    {
        imu_state.pitch_world=angles[i];Robot_Fallen_Update();assert(robot_state.fallen==expected[i]);
    }
    imu_state.pitch_world=0;imu_state.pitch_world_valid=0;Robot_Fallen_Update();assert(robot_state.fallen);
    imu_state.pitch_world_valid=1;imu_state.pitch_world=NAN;Robot_Fallen_Update();assert(robot_state.fallen);
    imu_state.pitch_world=0;imu_state.online=0;Robot_Fallen_Update();assert(robot_state.fallen);
    imu_state.online=1;Robot_Fallen_Update();assert(!robot_state.fallen);
    imu_state.pitch_world=LEG_PI;step();zero();assert(robot_state.fallen && !robot_state.motor_enabled);
    assert(standup_control.phase==STANDUP_IDLE && standup_control.need && !lqr_running);
    setup(-1.2f);imu_state.euler_rad[1]=1.0f;
    for(i=0;i<2;i++)
    {
        standup_control.length_cmd[i]=0.3f;standup_control.angle_cmd[i]=machine->lqr.leg_trim[i];
    }
    leg_l.output.virtual_leg_angle=machine->lqr.leg_trim[0];
    leg_r.output.virtual_leg_angle=machine->lqr.leg_trim[1];
    assert(!Standup_Need(&imu_state,legs));
    imu_state.pitch_world=0.6f;assert(!Standup_Need(&imu_state,legs));
    imu_state.pitch_world=0.75f;assert(Standup_Need(&imu_state,legs));
    standup_control.phase=STANDUP_SWING;standup_control.support=1;
    for(i=0;i<2;i++) { standup_control.length_cmd[i]=machine->lqr.leg_len_init[i]; }
    leg_l.output.virtual_leg_length=standup_control.length_cmd[0];
    leg_r.output.virtual_leg_length=standup_control.length_cmd[1];
    assert(Standup_Ready(&standup_control,legs));
    imu_state.pitch_world=0;assert(Standup_Ready(&standup_control,legs));
    standup_control.phase=STANDUP_SWING;
    first=standup_control;second=standup_control;
    assert(Standup_PID_Calculate(&first,legs,MACHINE_LQR_DT) && Standup_Torque_Output(&first,legs));imu_state.pitch_world=0.6f;imu_state.euler_rad[1]=-.5f;imu_state.gyro_rad_s[1]=5;
    assert(Standup_PID_Calculate(&second,legs,MACHINE_LQR_DT) && Standup_Torque_Output(&second,legs));
    assert(memcmp(first.force,second.force,sizeof(first.force))==0);
    assert(memcmp(first.tp,second.tp,sizeof(first.tp))==0);
    assert(memcmp(first.prepare.dm,second.prepare.dm,sizeof(first.prepare.dm))==0);
    imu_state.pitch_world_valid=0;assert(!Standup_Input_Valid(&imu_state,legs));
    return 0;
}
""",
    'sequence': r"""
int main(void)
{
    unsigned i,j,k,phases[7],previous;float alpha;
    const float angles[2]={-1.2f,1.2f};
    for(k=0;k<2;k++)
    {
        setup(angles[k]);memset(phases,0,sizeof(phases));rc_command.vel=0.2f;previous=STANDUP_IDLE;
        for(i=0;i<4200;i++)
        {
            step();assert(standup_control.phase!=STANDUP_FAILED);
            if(standup_control.phase!=previous)
            {
                assert((previous==STANDUP_IDLE && (standup_control.phase==STANDUP_RETRACT || standup_control.phase==STANDUP_REAR))
                    || (previous==STANDUP_REAR && standup_control.phase==STANDUP_RETRACT)
                    || (previous==STANDUP_RETRACT && standup_control.phase==STANDUP_SWING)
                    || (previous==STANDUP_SWING && standup_control.phase==STANDUP_DONE));
                previous=standup_control.phase;
            }
            phases[standup_control.phase]=1;
            if((standup_control.phase==STANDUP_RETRACT || standup_control.phase==STANDUP_SWING))
            {
                assert(standup_control.need && !(standup_control.phase==STANDUP_DONE) && !lqr_running);
                assert(standup_control.length_cmd[0]==standup_param.retract_len);
                assert(standup_control.length_cmd[1]==standup_param.retract_len);
                assert(sent_wheel[0]==0 && sent_wheel[1]==0);
                for(j=0;j<4;j++) { assert(fabsf(sent_dm[j])<=machine->dm_trq_clamp); }
                assert(standup_control.angle_speed_pid[0].MaxOutput==fminf(standup_param.tp_max,machine->dm_trq_clamp));
                assert(leg_balance.leg_len[0].err[LAST]==0);
                assert(standup_control.length_pid[0].p==machine->lqr.leg_len[0].kp);
                assert(standup_control.length_pid[0].d==machine->lqr.leg_len[0].kd);
                assert(standup_control.length_pid[0].i==0 && standup_control.length_pid[0].iout==0);
                if(standup_control.angle_history_ready)
                {
                    assert(standup_control.angle_pos_pid[0].i==0 && standup_control.angle_speed_pid[0].i==0);
                    assert(standup_control.angle_pos_pid[0].iout==0 && standup_control.angle_speed_pid[0].iout==0);
                }
                assert(fabsf(standup_control.tp[0])<=fminf(standup_param.tp_max,machine->dm_trq_clamp));
            }
            if(standup_control.phase==STANDUP_SWING)
            {
                assert(standup_control.angle_cmd[0]==Standup_Wrap(machine->lqr.leg_trim[0]));
                assert(standup_control.angle_cmd[1]==Standup_Wrap(machine->lqr.leg_trim[1]));
                assert(standup_control.tp[0]>=0);
            }
            if(standup_control.phase==STANDUP_DONE)
            {
                assert(!standup_control.need && !(standup_control.phase==STANDUP_RETRACT || standup_control.phase==STANDUP_SWING) && (standup_control.phase==STANDUP_DONE));
                alpha=standup_control.blend;
                assert(fabsf(sent_wheel[0])<=machine->dji_trq_clamp*alpha+1e-5f);
                if(alpha>=1) { break; }
            }
            follow();
        }
        assert(i<4200 && phases[1] && phases[2] && phases[3]);
        assert(lqr_running && lqr_state.target[LQR_X_DS]>0);
        rc_command.s2=DR16_SW_UP;step();zero();assert(standup_control.phase==STANDUP_IDLE);
        assert(standup_control.enabled);
    }
    return 0;
}
""",
    'direct_and_disabled': r"""
int main(void)
{
    unsigned i;
    setup(machine_table[MACHINE_DEFAULT].lqr.leg_trim[0]);standup_control.enabled=0;
    step();assert(lqr_running && standup_control.phase==STANDUP_IDLE);
    setup(machine_table[MACHINE_DEFAULT].lqr.leg_trim[0]);step();
    assert(standup_control.phase==STANDUP_REAR && standup_control.need && !lqr_running);
    setup(machine_table[MACHINE_DEFAULT].lqr.leg_trim[0]);standup_control.phase=STANDUP_DONE;standup_control.blend=1;
    step();assert(lqr_running && standup_control.phase==STANDUP_DONE);
    for(i=0;i<150;i++) { leg_l.output.virtual_leg_angle=1.2f;rc_command.vel=.2f;step(); }
    assert(standup_control.phase==STANDUP_DONE && lqr_running);
    rc_command.vel=0;
    for(i=0;i<110;i++) { step(); }
    assert(standup_control.phase==STANDUP_REAR && standup_control.need && !lqr_running);
    rc_command.s2=DR16_SW_UP;step();zero();
    return 0;
}
""",
    'rear_preposition': r"""
int main(void)
{
    const leg_state_t *legs[2]={&leg_l,&leg_r};unsigned i,crossed=0;float previous,old_angle;
    setup(.5f);leg_l.output.virtual_leg_length=leg_r.output.virtual_leg_length=.13f;step();
    assert(standup_control.phase==STANDUP_EXTEND && !lqr_running);
    assert(standup_control.length_cmd[0]==Standup_Extend_Length() && standup_control.angle_cmd[0]==.5f);
    assert(standup_control.length_pid[0].dout==0 && standup_control.support==0);
    assert(sent_wheel[0]==0 && sent_wheel[1]==0);
    leg_l.output.virtual_leg_length=Standup_Extend_Length();
    for(i=0;i<150;i++) { step(); }assert(standup_control.phase==STANDUP_EXTEND);
    leg_r.output.virtual_leg_length=Standup_Extend_Length();
    for(i=0;i<150 && standup_control.phase==STANDUP_EXTEND;i++) { step(); }
    assert(standup_control.phase==STANDUP_REAR && standup_control.tp[0]>0);
    assert(standup_control.tp[0]<1.0f);
    assert(fabsf(standup_control.angle_cmd[0]-(.5f+standup_param.rear_rate*MACHINE_LQR_DT))<1e-5f);
    assert(fabsf(standup_control.rear_goal[0]-(standup_param.rear_angle+LEG_2PI))<1e-5f);
    assert(standup_control.angle_pos_pid[0].angle_wrap==0);
    assert(standup_control.length_cmd[0]==Standup_Extend_Length());
    for(i=0;i<1800 && standup_control.phase==STANDUP_REAR;i++)
    {
        previous=standup_control.angle_cmd[0];old_angle=leg_l.output.virtual_leg_angle;
        follow();step();
        if(old_angle>3.0f && leg_l.output.virtual_leg_angle<-3.0f) { crossed=1; }
        if(standup_control.phase==STANDUP_REAR)
        {
            assert(standup_control.angle_cmd[0]>=previous);
            assert(standup_control.angle_cmd[0]-previous<=standup_param.rear_rate*MACHINE_LQR_DT+1e-6f);
        }
    }
    assert(crossed);
    assert(standup_control.phase==STANDUP_RETRACT && standup_control.angle_pos_pid[0].angle_wrap==1);
    assert(standup_control.length_cmd[0]==standup_param.retract_len && standup_control.length_pid[0].dout==0);
    setup(.5f);step();
    leg_l.output.virtual_leg_angle=leg_r.output.virtual_leg_angle=standup_param.rear_angle;step();
    assert(standup_control.phase==STANDUP_REAR && !Standup_Ready(&standup_control,legs));
    setup(.5f);imu_state.pitch_world=LEG_PI;Standup_Enter(&standup_control,STANDUP_REAR);
    Standup_Target_Update(&standup_control,&imu_state,legs,MACHINE_LQR_DT);
    assert(standup_control.rear_goal[0]<.5f); /* Planning reverses relative to gravity. */
    setup(standup_param.rear_angle);step();assert(standup_control.phase==STANDUP_RETRACT);
    setup(.5f);step();for(i=0;i<4100;i++) { step(); }
    zero();assert(standup_control.phase==STANDUP_FAILED && standup_control.fault==STANDUP_TIMEOUT);
    assert(standup_control.elapsed>standup_param.rear_timeout);
    setup(.5f);leg_l.output.virtual_leg_length=leg_r.output.virtual_leg_length=.13f;step();
    for(i=0;i<4100;i++) { step(); }
    zero();assert(standup_control.fault==STANDUP_TIMEOUT && standup_control.elapsed>standup_param.extend_timeout);
    setup(.5f);step();ctrl_fault=4;step();zero();assert(standup_control.fault==STANDUP_GATED);
    return 0;
}
""",
    'faults_and_rearm': r"""
int main(void)
{
    unsigned i;
    float failure_time;
    setup(-1.2f);step();
    for(i=0;i<3100;i++) { step(); }
    zero();assert(standup_control.phase==STANDUP_FAILED && standup_control.fault==STANDUP_TIMEOUT);
    failure_time=standup_control.elapsed;
    assert(failure_time>standup_param.timeout[0] && failure_time<standup_param.timeout[0]+.01f);
    assert(!standup_control.prepare.valid);
    assert(robot_state.motor_enabled); /* No new communication gate. */
    leg_l.output.virtual_leg_angle=leg_r.output.virtual_leg_angle=machine->lqr.leg_trim[0];
    step();zero();assert(standup_control.phase==STANDUP_FAILED && !lqr_running);
    assert(standup_control.elapsed==failure_time && standup_control.fault==STANDUP_TIMEOUT);
    setup(-1.2f);step();assert(standup_control.phase==STANDUP_RETRACT);
    leg_l.output.virtual_leg_length=.19f;leg_r.output.virtual_leg_length=.1901f;step();
    assert(standup_control.phase==STANDUP_RETRACT);
    leg_r.output.virtual_leg_length=.19f;
    leg_l.output.d_virtual_leg_length=leg_r.output.d_virtual_leg_length=-1.0f;
    step();assert(standup_control.phase==STANDUP_SWING && standup_control.elapsed==0 && standup_control.stable==0);
    leg_l.output.d_virtual_leg_length=leg_r.output.d_virtual_leg_length=0;
    for(i=0;i<4100;i++) { step(); }
    zero();assert(standup_control.phase==STANDUP_FAILED && standup_control.fault==STANDUP_TIMEOUT);
    assert(standup_control.elapsed>standup_param.timeout[1] && standup_control.elapsed<standup_param.timeout[1]+.01f);
    leg_l.output.virtual_leg_angle=leg_r.output.virtual_leg_angle=machine->lqr.leg_trim[0];
    rc_command.s2=DR16_SW_UP;step();assert(standup_control.phase==STANDUP_IDLE);
    rc_command.s2=DR16_SW_MID;step();assert((standup_control.phase==STANDUP_DONE) && lqr_running);
    ctrl_fault=4;step();zero();assert(standup_control.fault==STANDUP_GATED && !robot_state.motor_enabled);
    ctrl_fault=0;step();zero();assert(!lqr_running);
    setup(-1.2f);step();rc_command.online=0;step();zero();assert(standup_control.phase==STANDUP_FAILED);
    rc_command.online=1;step();zero();assert(!lqr_running);
    setup(-1.2f);step();torque_output_enabled=0;step();zero();assert(standup_control.fault==STANDUP_GATED);
    torque_output_enabled=1;step();zero();
    setup(-1.2f);leg_l.output.force_valid=0;step();zero();assert(standup_control.fault==STANDUP_BAD_INPUT);
    setup(-1.2f);standup_control.phase=99;step();zero();assert(standup_control.fault==STANDUP_BAD_CONFIG);
    setup(-1.2f);imu_state.pitch_world=1.0f;step();zero();assert(standup_control.fault==STANDUP_BAD_POSE);
    setup(-1.2f);step();imu_state.euler_rad[0]=0.6f;step();zero();assert(standup_control.fault==STANDUP_BAD_POSE);
    setup(-1.2f);imu_state.pitch_world=2.0f;step();zero();assert(!lqr_running && !(standup_control.phase==STANDUP_RETRACT || standup_control.phase==STANDUP_SWING));
    setup(machine_table[MACHINE_DEFAULT].lqr.leg_trim[0]);step();lqr_state.K[0][0]=NAN;step();zero();
    assert(standup_control.fault==STANDUP_BAD_INPUT);
    return 0;
}
""",
    'swing_support_and_wrap': r"""
int main(void)
{
    unsigned i;float previous,delta;
    setup(-1.2f);step();
    for(i=0;i<1600 && standup_control.phase==STANDUP_RETRACT;i++) { follow();step(); }
    assert(standup_control.phase==STANDUP_SWING);
    for(i=0;i<4100;i++) { step(); }
    zero();assert(standup_control.fault==STANDUP_TIMEOUT);
    setup(-1.2f);step();
    for(i=0;i<1600 && standup_control.phase==STANDUP_RETRACT;i++) { follow();step(); }
    assert(standup_control.phase==STANDUP_SWING);
    fixture_machine.lqr.leg_trim[0]=fixture_machine.lqr.leg_trim[1]=3.13f;
    standup_control.angle_cmd[0]=standup_control.angle_cmd[1]=-3.13f;
    for(i=0;i<50;i++)
    {
        previous=standup_control.angle_cmd[0];step();delta=Standup_Wrap(standup_control.angle_cmd[0]-previous);
        assert(delta<=0 && fabsf(delta)<0.03f);
        assert(standup_control.angle_cmd[0]==Standup_Wrap(fixture_machine.lqr.leg_trim[0]));
        assert(fabsf(standup_control.angle_cmd[0])<=LEG_PI && fabsf(standup_control.angle_speed_cmd[0])<=standup_param.angle_speed_max);
    }
    assert(standup_control.angle_cmd[0]>3.12f);
    return 0;
}
""",
}

class StandupTest(unittest.TestCase):
    def test_real_controller_and_selector(self):
        compiler=os.environ.get('CC') or shutil.which('gcc')
        if compiler is None:self.skipTest('set CC to host gcc')
        env=os.environ.copy();env['PATH']=str(Path(compiler).parent)+os.pathsep+env.get('PATH','')
        headers=[PREFIX.replace('#define DM_MOTOR_NUM 4\n',''),
                 '#define RL_ACTION_SIZE 6\ntypedef int rl_model_t;',
                 production_enum(read('imcalib/user-lib/dm.h'),'dm_motor_idx_t')]
        for path in ('imcalib/user-lib/rc_command.h','imcalib/Algorithm/torque_output.h','imcalib/Algorithm/rl_torque.h',
                     'imcalib/Algorithm/lqr_balance.h','imcalib/Algorithm/leg_balance.h','imcalib/Algorithm/gas_spring.h',
                     'imcalib/Algorithm/standup.h'):
            headers.append(without_includes(read(path)))
        headers.append(production_enum(read('imcalib/task/inc/robot_control.h'),'ctrl_strategy_t'))
        config=without_includes(read('imcalib/user-lib/machine_config.c')).split('const machine_cfg_t *const machine')[0]
        split=FIXTURE.index('static void zero(')
        content='\n'.join(headers)+'\n'+config+'\n'+FIXTURE[:split]
        for path in ('imcalib/Algorithm/lqr_balance.c','imcalib/Algorithm/leg_balance.c','imcalib/Algorithm/gas_spring.c',
                     'imcalib/Algorithm/standup.c','imcalib/task/task_actuation.c'):
            content+='\n'+without_includes(read(path))
        content+='\n'+production_function(read('imcalib/task/task_comm.c'),'Robot_Fallen_Update')
        content+='\n'+production_function(read('imcalib/task/task_comm.c'),'Robot_Enable_Update')+'\n'+FIXTURE[split:]
        with tempfile.TemporaryDirectory(prefix='standup-host-') as temp:
            folder=Path(temp);(folder/'arm_math.h').write_text(ARM_MATH_SHIM,encoding='utf-8')
            for name,checks in CHECKS.items():
                source=folder/(name+'.c');source.write_text(content+checks,encoding='utf-8')
                for machine in (0,1):
                    with self.subTest(case=name,machine=machine):
                        exe=folder/(name+str(machine)+'.exe')
                        dependencies=['imcalib/Algorithm/lqr_gain_table.c','imcalib/Algorithm/lqr_gain_big.c',
                            'imcalib/Algorithm/lqr_gain_small.c','imcalib/Algorithm/leg_solver.c','imcalib/user-lib/pid.c',
                            'imcalib/user-lib/simple-function.c','imcalib/user-lib/kalman.c']
                        r=subprocess.run([compiler,'-std=c99','-Wall','-Wextra','-Werror','-DLEG_TRIG_LIBM=1',
                            f'-DMACHINE_DEFAULT={machine}','-I',str(folder),'-I',str(ROOT/'imcalib/Algorithm'),
                            '-I',str(ROOT/'imcalib/user-lib'),str(source),*[str(ROOT/p) for p in dependencies],'-lm','-o',str(exe)],
                            capture_output=True,text=True,encoding='utf-8',errors='replace',env=env)
                        self.assertEqual(r.returncode,0,r.stdout+r.stderr)
                        r=subprocess.run([str(exe)],capture_output=True,text=True,encoding='utf-8',errors='replace',env=env)
                        self.assertEqual(r.returncode,0,r.stdout+r.stderr)

    def test_module_boundary(self):
        source=read('imcalib/Algorithm/standup.c')+read('imcalib/Algorithm/standup.h')
        for forbidden in ('LQR_', 'lqr_state', 'leg_balance', 'robot_state', 'ctrl_fault', 'Dm_Send', 'Dm_All_', 'Dji_Send'):
            self.assertNotIn(forbidden,source)
