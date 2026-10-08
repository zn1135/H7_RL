"""Simulate task preemption while communication publishes control inputs."""
import os,re,shutil,subprocess,tempfile,unittest
from pathlib import Path
from test_gas_spring_integration import ROOT, production_function, read, without_includes

PREFIX=r'''
#include <assert.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "leg_solver.h"
#define DM_MOTOR_NUM 4
#define DJI_MOTOR_NUM 2
typedef struct { uint8_t online,s1,s2; } dr16_t;
'''

FIXTURE=r'''
static motor_state_t motor_state;
static leg_state_t leg_l,leg_r;
static leg_map_t leg_map_l,leg_map_r;
static robot_state_t robot_state;
static input_command_t input_command;
static rc_command_t rc_command;
typedef struct { float pos_rad,pos_zero_rad,vel_rad_s,trq_nm;uint32_t last_rx_tick;uint64_t parsed_rx_ns; } dm_motor_feedback_t;
typedef struct { float angle_rad,angle_total_rad,vel_rad_s;int16_t current_raw;uint32_t last_rx_tick; } dji_motor_feedback_t;
static dm_motor_feedback_t dm_motor_feedback[4];
static dji_motor_feedback_t dji_motor_feedback[2];
static dr16_t remote;
static unsigned critical,suspended,observations,publish_count;
static uint8_t observe_enabled;
static uint32_t now_tick;
static void observe(void)
{
    unsigned i;float base;
    if(!observe_enabled || critical || suspended) { return; }
    assert(motor_state.timestamp_ms==100 || motor_state.timestamp_ms==200);
    base=motor_state.timestamp_ms*.001f;
    for(i=0;i<4;i++) { assert(fabsf(motor_state.dm.pos_zero_rad[i]-(base+.1f*i))<1e-6f); }
    for(i=0;i<2;i++) { assert(fabsf(motor_state.dji.vel_rad_s[i]-(base+.01f*i))<1e-6f); }
    assert(fabsf(leg_l.output.virtual_leg_length-(.15f+.0001f*motor_state.timestamp_ms))<1e-6f);
    assert(fabsf(leg_r.output.virtual_leg_length-(.16f+.0001f*motor_state.timestamp_ms))<1e-6f);
    if(rc_command.vel==.1f)
    {
        assert(rc_command.yaw==.2f && rc_command.s1==1 && robot_state.rc_enable==0 && input_command.mode==1);
    }
    else
    {
        assert(rc_command.vel==-.4f && rc_command.yaw==-.5f && rc_command.s1==3);
        assert(robot_state.rc_enable==1 && input_command.mode==3);
    }
    observations++;
}
static void enter(void) { observe();critical++; }
static void leave(void) { assert(critical);critical--;publish_count++;observe(); }
#define taskENTER_CRITICAL() enter()
#define taskEXIT_CRITICAL() leave()
static void vTaskSuspendAll(void) { suspended++; }
static int xTaskResumeAll(void) { assert(suspended);suspended--;observe();return 0; }
static uint32_t HAL_GetTick(void) { return now_tick; }
static uint8_t Dm_Is_Online(unsigned i) { (void)i;return 1; }
static uint8_t Dji_Is_Online(unsigned i) { (void)i;return 1; }
uint8_t Leg_Solve(leg_state_t *leg)
{
    observe();memset(&leg->output,0,sizeof(leg->output));observe();
    leg->output.virtual_leg_length=.15f+.0001f*now_tick+leg->config.offset_phi0;
    leg->output.valid=leg->output.force_valid=1;
    return 1;
}
void Rc_Command_Update(rc_command_t *cmd,const dr16_t *rc)
{ cmd->vel=-.4f;observe();cmd->yaw=-.5f;cmd->online=rc->online;cmd->s1=rc->s1;cmd->s2=rc->s2; }
static uint8_t strategy_rc_enable(const rc_command_t *cmd) { return cmd->online && cmd->s1==3; }
static dr16_t DR16_Snapshot(void) { return remote; }
static void DR16_Process(void) { observe(); }
static void Dm_Parse(void) { observe(); }
static void Dji_Parse(void) { observe(); }
static void WS2812_RainbowBlink(void) { observe(); }
static void JointUsb_Process(void) { observe(); }
static void Robot_Fault_Update(void) { observe(); }
static void Robot_Fallen_Update(void) { observe(); }
static void Robot_Enable_Update(void) { observe(); }
static void JointUsb_Pump(void) { observe(); }
static uint8_t JointUsb_ModeLock(void) { return 0; }
static uint8_t JointUsb_StreamRequested(void) { return 0; }
static uint8_t Vofa_Transport_Update(uint8_t v) { (void)v;return 0; }
static void Vofa_Trace_Discard(void) {}
static uint8_t Vofa_Trace_Pump(void) { return 0; }
static void Robot_Control_Send_Vofa(void) { observe(); }
'''

CHECK=r'''
int main(void)
{
    unsigned i;
    memset(&motor_state,0,sizeof(motor_state));memset(&leg_l,0,sizeof(leg_l));memset(&leg_r,0,sizeof(leg_r));
    motor_state.timestamp_ms=100;
    for(i=0;i<4;i++)
    {
        motor_state.dm.pos_zero_rad[i]=.1f+.1f*i;
        dm_motor_feedback[i].pos_zero_rad=.2f+.1f*i;
        dm_motor_feedback[i].vel_rad_s=.3f+.1f*i;
    }
    for(i=0;i<2;i++) { motor_state.dji.vel_rad_s[i]=.1f+.01f*i;dji_motor_feedback[i].vel_rad_s=.2f+.01f*i; }
    leg_l.output.virtual_leg_length=.16f;leg_r.output.virtual_leg_length=.17f;
    leg_l.output.valid=leg_r.output.valid=1;leg_r.config.offset_phi0=.01f;
    leg_map_l.configured=leg_map_r.configured=1;
    leg_map_l.dm_front=0;leg_map_l.dm_rear=1;leg_map_r.dm_front=2;leg_map_r.dm_rear=3;
    rc_command.vel=.1f;rc_command.yaw=.2f;rc_command.online=1;rc_command.s1=1;input_command.mode=1;
    remote.online=1;remote.s1=remote.s2=3;now_tick=200;observe_enabled=1;
    comm_task_body();
    assert(observations>8 && critical==0 && suspended==0);
    assert(motor_state.timestamp_ms==200 && rc_command.vel==-.4f);
    assert(leg_l.output.valid && leg_r.output.valid);
    assert(fabsf(leg_l.input.hip_f-(.2f+LEG_PI))<1e-6f);
    assert(fabsf(leg_r.input.hip_f-(.4f+LEG_PI))<1e-6f);
    assert(publish_count>=2);
    observe_enabled=0;
    leg_map_l.configured=0;comm_task_body();
    assert(!leg_l.output.valid && !leg_l.output.force_valid && leg_r.output.valid);
    leg_map_l.configured=1;leg_map_l.dm_front=-1;comm_task_body();
    assert(!leg_l.output.valid && leg_r.output.valid);
    leg_map_l.dm_front=0;leg_map_r.dm_rear=DM_MOTOR_NUM;comm_task_body();
    assert(leg_l.output.valid && !leg_r.output.valid && !leg_r.output.force_valid);
    return 0;
}
'''

class ControlPublicationTest(unittest.TestCase):
    def test_preemption_observes_complete_inputs(self):
        compiler=os.environ.get('CC') or shutil.which('gcc')
        if compiler is None:self.skipTest('set CC to host gcc')
        header=read('imcalib/task/inc/robot_control.h')
        types=''
        for name in ('dm_motor_state_t','dji_motor_state_t','motor_state_t','robot_state_t','input_command_t','leg_map_t'):
            types+='\n'+re.search(r'typedef struct\s*\{[^}]*\}\s*'+name+r'\s*;',header).group()
        types+='\n'+without_includes(read('imcalib/user-lib/rc_command.h'))
        source=read('imcalib/task/task_comm.c')
        text=PREFIX+types+FIXTURE
        for name in ('Motor_State_Update','Leg_State_Update','Remote_Control_Update','comm_task_body'):
            text+='\n'+production_function(source,name)
        text+='\n'+CHECK
        with tempfile.TemporaryDirectory(prefix='control-publication-') as tmp:
            folder=Path(tmp);c=folder/'publication.c';c.write_text(text,encoding='utf-8');exe=folder/'publication.exe'
            r=subprocess.run([compiler,'-std=c99','-Wall','-Wextra','-Werror','-Wno-unused-function',
                '-I',str(ROOT/'imcalib/Algorithm'),str(c),'-lm','-o',str(exe)],capture_output=True,text=True)
            self.assertEqual(r.returncode,0,r.stdout+r.stderr)
            r=subprocess.run([str(exe)],capture_output=True,text=True)
            self.assertEqual(r.returncode,0,r.stdout+r.stderr)

if __name__=='__main__':unittest.main()
