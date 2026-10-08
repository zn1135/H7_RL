"""Check the actual 39-channel standup packing and JustFloat DMA buffer."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

from test_gas_spring_integration import ROOT, production_function, read, without_includes, ARM_MATH_SHIM
from test_lqr_unified import PREFIX

FIXTURE = r"""
#include <assert.h>
#include <stdint.h>
#include <math.h>
#include <string.h>
#include "machine_config.h"
#define VOFA_MAX_CH 39u
#define DMA_CACHE_LINE_SIZE 32u
#define DM_MOTOR_LEG_F_LFT 0u
#define DM_MOTOR_LEG_B_LFT 1u
#define DM_MOTOR_LEG_F_RGT 2u
#define DM_MOTOR_LEG_B_RGT 3u
standup_ctx_t standup_control;
standup_param_t standup_param;
static leg_state_t leg_l, leg_r;
static imu_state_t imu_state;
static lqr_state_t lqr_state;
static leg_balance_t leg_balance;
static struct { uint8_t motor_enabled, fallen; } robot_state;
static struct { struct { uint8_t online[4]; } dm; struct { uint8_t online[2]; } dji;
                uint32_t timestamp_ms; } motor_state;
static uint32_t ctrl_fault;
static uint8_t output_debug_dm_sent, output_debug_dji_sent;
static float rl_output_dm_cmd_nm[4], rl_output_wheel_cmd_nm[2];
static machine_cfg_t fixture_machine;
const machine_cfg_t *const machine=&fixture_machine;
static uint8_t lqr_engaged, dma_ready=1;
static uint8_t *dma_buffer;
static uint16_t dma_length;
static unsigned send_count;
static const uint8_t tail[4]={0,0,0x80,0x7f};
static uint8_t DR16_Online(void) { return 1; }
static uint8_t Dm_Is_Enabled(uint8_t i) { return motor_state.dm.online[i]; }
static uint8_t output_task_lqr_engaged(void) { return lqr_engaged; }
static uint8_t output_task_rl_engaged(void) { return 0; }
static uint8_t Vofa_Transport_Ready(void) { return dma_ready; }
static uint8_t Vofa_Transport_Send(uint8_t *data,uint16_t len)
{ assert(dma_ready);dma_buffer=data;dma_length=len;send_count++;dma_ready=0;return 1; }
"""

CHECK = r"""
static void frame(float out[39])
{
    dma_ready=1;Robot_Control_Send_Vofa();
    assert(dma_length==160 && memcmp(dma_buffer+156,tail,4)==0);
    memcpy(out,dma_buffer,156);
}
int main(void)
{
    float out[39];uint8_t prior[160];unsigned i,count;
    imu_state.online=imu_state.pitch_world_valid=robot_state.motor_enabled=1;
    leg_l.output.valid=leg_r.output.valid=leg_l.output.force_valid=leg_r.output.force_valid=1;
    for(i=0;i<4;i++) { motor_state.dm.online[i]=1;rl_output_dm_cmd_nm[i]=10.0f+i; }
    motor_state.dji.online[0]=motor_state.dji.online[1]=1;
    motor_state.timestamp_ms=0x0100007bu;
    ctrl_fault=8;standup_control.phase=4;standup_control.fault=2;
    standup_control.elapsed=2.25f;standup_control.stable=.05f;
    standup_control.enabled=standup_control.need=standup_control.recovery_enabled=1;
    standup_control.recovered=standup_control.prepare.valid=1;
    lqr_state.valid=lqr_state.gain_valid=output_debug_dm_sent=output_debug_dji_sent=1;
    standup_control.ready_block=409;standup_control.retry=2;standup_control.pose=1;
    imu_state.pitch_world=-1.2f;imu_state.euler_rad[1]=-.4f;imu_state.euler_rad[0]=.1f;
    imu_state.gyro_rad_s[1]=-.3f;standup_control.upright=.4f;standup_control.support=.7f;
    leg_l.output.virtual_leg_length=.15f;leg_r.output.virtual_leg_length=.16f;
    standup_control.length_cmd[0]=.18f;standup_control.length_cmd[1]=.19f;
    leg_l.output.virtual_leg_angle=-1.1f;leg_r.output.virtual_leg_angle=-1.2f;
    standup_control.angle_cmd[0]=5;standup_control.angle_cmd[1]=5.1f;
    leg_l.output.d_virtual_leg_angle=2;leg_r.output.d_virtual_leg_angle=3;
    standup_control.force[0]=120;standup_control.force[1]=130;
    standup_control.tp[0]=30;standup_control.tp[1]=31;
    rl_output_wheel_cmd_nm[0]=-2;rl_output_wheel_cmd_nm[1]=2;
    standup_param.stable_time=.1f;frame(out);
    assert(out[0]==255 && out[1]==253 && out[2]==8 && out[3]==4 && out[4]==2);
    assert(out[5]==2250 && fabsf(out[6]-50)<1e-5f && out[7]==4095 && out[8]==409);
    assert(out[9]==2 && out[10]==1 && out[11]==-1.2f && out[12]==-.4f && out[13]==.1f);
    assert(out[14]==-.3f && out[15]==.4f && out[16]==.7f);
    assert(out[17]==.15f && out[18]==.16f && out[19]==.18f && out[20]==.19f);
    assert(out[21]==-1.1f && out[22]==-1.2f && out[23]==5 && out[24]==5.1f);
    assert(out[25]==2 && out[26]==3 && out[27]==120 && out[28]==130 && out[29]==30 && out[30]==31);
    for(i=0;i<4;i++) { assert(out[31+i]==rl_output_dm_cmd_nm[i]); }
    assert(out[35]==-2 && out[36]==2 && out[37]==123 && fabsf(out[38]-100)<1e-5f);
    memcpy(prior,dma_buffer,160);count=send_count;
    standup_control.phase=3;Robot_Control_Send_Vofa();
    assert(send_count==count && memcmp(prior,dma_buffer,160)==0);
    lqr_engaged=1;leg_balance.F[0]=210;leg_balance.F[1]=220;leg_balance.Tp[0]=1;leg_balance.Tp[1]=2;
    lqr_state.leg_len_tgt[0]=.21f;lqr_state.leg_len_tgt[1]=.22f;
    fixture_machine.lqr.leg_trim[0]=-.06f;fixture_machine.lqr.leg_trim[1]=-.05f;
    frame(out);assert(out[1]==509 && out[3]==3 && out[19]==.21f && out[20]==.22f);
    assert(fabsf(out[23]+.46f)<1e-6f && fabsf(out[24]+.45f)<1e-6f);
    assert(out[27]==210 && out[28]==220 && out[29]==1 && out[30]==2);
    leg_l.output.virtual_leg_angle=NAN;frame(out);assert(isnan(out[21]));
    return 0;
}
"""


class VofaStandupTest(unittest.TestCase):
    def test_packing(self):
        compiler=os.environ.get('CC') or shutil.which('gcc')
        if compiler is None:self.skipTest('set CC to host gcc')
        headers=PREFIX
        for path in ('imcalib/Algorithm/leg_solver.h','imcalib/Algorithm/imu_state.h','imcalib/user-lib/pid.h',
                     'imcalib/user-lib/rc_command.h',
                     'imcalib/user-lib/simple-function.h','imcalib/user-lib/kalman.h','imcalib/Algorithm/lqr_gain_table.h',
                     'imcalib/Algorithm/torque_output.h','imcalib/Algorithm/lqr_balance.h','imcalib/Algorithm/leg_balance.h',
                     'imcalib/Algorithm/standup.h'):
            headers+='\n'+without_includes(read(path))
        content=headers+'\n'+FIXTURE+'\n'+production_function(read('imcalib/user-lib/Vofa_send.c'),'Vofa_Send')
        content+='\n'+production_function(read('imcalib/task/task_comm.c'),'Robot_Control_Send_Vofa')+'\n'+CHECK
        with tempfile.TemporaryDirectory(prefix='vofa-standup-') as temp:
            folder=Path(temp);(folder/'arm_math.h').write_text(ARM_MATH_SHIM,encoding='utf-8')
            source=folder/'vofa_standup.c';source.write_text(content,encoding='utf-8')
            for machine in (0,1):
                with self.subTest(machine=machine):
                    exe=folder/f'vofa_standup{machine}.exe'
                    r=subprocess.run([compiler,'-std=c99','-Wall','-Wextra','-Werror',f'-DMACHINE_DEFAULT={machine}',
                                      '-I',str(ROOT/'imcalib/Algorithm'),'-I',str(ROOT/'imcalib/user-lib'),
                                      str(source),'-lm','-o',str(exe)],capture_output=True,text=True)
                    self.assertEqual(r.returncode,0,r.stdout+r.stderr)
                    r=subprocess.run([str(exe)],capture_output=True,text=True)
                    self.assertEqual(r.returncode,0,r.stdout+r.stderr)


if __name__=='__main__':unittest.main()
