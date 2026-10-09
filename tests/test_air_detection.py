"""Production force observer, contact transitions, masked LQR and composition."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

from test_lqr_unified import PREFIX
from test_gas_spring_integration import ROOT, ARM_MATH_SHIM, read, without_includes

CHECK = r"""
static void near(float a,float b,float tolerance)
{
    if(!isfinite(a)||!isfinite(b)||fabsf(a-b)>tolerance) {
        fprintf(stderr,"%g != %g\n",a,b);assert(0);
    }
}
static void rotate(const float q[4],const float v[3],float o[3])
{
    float t[3]={2*(q[2]*v[2]-q[3]*v[1]),2*(q[3]*v[0]-q[1]*v[2]),2*(q[1]*v[1]-q[2]*v[0])};
    o[0]=v[0]+q[0]*t[0]+q[2]*t[2]-q[3]*t[1];
    o[1]=v[1]+q[0]*t[1]+q[3]*t[0]-q[1]*t[2];
    o[2]=v[2]+q[0]*t[2]+q[1]*t[1]-q[2]*t[0];
}
static void sample(imu_state_t *imu,float r,float p,float y,float az)
{
    float q[4],inv[4],world[3]={.1f,-.2f,1+az/9.81f},raw[3];unsigned i;
    float cr=cosf(r/2),sr=sinf(r/2),cp=cosf(p/2),sp=sinf(p/2),cy=cosf(y/2),sy=sinf(y/2);
    memset(imu,0,sizeof(*imu));imu->online=1;
    q[0]=cr*cp*cy+sr*sp*sy;q[1]=sr*cp*cy-cr*sp*sy;
    q[2]=cr*sp*cy+sr*cp*sy;q[3]=cr*cp*sy-sr*sp*cy;
    inv[0]=q[0];for(i=1;i<4;i++) { inv[i]=-q[i]; }rotate(inv,world,raw);
    imu->quat[0]=q[0];
    for(i=0;i<3;i++) {
        imu->quat[i+1]=machine->imu.quat_sign[i]*q[machine->imu.quat_src[i]+1];
        imu->acc_g[i]=machine->imu.acc_sign[i]*raw[machine->imu.acc_src[i]];
    }
}
static void setup_leg(leg_state_t *leg,float angle)
{
    Leg_Init(leg);leg->config.configured=1;leg->config.lu=machine->leg_lu;leg->config.lg=machine->leg_lg;
    leg->input.hip_f=LEG_PI-.4f-angle;leg->input.hip_b=.4f-angle;
    assert(Leg_Solve(leg));
}
static void geometry(void)
{
    leg_state_t leg;float motor[2],f,t,pressure;unsigned i;
    for(i=0;i<500;i++) {
        setup_leg(&leg,-.8f+i*.003f);
        f=13.f+i*.1f;t=-.6f+i*.003f;
        assert(Leg_Force_Map_Forward(&leg,f,t,motor));
        assert(Leg_Force_Map_Inverse(&leg,motor,&f,&t));
        near(f,13.f+i*.1f,.0001f);near(t,-.6f+i*.003f,.00001f);
        /* Independent vertical external load in front-positive coordinates. */
        f=50*cosf(leg.output.virtual_leg_angle);
        t=-50*leg.output.virtual_leg_length*sinf(leg.output.virtual_leg_angle);
        assert(Leg_Force_Map_Forward(&leg,f,t,motor));
        assert(Leg_Force_Map_Inverse(&leg,motor,&f,&t));
        pressure=f*cosf(leg.output.virtual_leg_angle)-t*sinf(leg.output.virtual_leg_angle)/leg.output.virtual_leg_length;
        near(pressure,50,.0001f);
    }
    leg.output.force_map[1][0]=leg.output.force_map[0][0];
    leg.output.force_map[1][1]=leg.output.force_map[0][1];
    assert(!Leg_Force_Map_Inverse(&leg,motor,&f,&t)&&f==0&&t==0);
    setup_leg(&leg,0);motor[0]=NAN;assert(!Leg_Force_Map_Inverse(&leg,motor,&f,&t));
}
static void acceleration(void)
{
    imu_state_t imu;float out,az;unsigned i,j;
    for(i=0;i<120;i++)for(j=0;j<3;j++) {
        az=j==0?-9.81f:(j==1?0:2.5f);
        sample(&imu,-.6f+(i%3)*.6f,-.7f+(i%5)*.35f,-2.f+(i%7)*.6f,az);
        assert(Attitude_Accel_Vertical(&imu,&out));near(out,az,.00001f);
    }
    imu.quat[0]=NAN;assert(!Attitude_Accel_Vertical(&imu,&out));
}
static air_input_t input;
static leg_state_t legs[2];static imu_state_t imu;
static void support(float left,float right,float dt)
{
    float fn[2]={left,right},length,theta,f,t;gravity_leg_result_t gravity;unsigned side;
    input.now_ns+=(uint64_t)(dt*1e9f);imu.last_timestamp_ms++;
    for(side=0;side<2;side++) {
        length=legs[side].output.virtual_leg_length;theta=legs[side].output.virtual_leg_angle;
        t=0;
        if(machine->gravity) { assert(Gravity_Comp_Compute(machine->gravity,length,theta,&gravity));t=gravity.torque; }
        f=(fn[side]-air_param.wheel_mass*9.81f)/cosf(theta)-Gas_Spring_Force(length);
        assert(Leg_Force_Map_Forward(&legs[side],f,t,input.torque[side]));
        input.rx_ns[side][0]=input.rx_ns[side][1]=input.now_ns;
    }
}
static void observer(void)
{
    air_detection_t st;air_param_t original=air_param;unsigned i;float previous;
    air_param.force_tau=0;air_param.derivative_tau=0;air_param.control_enabled=1;
    Air_Detection_Reset(&st);memset(&input,0,sizeof(input));
    sample(&imu,0,0,0,0);setup_leg(&legs[0],0);setup_leg(&legs[1],0);
    input.imu=&imu;input.leg[0]=&legs[0];input.leg[1]=&legs[1];input.permit=1;input.now_ns=1000000000u;
    support(50,50,.001f);Air_Detection_Update(&st,&input);
    assert(st.valid&&!st.flight);near(st.mean_force,50,.0001f);
    legs[0].output.d_virtual_leg_length=.01f;
    support(50,50,.001f);Air_Detection_Update(&st,&input);
    near(st.leg[0].wheel_acceleration,-10,.001f);
    near(st.leg[0].force,50-air_param.wheel_mass*10,.001f);
    support(50,50,.001f);Air_Detection_Update(&st,&input);near(st.leg[0].force,50,.001f);
    legs[0].output.d_virtual_leg_length=0;
    support(50,50,.001f);Air_Detection_Update(&st,&input);
    support(50,50,.001f);Air_Detection_Update(&st,&input);
    for(i=0;i<25;i++) { support(50,50,.001f);Air_Detection_Update(&st,&input); }
    assert(st.armed);
    support(0,2*air_param.force_on,.001f);Air_Detection_Update(&st,&input);
    assert(!st.flight);near(st.mean_force,air_param.force_on,.0001f);
    for(i=0;i<9;i++) { support(0,0,.001f);Air_Detection_Update(&st,&input);assert(!st.flight); }
    previous=st.elapsed;
    for(i=0;i<3;i++) { input.now_ns+=1000000;Air_Detection_Update(&st,&input);near(st.elapsed,previous,1e-6f); }
    support(50,50,.001f);Air_Detection_Update(&st,&input);assert(st.elapsed==0);
    for(i=0;i<10;i++) { support(0,0,.001f);Air_Detection_Update(&st,&input); }
    assert(st.flight&&Air_Detection_Active(&st));
    /* Data loss retains flight; valid recovery must debounce landing again. */
    input.torque[0][0]=NAN;input.rx_ns[0][0]++;input.rx_ns[0][1]++;
    input.rx_ns[1][0]++;input.rx_ns[1][1]++;input.now_ns++;
    Air_Detection_Update(&st,&input);assert(!st.valid&&st.flight);
    input.now_ns+=60000000;Air_Detection_Update(&st,&input);
    assert(st.flight&&st.fault&&Air_Detection_Active(&st));
    support(50,50,.001f);Air_Detection_Update(&st,&input);assert(st.flight&&st.elapsed==0);
    for(i=0;i<19;i++) { support(50,50,.001f);Air_Detection_Update(&st,&input);assert(st.flight); }
    support(50,50,.001f);Air_Detection_Update(&st,&input);
    assert(!st.flight&&st.valid&&st.elapsed==0&&!Air_Detection_Active(&st));
    for(i=0;i<5;i++) { support(0,0,.001f);Air_Detection_Update(&st,&input);assert(!st.flight); }
    support(50,50,.001f);Air_Detection_Update(&st,&input);assert(st.elapsed==0);
    for(i=0;i<10;i++) { support(0,0,.001f);Air_Detection_Update(&st,&input); }
    assert(st.flight);
    input.rx_ns[0][0]-=10000000;Air_Detection_Update(&st,&input);assert(!st.valid);
    input.permit=0;Air_Detection_Update(&st,&input);assert(!st.valid&&!st.flight);
    air_param=original;
}
static void control(void)
{
    lqr_state_t st,other,ground;leg_balance_t balance,reference;torque_output_t cmd,reference_cmd;
    slip_state_t fusion;float old;unsigned j,k;
    LQR_Init(&st);st.valid=1;st.len[0]=st.len[1]=.2f;
    st.air_control=1;st.air_hip_max=8;
    LQR_Control_Update(&st);assert(st.gain_valid);other=st;
    for(j=0;j<10;j++) {
        if(j>=4&&j<=7) { continue; }
        other=st;other.x[j]=.2f;LQR_Control_Update(&other);
        for(k=0;k<4;k++) { near(other.u[k],st.u[k],1e-6f); }
    }
    other=st;other.x[4]=.01f;LQR_Control_Update(&other);
    assert(other.u[0]==0&&other.u[1]==0&&fabsf(other.u[2])>.0001f);
    near(other.u[2],-.01f*other.K[2][4],1e-5f);
    other=st;other.target[4]=LEG_PI+.005f;other.x[4]=-LEG_PI+.005f;
    LQR_Control_Update(&other);near(other.u[2],0,.00001f);near(other.u[3],0,.00001f);
    other=st;other.x[4]=.01f;LQR_Control_Update(&other);
    ground=other;ground.air_control=0;LQR_Control_Update(&ground);
    other.air_control=0;LQR_Control_Update(&other);
    for(k=0;k<4;k++) { near(other.u[k],ground.u[k],1e-6f); }
    setup_leg(&legs[0],0);setup_leg(&legs[1],0);
    st.leg_len_tgt[0]=legs[0].output.virtual_leg_length;
    st.leg_len_tgt[1]=legs[1].output.virtual_leg_length;
    st.roll=.1f;ground=st;ground.air_control=0;
    Leg_Balance_Init(&balance);Leg_Balance_Init(&reference);
    for(j=0;j<60;j++) {
        st.air_control=(j/10)%2;
        st.leg_len_tgt[0]-=.0001f;ground.leg_len_tgt[0]=st.leg_len_tgt[0];
        assert(Leg_Balance_Compute(&balance,&st,&legs[0],&legs[1],.001f,&cmd));
        assert(Leg_Balance_Compute(&reference,&ground,&legs[0],&legs[1],.001f,&reference_cmd));
        near(balance.F[0],reference.F[0],1e-6f);near(balance.F[1],reference.F[1],1e-6f);
        assert(!memcmp(&balance.leg_len,&reference.leg_len,sizeof(balance.leg_len)));
        assert(!memcmp(&balance.length_ramp,&reference.length_ramp,sizeof(balance.length_ramp)));
        assert(cmd.dji[0]==0&&cmd.dji[1]==0);
    }
    Slip_Init(&fusion,1,0);old=fusion.velocity;
    for(j=0;j<100;j++) { assert(Slip_Update_Contact(&fusion,1000,0,.001f,0)); }
    near(fusion.velocity,old,1e-6f);assert(Slip_Update_Contact(&fusion,1,0,.001f,1));
    st.air_hip_max=NAN;LQR_Control_Update(&st);assert(!st.gain_valid);
}
int main(int argc,char **argv)
{
    int test=argc>1?atoi(argv[1]):0;
    switch(test) { case 0:geometry();break;case 1:acceleration();break;case 2:observer();break;case 3:control();break;default:assert(0); }
    return 0;
}
"""


class AirDetectionTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cc = os.environ.get("CC") or shutil.which("gcc")
        if not cc:
            raise unittest.SkipTest("set CC to host gcc")
        cls.temp = tempfile.TemporaryDirectory(prefix="air-detection-")
        cls.addClassCleanup(cls.temp.cleanup)
        folder = Path(cls.temp.name)
        (folder / "arm_math.h").write_text(ARM_MATH_SHIM)
        source = PREFIX + '\n#include "air_detection.h"\n#include "Attitude_Algorithm.h"\n#include "slip.h"\n'
        for path in ("imcalib/user-lib/rc_command.h", "imcalib/Algorithm/torque_output.h",
                     "imcalib/Algorithm/lqr_balance.h", "imcalib/Algorithm/leg_balance.h",
                     "imcalib/Algorithm/gas_spring.h", "imcalib/Algorithm/lqr_balance.c",
                     "imcalib/Algorithm/leg_balance.c", "imcalib/Algorithm/gas_spring.c",
                     "imcalib/Algorithm/gravity_comp.c", "imcalib/Algorithm/Attitude_Algorithm.c",
                     "imcalib/Algorithm/air_detection.c"):
            source += '\n' + without_includes(read(path))
        harness = folder / 'air.c'
        harness.write_text(source + CHECK, encoding='utf-8')
        dependencies = ('imcalib/user-lib/machine_config.c', 'imcalib/Algorithm/lqr_gain_table.c',
                        'imcalib/Algorithm/lqr_gain_big.c', 'imcalib/Algorithm/lqr_gain_small.c',
                        'imcalib/Algorithm/leg_solver.c', 'imcalib/Algorithm/slip.c',
                        'imcalib/user-lib/pid.c', 'imcalib/user-lib/simple-function.c')
        cls.env = os.environ.copy()
        cls.env['PATH'] = str(Path(cc).parent) + os.pathsep + cls.env.get('PATH', '')
        cls.executables = []
        for machine in (0, 1):
            exe = folder / f'air{machine}.exe'
            result = subprocess.run([cc, '-std=c99', '-Wall', '-Wextra', '-Werror',
                '-DLEG_TRIG_LIBM=1', f'-DMACHINE_DEFAULT={machine}', '-I', str(folder),
                '-I', str(ROOT/'imcalib/Algorithm'), '-I', str(ROOT/'imcalib/user-lib'),
                str(harness), *[str(ROOT/p) for p in dependencies], '-lm', '-o', str(exe)],
                capture_output=True, text=True, env=cls.env)
            if result.returncode:
                raise AssertionError(result.stdout + result.stderr)
            cls.executables.append(exe)

    def test_inverse_and_independent_projection(self):
        self.run_case(0)

    def test_rotated_vertical_acceleration(self):
        self.run_case(1)

    def test_contact_mean_timing_and_recovery(self):
        self.run_case(2)

    def test_air_feedback_and_force_composition(self):
        self.run_case(3)

    def test_real_task_flight_landing_and_bench_isolation(self):
        from test_standup import StandupTest
        StandupTest().run_cases({'air_task': r'''
static void feedback_step(float fn)
{
    const leg_state_t *legs[2]={&leg_l,&leg_r};
    gravity_leg_result_t gravity;float f,t;unsigned side;
    imu_state.last_timestamp_ms++;
    for(side=0;side<2;side++) {
        t=0;
        if(machine->gravity) {
            assert(Gravity_Comp_Compute(machine->gravity,legs[side]->output.virtual_leg_length,
                legs[side]->output.virtual_leg_angle,&gravity));t=gravity.torque;
        }
        f=(fn-air_param.wheel_mass*9.81f)/cosf(legs[side]->output.virtual_leg_angle)
            -Gas_Spring_Force(legs[side]->output.virtual_leg_length);
        assert(Leg_Force_Map_Forward(legs[side],f,t,&motor_state.dm.trq_nm[side*2]));
        motor_state.dm.parsed_rx_ns[side*2]=motor_state.dm.parsed_rx_ns[side*2+1]=now_ns+1000000;
        motor_state.dm.online[side*2]=motor_state.dm.online[side*2+1]=1;
    }
    step();
}
int main(void)
{
    unsigned i;float held,previous_set;
    setup(0);standup_control.enabled=0;air_param.control_enabled=1;
    air_param.force_tau=air_param.derivative_tau=0;
    imu_state.acc_g[2]=-1;now_ns=1000000000u;
    for(i=0;i<35;i++) { feedback_step(0); }
    assert(!air_detection.flight && !lqr_state.air_control);
    for(i=0;i<25;i++) { feedback_step(50); }
    lqr_state.leg_len_tgt[0]=.20f;lqr_state.leg_len_tgt[1]=.18f;
    assert(lqr_running && air_detection.valid && !lqr_state.air_control);
    rc_command.vel=1;rc_command.yaw=1;rc_command.len=0;
    for(i=0;i<12;i++) { feedback_step(0); }
    assert(lqr_state.air_control && air_detection.flight && sent_wheel[0]==0 && sent_wheel[1]==0);
    assert(lqr_state.target[LQR_X_DS]==0 && lqr_state.target[LQR_X_DPHI]==0);
    assert(lqr_state.leg_len_tgt[0]==.20f && lqr_state.leg_len_tgt[1]==.18f);
    held=lqr_state.leg_len_tgt[0];
    for(i=0;i<20;i++) { feedback_step(0); }
    assert(lqr_state.leg_len_tgt[0]==held && lqr_state.pos==0 && !lqr_state.pos_armed);
    rc_command.len=.3f;feedback_step(0);
    assert(fabsf(lqr_state.leg_len_tgt[0]-(held+.3f*machine->lqr.len_rate*MACHINE_LQR_DT))<1e-6f);
    held=lqr_state.leg_len_tgt[0];rc_command.len=0;
    assert(air_debug.applied && air_debug.flight);
    fixture_dm_enable_mask=7;feedback_step(0);
    assert(air_detection.flight && !air_detection.valid && sent_wheel[0]==0 && sent_wheel[1]==0);
    for(i=0;i<60;i++) { feedback_step(0); }
    assert(air_detection.fault && lqr_running && leg_balance.cmd.valid);
    assert(lqr_state.leg_len_tgt[0]==held);
    fixture_dm_enable_mask=15;feedback_step(0);
    for(i=0;i<19;i++) { feedback_step(50); }
    previous_set=leg_balance.leg_len[0].set[NOW];feedback_step(50);
    assert(!air_detection.flight && !lqr_state.air_control);
    assert(lqr_state.leg_len_tgt[0]==held);
    assert(fabsf(leg_balance.leg_len[0].set[NOW]-(previous_set-machine->lqr.len_rate*MACHINE_LQR_DT))<1e-6f);
    assert(lqr_state.yaw_tgt==lqr_state.x[LQR_X_PHI]);
    assert(lqr_state.target[LQR_X_DS]==1 && lqr_state.target[LQR_X_DPHI]==1);
    for(i=0;i<12;i++) { feedback_step(0); }
    assert(air_detection.flight);
    rc_command.s1=DR16_SW_UP;fixture_machine.rl.configured=1;action_state.rl_ready=1;rl_fixture_ok=1;
    feedback_step(0);assert(!lqr_state.air_control && !air_detection.valid && !air_detection.flight);
    assert(output_task_rl_engaged() && sent_wheel[0]==.25f);
    rc_command.s1=DR16_SW_MID;rc_command.s2=DR16_SW_UP;feedback_step(0);
    assert(!air_detection.valid && !air_detection.flight && !air_debug.applied);
    rc_command.s2=DR16_SW_MID;standup_control.enabled=1;
    Standup_Enter(&standup_control,STANDUP_RETRACT);
    for(i=0;i<15;i++) { feedback_step(0); }
    assert(standup_control.phase==STANDUP_RETRACT && standup_control.prepare.valid);
    assert(!air_detection.armed && !air_detection.flight && !air_detection.valid);
    assert(standup_control.length_cmd[0]==standup_param.retract_len);
    assert(lqr_state.leg_len_tgt[0]==held);
    rc_command.s1=DR16_SW_DOWN;feedback_step(0);zero();
    return 0;
}
'''})

    def run_case(self, case):
        for machine, exe in enumerate(self.executables):
            with self.subTest(machine=machine):
                result = subprocess.run([str(exe), str(case)], capture_output=True, text=True, env=self.env)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


if __name__ == '__main__':
    unittest.main()
