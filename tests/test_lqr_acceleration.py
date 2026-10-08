"""Exercise real acceleration projection with independent synthetic IMU inputs."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

from test_gas_spring_integration import ROOT, production_function, read

FIXTURE = r"""
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "machine_config.h"
#include "imu_state.h"
#include "slip.h"
#define LQR_GRAVITY 9.81f
"""

CHECK = r"""
/* Rotate using quaternion vector products, independently of production matrix. */
static void rotate(const float q[4],const float v[3],float out[3])
{
    float t[3];
    t[0]=2*(q[2]*v[2]-q[3]*v[1]);
    t[1]=2*(q[3]*v[0]-q[1]*v[2]);
    t[2]=2*(q[1]*v[1]-q[2]*v[0]);
    out[0]=v[0]+q[0]*t[0]+q[2]*t[2]-q[3]*t[1];
    out[1]=v[1]+q[0]*t[1]+q[3]*t[0]-q[1]*t[2];
    out[2]=v[2]+q[0]*t[2]+q[1]*t[1]-q[2]*t[0];
}
static void sample(float roll,float pitch,float yaw,float gravity,float acceleration,imu_state_t *imu)
{
    float qr[4],inverse[4],forward[3]={0},heading[3],world[3],raw[3],norm;
    float cr=cosf(roll/2),sr=sinf(roll/2),cp=cosf(pitch/2),sp=sinf(pitch/2);
    float cy=cosf(yaw/2),sy=sinf(yaw/2);
    unsigned i;
    memset(imu,0,sizeof(*imu));
    qr[0]=cr*cp*cy+sr*sp*sy;qr[1]=sr*cp*cy-cr*sp*sy;
    qr[2]=cr*sp*cy+sr*cp*sy;qr[3]=cr*cp*sy-sr*sp*cy;
    forward[machine->imu.quat_src[0]]=(float)machine->imu.quat_sign[0];
    rotate(qr,forward,heading);norm=hypotf(heading[0],heading[1]);assert(norm>.1f);
    world[0]=acceleration/LQR_GRAVITY*heading[0]/norm;
    world[1]=acceleration/LQR_GRAVITY*heading[1]/norm;world[2]=gravity;
    inverse[0]=qr[0];for(i=1;i<4;i++) { inverse[i]=-qr[i]; }
    rotate(inverse,world,raw);
    imu->quat[0]=qr[0];
    for(i=0;i<3;i++)
    {
        imu->quat[i+1]=machine->imu.quat_sign[i]*qr[machine->imu.quat_src[i]+1];
        imu->acc_g[i]=machine->imu.acc_sign[i]*raw[machine->imu.acc_src[i]];
    }
}
int main(void)
{
    const float rolls[]={-.6f,0,.6f},pitches[]={-.7f,-.3f,0,.3f,.7f},yaws[]={-2,0,2};
    const float gravities[]={-1,1},accelerations[]={-2,0,2};
    imu_state_t imu,before;slip_state_t fusion;unsigned r,p,y,g,a,count=0,i;float value,error,maximum=0;
    for(r=0;r<3;r++)for(p=0;p<5;p++)for(y=0;y<3;y++)for(g=0;g<2;g++)for(a=0;a<3;a++)
    {
        sample(rolls[r],pitches[p],yaws[y],gravities[g],accelerations[a],&imu);before=imu;
        value=LQR_Accel_Forward(&imu);error=fabsf(value-accelerations[a]);
        if(error>maximum) { maximum=error; }
        if(!isfinite(value)||error>2e-5f)
        {
            fprintf(stderr,"machine %u pose %g %g %g expected %g got %g\n",MACHINE_DEFAULT,
                rolls[r],pitches[p],yaws[y],accelerations[a],value);assert(0);
        }
        assert(memcmp(&before,&imu,sizeof(imu))==0);count++;
    }
    sample(.6f,-.7f,1.2f,1,0,&imu);
    Slip_Init(&fusion,0,0);
    for(i=0;i<2000;i++)
    {
        value=LQR_Accel_Forward(&imu);assert(Slip_Update(&fusion,0,value,.001f));
        assert(fabsf(fusion.velocity)<1e-5f);
    }
    printf("machine %u: %u static/motion cases, max residual %.9g m/s2\n",MACHINE_DEFAULT,count,maximum);
    return 0;
}
"""


class LqrAccelerationTest(unittest.TestCase):
    def test_static_gravity_and_motion(self):
        compiler=os.environ.get('CC') or shutil.which('gcc')
        if compiler is None:self.skipTest('set CC to host gcc')
        content=FIXTURE+'\n'+production_function(read('imcalib/Algorithm/lqr_balance.c'),'LQR_Accel_Forward')+'\n'+CHECK
        with tempfile.TemporaryDirectory(prefix='lqr-accel-') as temp:
            folder=Path(temp);source=folder/'lqr_accel.c';source.write_text(content,encoding='utf-8')
            for machine in (0,1):
                with self.subTest(machine=machine):
                    exe=folder/f'lqr_accel{machine}.exe'
                    r=subprocess.run([compiler,'-std=c99','-Wall','-Wextra','-Werror',f'-DMACHINE_DEFAULT={machine}',
                        '-I',str(ROOT/'imcalib/Algorithm'),'-I',str(ROOT/'imcalib/user-lib'),str(source),
                        str(ROOT/'imcalib/user-lib/machine_config.c'),str(ROOT/'imcalib/Algorithm/slip.c'),
                        '-lm','-o',str(exe)],capture_output=True,text=True)
                    self.assertEqual(r.returncode,0,r.stdout+r.stderr)
                    r=subprocess.run([str(exe)],capture_output=True,text=True)
                    self.assertEqual(r.returncode,0,r.stdout+r.stderr)
                    print(r.stdout.strip())


if __name__=='__main__':unittest.main()
