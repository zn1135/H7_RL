"""Real gravity model and geometry; independent potential and force checks."""
import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]

HARNESS = r"""
#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "gravity_comp.h"

static void near(float a, float b)
{
    if(!isfinite(a) || !isfinite(b) || fabsf(a-b)>0.00003f) {
        fprintf(stderr,"%g != %g\n",a,b);assert(0);
    }
}
static void table_and_potential(void)
{
    const machine_gravity_cfg_t *cfg = machine_table[0].gravity;
    gravity_leg_result_t result;
    machine_gravity_cfg_t broken;
    unsigned i;
    float theta, h=.001f, up, down, expected;
    assert(cfg && cfg->count==17);
    near(cfg->mass,1.408f);near(cfg->gravity,9.81f);
    for(i=0;i<cfg->count;i++) {
        assert(Gravity_Comp_Compute(cfg,cfg->nodes[i].length,.7f,&result));
        near(result.offset,cfg->nodes[i].offset);near(result.radius,cfg->nodes[i].radius);
        assert(!result.clamped && result.valid);
        assert(Gravity_Comp_Compute(cfg,cfg->nodes[i].length,cfg->nodes[i].offset,&result));
        near(result.torque,0);
        if(i+1<cfg->count) {
            assert(Gravity_Comp_Compute(cfg,(cfg->nodes[i].length+cfg->nodes[i+1].length)/2,.7f,&result));
            near(result.offset,(cfg->nodes[i].offset+cfg->nodes[i+1].offset)/2);
            near(result.radius,(cfg->nodes[i].radius+cfg->nodes[i+1].radius)/2);
        }
    }
    assert(Gravity_Comp_Compute(cfg,.23f,0,&result));near(result.torque,-.199209266f);
    for(i=0;i<81;i++) {
        theta=-4.f+i*.1f;
        assert(Gravity_Comp_Compute(cfg,.23f,theta,&result));
        up=-cfg->mass*cfg->gravity*result.radius*cosf(theta+h-result.offset);
        down=-cfg->mass*cfg->gravity*result.radius*cosf(theta-h-result.offset);
        assert(fabsf(result.torque-(up-down)/(2*h))<.00015f);
        /* Reference rear-positive angle and moment are both negated. */
        expected=-cfg->mass*cfg->gravity*result.radius*sinf(-theta+result.offset);
        near(result.torque,expected);
    }
    assert(Gravity_Comp_Compute(cfg,.13f,0,&result) && result.clamped);
    near(result.radius,cfg->nodes[0].radius);
    assert(Gravity_Comp_Compute(cfg,.34f,0,&result) && result.clamped);
    near(result.radius,cfg->nodes[16].radius);
    assert(!Gravity_Comp_Compute(cfg,NAN,0,&result) && !result.valid && result.torque==0);
    assert(!Gravity_Comp_Compute(cfg,.23f,INFINITY,&result));
    assert(!Gravity_Comp_Compute(NULL,.23f,0,&result));
    assert(!Gravity_Comp_Compute(cfg,.23f,0,NULL));
    broken=*cfg;broken.mass=NAN;assert(!Gravity_Comp_Compute(&broken,.23f,0,&result));
    broken=*cfg;broken.mass=FLT_MAX;broken.gravity=FLT_MAX;
    assert(!Gravity_Comp_Compute(&broken,.23f,.8f,&result));
}
static void adapter(void)
{
    leg_state_t left,right;
    gravity_comp_result_t result;
    float base[4]={1,-2,3,-4},raw[4],angles[2]={.2f,-.3f};
    float a,b,c,d,det,delta0,delta1,recovered_force,recovered_tp;
    unsigned i,side;
    const leg_state_t *legs[2];
    Leg_Init(&left);left.config.configured=1;left.config.lu=.21f;left.config.lg=.25f;
    left.input.hip_f=2.6f;left.input.hip_b=.4f;right=left;right.input.hip_f=2.4f;
    assert(Leg_Solve(&left)&&Leg_Solve(&right));legs[0]=&left;legs[1]=&right;
    assert(Gravity_Comp_Apply(&left,&right,angles,base,raw,&result));
    if(machine->gravity==NULL) {
        assert(!result.valid && !memcmp(base,raw,sizeof(base)));
        assert(Gravity_Comp_Apply(NULL,NULL,NULL,base,base,&result));
    } else {
        assert(result.valid);
        for(side=0;side<2;side++) {
            a=legs[side]->output.force_map[0][0];b=legs[side]->output.force_map[0][1];
            c=legs[side]->output.force_map[1][0];d=legs[side]->output.force_map[1][1];det=a*d-b*c;
            delta0=raw[side*2]-base[side*2];delta1=raw[side*2+1]-base[side*2+1];
            recovered_force=(d*delta0-b*delta1)/det;
            recovered_tp=(-c*delta0+a*delta1)/det;
            near(recovered_force,0);near(recovered_tp,result.leg[side].torque);
        }
        assert(Gravity_Comp_Apply(&left,&right,angles,base,base,&result));
        near(base[0],raw[0]);
        right.output.force_valid=0;
        assert(!Gravity_Comp_Apply(&left,&right,angles,base,raw,&result));
        for(i=0;i<4;i++) {assert(raw[i]==0 && result.delta_dm[i]==0);}
        assert(!result.valid);right.output.force_valid=1;
        angles[1]=NAN;assert(!Gravity_Comp_Apply(&left,&right,angles,base,raw,&result));
        angles[1]=-.3f;
        left.output.force_map[0][1]=NAN;
        assert(!Gravity_Comp_Apply(&left,&right,angles,base,raw,&result));
        assert(Leg_Solve(&left));
        angles[0]=1.5f;
        left.output.force_map[0][1]=FLT_MAX*.75f;
        base[0]=FLT_MAX*.75f;
        assert(!Gravity_Comp_Apply(&left,&right,angles,base,raw,&result));
        for(i=0;i<4;i++) {assert(raw[i]==0);}
    }
    base[0]=NAN;assert(!Gravity_Comp_Apply(&left,&right,angles,base,raw,&result));
    for(i=0;i<4;i++) {assert(raw[i]==0);}
    assert(!Gravity_Comp_Apply(&left,&right,angles,NULL,raw,&result));
    assert(!Gravity_Comp_Apply(&left,&right,angles,base,NULL,&result));
}
int main(int argc,char **argv)
{
    assert(argc==2);
    if(atoi(argv[1])==0) {table_and_potential();} else {adapter();}
    return 0;
}
"""


class GravityCompTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = os.environ.get("CC") or shutil.which("gcc")
        if compiler is None:
            raise unittest.SkipTest("set CC to host gcc")
        cls.temp = tempfile.TemporaryDirectory(prefix="gravity-comp-")
        cls.addClassCleanup(cls.temp.cleanup)
        folder = Path(cls.temp.name)
        source = folder / "gravity.c"
        source.write_text(HARNESS, encoding="utf-8")
        cls.environment = os.environ.copy()
        cls.environment["PATH"] = str(Path(compiler).parent) + os.pathsep + cls.environment.get("PATH", "")
        cls.executables = []
        for machine in (0, 1):
            exe = folder / f"gravity_{machine}.exe"
            result = subprocess.run([compiler, "-std=c99", "-Wall", "-Wextra", "-Werror",
                "-DLEG_TRIG_LIBM=1", f"-DMACHINE_DEFAULT={machine}",
                "-I", str(ROOT / "imcalib/Algorithm"), "-I", str(ROOT / "imcalib/user-lib"),
                str(source), str(ROOT / "imcalib/Algorithm/gravity_comp.c"),
                str(ROOT / "imcalib/Algorithm/leg_solver.c"), str(ROOT / "imcalib/user-lib/machine_config.c"),
                "-lm", "-o", str(exe)], capture_output=True, text=True, env=cls.environment)
            if result.returncode:
                raise AssertionError(result.stdout + result.stderr)
            cls.executables.append(exe)

    def run_case(self, case):
        for exe in self.executables:
            with self.subTest(machine=exe.name):
                result = subprocess.run([str(exe), str(case)], capture_output=True, text=True, env=self.environment)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_nodes_interpolation_reference_direction_and_potential(self):
        self.run_case(0)

    def test_zero_axial_force_alias_atomic_errors_and_small_bypass(self):
        self.run_case(1)

    def test_model_is_exact_existing_accepted_snapshot(self):
        text = (ROOT / "tools/matlab/reference/leg3_leg_data.m").read_text(encoding="utf-8")
        rows = re.findall(r"^\s*([\d.]+),\s*([\d.]+),\s*([\d.]+),", text, re.M)[:17]
        config = (ROOT / "imcalib/user-lib/machine_config.c").read_text(encoding="utf-8")
        table = config.split("gravity_nodes_big[] = {", 1)[1].split("};", 1)[0]
        actual = re.findall(r"\{([\d.]+)f, ([\d.]+)f, ([\d.]+)f\}", table)
        self.assertEqual([[float(v) for v in row] for row in actual], [[float(v) for v in row] for row in rows])
