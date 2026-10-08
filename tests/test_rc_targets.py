"""Check the real RC -> LQR targets and RL command unit conversion."""
import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import unittest

from test_gas_spring_integration import ARM_MATH_SHIM, production_function, without_includes
from test_lqr_unified import PREFIX
from test_rl_command_input import production_type

ROOT = Path(__file__).resolve().parents[1]


def read(path):
    return (ROOT / path).read_text(encoding="utf-8")


CHECKS = r"""
static void near(float a, float b) { assert(isfinite(a) && fabsf(a-b)<0.00001f); }
int main(void)
{
    dr16_t r={0}; lqr_state_t s; float out[3],len;
    r.online=1; r.s1=r.s2=DR16_SW_MID; r.ch0=r.ch3=660;
    LQR_Init(&s); LQR_Enable_Latch(&s,NULL,NULL);
    Rc_Command_Update(&rc_command,&r); near(rc_command.vel,0); near(rc_command.yaw,0);
    r.ch1=10; r.ch2=-20; r.wheel=20; Rc_Command_Update(&rc_command,&r);
    near(rc_command.vel,0); near(rc_command.yaw,0); near(rc_command.len,0);
    r.ch1=11; r.ch2=21; Rc_Command_Update(&rc_command,&r);
    near(rc_command.vel,11.0f/660*machine->lqr.vel_max);
    near(rc_command.yaw,-21.0f/660*machine->lqr.yaw_max);
    assert(LQR_Target_Update(&s,&rc_command,MACHINE_LQR_DT));
    near(s.target[LQR_X_DS],rc_command.vel);
    r.ch1=660; r.ch2=660; r.wheel=660; Rc_Command_Update(&rc_command,&r);
    near(rc_command.vel,machine->lqr.vel_max); near(rc_command.yaw,-machine->lqr.yaw_max);
    near(rc_command.len,1); len=s.leg_len_tgt[0]; s.x[LQR_X_PHI]=.4f;
    assert(LQR_Target_Update(&s,&rc_command,MACHINE_LQR_DT));
    near(s.target[LQR_X_DS],machine->lqr.vel_max); near(s.target[LQR_X_DPHI],-machine->lqr.yaw_max);
    near(s.yaw_tgt,.4f); near(s.leg_len_tgt[0],len+machine->lqr.len_rate*MACHINE_LQR_DT);
    RL_Command_From_Rc(out); near(out[0],RL_CMD_VX_MAX); near(out[1],RL_CMD_YAW_MAX);
    r.ch1=r.ch2=r.wheel=0; Rc_Command_Update(&rc_command,&r); s.x[LQR_X_PHI]=.5f;
    assert(LQR_Target_Update(&s,&rc_command,MACHINE_LQR_DT));
    near(s.target[LQR_X_DS],0);
    near(s.target[LQR_X_DPHI],0); near(s.yaw_tgt,.4f);
    r.ch1=-660; r.ch2=-660; Rc_Command_Update(&rc_command,&r);
    assert(LQR_Target_Update(&s,&rc_command,MACHINE_LQR_DT));
    near(s.target[LQR_X_DS],-machine->lqr.vel_max); near(s.target[LQR_X_DPHI],machine->lqr.yaw_max);
    RL_Command_From_Rc(out); near(out[0],-RL_CMD_VX_MAX); near(out[1],-RL_CMD_YAW_MAX);
    r.ch1=1000; r.ch2=-1000; Rc_Command_Update(&rc_command,&r);
    near(rc_command.vel,machine->lqr.vel_max); near(rc_command.yaw,machine->lqr.yaw_max);
    r.online=0; Rc_Command_Update(&rc_command,&r);
    near(rc_command.vel,0); near(rc_command.yaw,0); near(rc_command.len,0);
    assert(!rc_command.online && !rc_command.s1 && !rc_command.s2);
    assert(!LQR_Target_Update(&s,&rc_command,MACHINE_LQR_DT));
    return 0;
}
"""


class RcTargetsTest(unittest.TestCase):
    def test_raw_channels_targets_and_rl_units(self):
        compiler = os.environ.get("CC") or shutil.which("gcc")
        if compiler is None:
            self.skipTest("set CC to host gcc")
        environment = os.environ.copy()
        environment["PATH"] = str(Path(compiler).parent) + os.pathsep + environment.get("PATH", "")
        source = "#include <stdbool.h>\n" + PREFIX.replace(
            "typedef struct { int unused; } dr16_t;", production_type(read("imcalib/user-lib/dr16.h"), "dr16_t"))
        source += "\n" + "\n".join(re.findall(r"^#define DR16_[^\n]+", read("imcalib/user-lib/dr16.h"), re.M))
        source += "\n" + "\n".join(re.findall(r"^#define RL_CMD_[^\n]+", read("imcalib/Algorithm/rl_policy.h"), re.M))
        for path in ("imcalib/user-lib/rc_command.h", "imcalib/Algorithm/lqr_balance.h"):
            source += "\n" + without_includes(read(path))
        source += "\nstatic rc_command_t rc_command;\nstatic struct { float vx_cmd,yaw_cmd,height_cmd; } input_command;\n"
        source += "static uint8_t output_task_rl_engaged(void) { return 1u; }\n"
        source += production_function(read("imcalib/user-lib/dr16.c"), "DR16_Deadline")
        for path in ("imcalib/user-lib/rc_command.c", "imcalib/Algorithm/lqr_balance.c"):
            source += "\n" + without_includes(read(path))
        source += "\n" + production_function(read("imcalib/task/task_policy.c"), "RL_Command_From_Rc")
        dependencies = ("imcalib/user-lib/machine_config.c", "imcalib/Algorithm/lqr_gain_table.c",
                        "imcalib/Algorithm/lqr_gain_big.c", "imcalib/Algorithm/lqr_gain_small.c",
                        "imcalib/user-lib/simple-function.c", "imcalib/user-lib/kalman.c")
        # Also exercise nonzero training ranges without changing firmware parameters.
        active = re.sub(r"(#define RL_CMD_VX_MAX\s+)\S+", r"\g<1>1.0f", source)
        active = re.sub(r"(#define RL_CMD_YAW_MAX\s+)\S+", r"\g<1>3.0f", active)
        with tempfile.TemporaryDirectory(prefix="rc-target-check-") as temporary:
            folder = Path(temporary)
            (folder / "arm_math.h").write_text(ARM_MATH_SHIM, encoding="utf-8")
            for label, content in (("current", source), ("active_rl", active)):
                harness = folder / (label + ".c")
                harness.write_text(content + CHECKS, encoding="utf-8")
                for machine in (0, 1):
                    with self.subTest(range=label, machine=machine):
                        executable = folder / (label + str(machine) + ".exe")
                        command = [compiler, "-std=c99", "-Wall", "-Wextra", "-Werror",
                                   f"-DMACHINE_DEFAULT={machine}", "-I" + str(folder),
                                   "-I" + str(ROOT / "imcalib/Algorithm"), "-I" + str(ROOT / "imcalib/user-lib"),
                                   str(harness), *[str(ROOT / p) for p in dependencies], "-lm", "-o", str(executable)]
                        result = subprocess.run(command, capture_output=True, text=True, env=environment)
                        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                        result = subprocess.run([str(executable)], capture_output=True, text=True, env=environment)
                        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
