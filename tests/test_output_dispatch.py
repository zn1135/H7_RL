"""Verify the production output gate after removal of side filtering."""

from pathlib import Path
import os
import re
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]


def production_function(source, name):
    match = re.search(r"^static void " + name + r"\([^)]*\)\s*\{", source, re.M)
    if match is None:
        raise AssertionError("missing production function: " + name)
    depth, end = 1, match.end()
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[match.start():end]


HARNESS = r"""
#include <assert.h>
#include <stdint.h>
#include <string.h>
#include "torque_output.h"
typedef enum { HAL_OK, HAL_ERROR } HAL_StatusTypeDef;
static uint8_t torque_output_enabled, output_debug_dm_sent, output_debug_dji_sent;
static float rl_output_dm_cmd_nm[4], rl_output_wheel_cmd_nm[2];
static float sent_dm[4], sent_wheel[2];
static HAL_StatusTypeDef dm_result, wheel_result;
static HAL_StatusTypeDef Dm_Send_Zero(void)
{
    memset(sent_dm, 0, sizeof(sent_dm)); return dm_result;
}
static HAL_StatusTypeDef Dji_All_Stop(void)
{
    memset(sent_wheel, 0, sizeof(sent_wheel)); return wheel_result;
}
static HAL_StatusTypeDef Dm_Send_Torque(const float *values)
{
    memcpy(sent_dm, values, sizeof(sent_dm)); return dm_result;
}
static HAL_StatusTypeDef Dji_Send_Wheel_Torque(float left, float right)
{
    sent_wheel[0] = left; sent_wheel[1] = right; return wheel_result;
}
"""

CHECK = r"""
int main(void)
{
    torque_output_t torque = {{1.25f, -2.5f, 3.75f, -4}, {0.2f, -0.35f}, 1};
    unsigned i, enabled, valid;
    for (enabled = 0; enabled < 2; enabled++)
    {
        for (valid = 0; valid < 2; valid++)
        {
            torque_output_enabled = enabled; torque.valid = valid;
            output_dispatch(&torque);
            for (i = 0; i < 4; i++)
            {
                assert(sent_dm[i] == ((enabled && valid) ? torque.dm[i] : 0));
                assert(rl_output_dm_cmd_nm[i] == sent_dm[i]);
            }
            for (i = 0; i < 2; i++)
            {
                assert(sent_wheel[i] == ((enabled && valid) ? torque.dji[i] : 0));
                assert(rl_output_wheel_cmd_nm[i] == sent_wheel[i]);
            }
            assert(output_debug_dm_sent && output_debug_dji_sent);
        }
    }
    torque_output_enabled = torque.valid = 1;
    dm_result = HAL_ERROR; output_dispatch(&torque);
    assert(!output_debug_dm_sent && output_debug_dji_sent);
    wheel_result = HAL_ERROR; output_dispatch(&torque);
    assert(!output_debug_dm_sent && !output_debug_dji_sent);
    return 0;
}
"""


class OutputDispatchTest(unittest.TestCase):
    def test_full_output_and_zero_output_gates(self):
        compiler = os.environ.get("CC") or shutil.which("gcc")
        if compiler is None:
            self.skipTest("no native C compiler; set CC to host gcc")
        source = (ROOT / "imcalib/task/task_actuation.c").read_text(encoding="utf-8")
        environment = os.environ.copy()
        resolved = shutil.which(compiler) or compiler
        environment["PATH"] = str(Path(resolved).resolve().parent) + os.pathsep + environment.get("PATH", "")
        with tempfile.TemporaryDirectory(prefix="output-dispatch-") as temporary:
            folder = Path(temporary)
            (folder / "dm.h").write_text("#define DM_MOTOR_NUM 4\n", encoding="utf-8")
            (folder / "dji.h").write_text(
                "#define DJI_MOTOR_NUM 2\n#define DJI_MOTOR_WHEEL_LFT 0\n#define DJI_MOTOR_WHEEL_RGT 1\n",
                encoding="utf-8")
            host = folder / "output_dispatch.c"
            host.write_text(HARNESS + production_function(source, "output_dispatch") + CHECK,
                            encoding="utf-8")
            executable = folder / ("output_dispatch.exe" if os.name == "nt" else "output_dispatch")
            command = [compiler, "-std=c99", "-Wall", "-Wextra", "-Werror",
                       "-I" + str(folder), "-I" + str(ROOT / "imcalib/Algorithm"),
                       str(host), "-o", str(executable)]
            result = subprocess.run(command, capture_output=True, text=True, env=environment)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            result = subprocess.run([str(executable)], capture_output=True, text=True, env=environment)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


if __name__ == "__main__":
    unittest.main()
