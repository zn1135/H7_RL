"""Check the production DM receive-age gate and fault checks for both machines."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

from test_gas_spring_integration import ROOT, read


HARNESS = r"""
#include <assert.h>
#include <limits.h>
#include "dm.h"
#include "machine_config.h"
dm_motor_feedback_t dm_motor_feedback[DM_MOTOR_NUM];
static uint32_t tick;
uint32_t HAL_GetTick(void) { return tick; }
"""

CHECK = r"""
int main(void)
{
    unsigned i, err;
    uint32_t expected = MACHINE_DM_OFFLINE_MS;
    assert(expected >= 10u);
    assert(DM_OFFLINE_MS == expected);
    tick = 100;
    for (i = 0; i < DM_MOTOR_NUM; i++)
    {
        dm_motor_feedback[i].last_rx_tick = 100;
        dm_motor_feedback[i].err_raw = 1;
        assert(!Dm_Is_Online(i) && !Dm_Is_Enabled(i));
        dm_motor_feedback[i].rx_seen = 1;
    }
    tick = 111;
    assert(Dm_Is_Online(0) == (MACHINE_DEFAULT == MACHINE_ID_BIG_WHEELLEG));
    for (i = 0; i < DM_MOTOR_NUM; i++)
    {
        tick = 100 + expected - 1; assert(Dm_Is_Online(i) && Dm_Is_Enabled(i));
        tick = 100 + expected; assert(Dm_Is_Online(i) && Dm_Is_Enabled(i));
        tick++; assert(!Dm_Is_Online(i) && !Dm_Is_Enabled(i));
        dm_motor_feedback[i].last_rx_tick = tick;
        assert(Dm_Is_Online(i));
        dm_motor_feedback[i].err_raw = 0; assert(!Dm_Is_Enabled(i));
        for (err = 0; err < 16; err++)
        {
            dm_motor_feedback[i].err_raw = err;
            assert(Dm_Has_Fault(i) == (err >= 8 && err <= 14));
        }
        dm_motor_feedback[i].err_raw = 1;
        dm_motor_feedback[i].last_rx_tick = UINT32_MAX - 3;
        tick = expected - 4; assert(Dm_Is_Online(i));
        tick++; assert(!Dm_Is_Online(i));
    }
    assert(!Dm_Is_Online(DM_MOTOR_NUM) && !Dm_Is_Enabled(DM_MOTOR_NUM));
    assert(!Dm_Has_Fault(DM_MOTOR_NUM));
    return 0;
}
"""


class DmOfflineGateTest(unittest.TestCase):
    def test_feedback_timeout_boundaries_and_fault_codes(self):
        compiler = os.environ.get("CC") or shutil.which("gcc")
        if compiler is None:
            self.skipTest("no host compiler; set CC")
        env = os.environ.copy()
        env["PATH"] = str(Path(compiler).parent) + os.pathsep + env.get("PATH", "")
        source = read("imcalib/user-lib/dm.c")
        functions = []
        for name in ("Dm_Is_Online", "Dm_Is_Enabled", "Dm_Has_Fault"):
            start = source.index("bool " + name + "(")
            end = source.index("{", start) + 1
            depth = 1
            while depth:
                depth += (source[end] == "{") - (source[end] == "}")
                end += 1
            functions.append(source[start:end])
        with tempfile.TemporaryDirectory(prefix="dm-offline-") as temporary:
            folder = Path(temporary)
            (folder / "main.h").write_text("#pragma once\n#include <stdint.h>\n"
                "typedef int HAL_StatusTypeDef;\n"
                "typedef struct { int unused; } FDCAN_HandleTypeDef;\n"
                "typedef struct { int unused; } FDCAN_TxHeaderTypeDef;\n", encoding="utf-8")
            (folder / "fdcan.h").write_text('#include "main.h"\n', encoding="utf-8")
            test = folder / "gate.c"
            test.write_text(HARNESS + "\n" + "\n".join(functions) + CHECK, encoding="utf-8")
            for machine in (None, 0, 1):
                with self.subTest(machine=machine):
                    exe = folder / (f"gate_{machine}.exe")
                    flags = [] if machine is None else [f"-DMACHINE_DEFAULT={machine}"]
                    result = subprocess.run([compiler, "-std=c99", "-Wall", "-Wextra", "-Werror",
                        *flags, "-I", str(folder), "-I", str(ROOT / "imcalib/user-lib"),
                        str(test), "-o", str(exe)], capture_output=True, text=True,
                        encoding="utf-8", errors="replace", env=env)
                    self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                    result = subprocess.run([str(exe)], capture_output=True, text=True,
                        encoding="utf-8", errors="replace", env=env)
                    self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
