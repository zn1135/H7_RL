"""Run the real pitch module and IMU task on both machine configurations."""

import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
HEADERS = {
    "robot_control.h": """
#include <stdint.h>
#include "imu_state.h"
extern imu_state_t imu_state;
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t mask);
""",
    "hi229.h": """
#include <stdint.h>
typedef struct {
    float quat[4], eul[3], gyr[3], acc[3];
    uint32_t ts;
} hi229_data_t;
void HI229_Process(void);
uint8_t HI229_Online(void);
hi229_data_t HI229_Snapshot(void);
""",
}

HARNESS = r"""
#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdint.h>
#include <string.h>
#include "pitch_world.h"
#include "Attitude_Algorithm.h"
#include "machine_config.h"
#include "hi229.h"

#define TEST_PI 3.14159265358979323846f
#define TEST_PITCH_IDX ATTITUDE_ROLL
imu_state_t imu_state;
static hi229_data_t sample;
static uint8_t online;
static uint32_t irq_mask, publications;
void imu_task_init(void);
void imu_task_body(void);

uint32_t __get_PRIMASK(void) { return irq_mask; }
void __disable_irq(void) { irq_mask = 1; }
void __set_PRIMASK(uint32_t mask)
{
    float expected;
    assert(irq_mask == 1);
    publications++;
    if (imu_state.pitch_world_valid)
    {
        assert(imu_state.online);
        assert(Pitch_World_Calc(imu_state.euler_rad[TEST_PITCH_IDX], imu_state.quat, &expected));
        assert(fabsf(imu_state.pitch_world - expected) < 2e-6f);
        assert(imu_state.last_timestamp_ms == sample.ts);
    }
    else
    {
        assert(imu_state.pitch_world == 0);
    }
    irq_mask = mask;
}
void HI229_Process(void) {}
uint8_t HI229_Online(void) { return online; }
hi229_data_t HI229_Snapshot(void) { return sample; }

static void euler_quat(float roll, float pitch, float yaw, float q[4])
{
    float cr = cosf(roll / 2), sr = sinf(roll / 2);
    float cp = cosf(pitch / 2), sp = sinf(pitch / 2);
    float cy = cosf(yaw / 2), sy = sinf(yaw / 2);
    q[0] = cr*cp*cy + sr*sp*sy;
    q[1] = sr*cp*cy - cr*sp*sy;
    q[2] = cr*sp*cy + sr*cp*sy;
    q[3] = cr*cp*sy - sr*sp*cy;
}

static void check_math(void)
{
    const float scales[] = {1, -1, 3, FLT_MAX / 4, FLT_MIN};
    float q[4], scaled[4], angle, expected, tilt, folded;
    int pitch, roll, yaw;
    unsigned i, j;
    for (pitch = -36; pitch <= 36; pitch++)
    {
        tilt = pitch * TEST_PI / 36;
        folded = atan2f(sinf(tilt), fabsf(cosf(tilt)));
        expected = tilt;
        for (roll = -2; roll <= 2; roll++)
        {
            for (yaw = -2; yaw <= 2; yaw++)
            {
                euler_quat(roll * .6f, tilt, yaw * 1.3f, q);
                for (j = 0; j < sizeof(scales)/sizeof(scales[0]); j++)
                {
                    for (i = 0; i < 4; i++) { scaled[i] = q[i] * scales[j]; }
                    assert(Pitch_World_Calc(folded, scaled, &angle));
                    assert(isfinite(angle));
                    assert(fabsf(remainderf(angle - expected, 2*TEST_PI)) < 3e-6f);
                    assert(angle >= -TEST_PI && angle < TEST_PI);
                }
            }
        }
    }
    euler_quat(0, 2*TEST_PI/3, 0, q);
    assert(Pitch_World_Calc(TEST_PI/3, q, &angle));
    assert(fabsf(angle - 2*TEST_PI/3) < 2e-6f);
    q[0] = 0; q[1] = 0; q[2] = 1; q[3] = 0;
    assert(Pitch_World_Calc(0, q, &angle) && angle == -TEST_PI);
    angle = 123;
    assert(!Pitch_World_Calc(NAN, q, &angle) && angle == 123);
    assert(!Pitch_World_Calc(INFINITY, q, &angle) && angle == 123);
    assert(!Pitch_World_Calc(-INFINITY, q, &angle) && angle == 123);
    assert(!Pitch_World_Calc(TEST_PI, q, &angle) && angle == 123);
    assert(!Pitch_World_Calc(-TEST_PI, q, &angle) && angle == 123);
    assert(!Pitch_World_Calc(0, NULL, &angle) && angle == 123);
    assert(!Pitch_World_Calc(0, q, NULL));
    memset(q, 0, sizeof(q));
    assert(!Pitch_World_Calc(0, q, &angle) && angle == 123);
    for (i = 0; i < 4; i++)
    {
        memset(q, 0, sizeof(q)); q[0] = 1; q[i] = NAN;
        assert(!Pitch_World_Calc(0, q, &angle) && angle == 123);
        q[i] = INFINITY;
        assert(!Pitch_World_Calc(0, q, &angle) && angle == 123);
        q[i] = -INFINITY;
        assert(!Pitch_World_Calc(0, q, &angle) && angle == 123);
    }
}

static void check_full_circle(void)
{
    const float cases[][2] = {
        {0, 0}, {60, 60}, {90, 90}, {120, 120}, {179, 179},
        {180, -180}, {181, -179}, {240, -120}, {270, -90},
        {300, -60}, {359, -1}, {360, 0}, {-120, -120},
        {-180, -180}, {-181, 179}, {-360, 0}
    };
    float q[4], angle, expected, folded, tilt;
    unsigned i;
    for (i = 0; i < sizeof(cases)/sizeof(cases[0]); i++)
    {
        tilt = cases[i][0] * TEST_PI/180;
        euler_quat(0, tilt, .7f, q);
        folded = atan2f(sinf(tilt), fabsf(cosf(tilt)));
        expected = cases[i][1] * TEST_PI/180;
        assert(Pitch_World_Calc(folded, q, &angle));
        assert(fabsf(remainderf(angle - expected, 2*TEST_PI)) < 2e-6f);
        assert(angle >= -TEST_PI && angle < TEST_PI);
    }
}

static void check_original_pitch(void)
{
    float q[4], angle;
    euler_quat(.8f, .5f, 1.2f, q);
    assert(Pitch_World_Calc(.2f, q, &angle) && angle == .2f);
    euler_quat(-.8f, 2*TEST_PI/3, -.7f, q);
    assert(Pitch_World_Calc(.2f, q, &angle));
    assert(fabsf(angle - (TEST_PI-.2f)) < 1e-6f);
    assert(Pitch_World_Calc(-.2f, q, &angle));
    assert(fabsf(angle - (-TEST_PI+.2f)) < 1e-6f);
}

static void set_sample_pose(float roll, float pitch)
{
    float q[4];
    float folded;
    unsigned i;
    memset(&sample, 0, sizeof(sample));
    euler_quat(roll, pitch, 1.2f, q);
    sample.quat[0] = q[0];
    for (i = 0; i < 3; i++)
    {
        sample.quat[machine->imu.quat_src[i]+1] = machine->imu.quat_sign[i] * q[i+1];
        sample.eul[i] = 10.0f * (i+1);
        sample.gyr[i] = 20.0f * (i+1);
        sample.acc[i] = .1f * (i+1);
    }
    folded = atan2f(sinf(pitch), fabsf(cosf(pitch)));
    sample.eul[machine->imu.eul_src[TEST_PITCH_IDX]] = folded * 180/TEST_PI
        / machine->imu.eul_sign[TEST_PITCH_IDX];
}

static void set_sample(float pitch)
{
    set_sample_pose(.4f, pitch);
}

static void check_task(void)
{
    const float invalid[] = {0, 1e-6f, NAN, INFINITY};
    float expected;
    uint32_t count;
    unsigned i;
    imu_task_init();
    assert(!imu_state.pitch_world_valid && imu_state.pitch_world == 0);
    online = 1;
    set_sample(.35f); sample.ts = 100;
    imu_task_body();
    assert(imu_state.online && imu_state.pitch_world_valid);
    assert(fabsf(imu_state.pitch_world - .35f) < 2e-6f && irq_mask == 0);
    for (i = 0; i < 3; i++)
    {
        expected = machine->imu.eul_sign[i] * sample.eul[machine->imu.eul_src[i]];
        assert(imu_state.euler_deg[i] == expected);
        assert(fabsf(imu_state.euler_rad[i] - expected*TEST_PI/180) < 2e-6f);
        expected = machine->imu.gyr_sign[i] * sample.gyr[machine->imu.gyr_src[i]];
        assert(fabsf(imu_state.gyro_rad_s[i] - expected*TEST_PI/180) < 2e-6f);
    }
    count = publications;
    sample.quat[0] = NAN;
    imu_task_body();
    assert(publications == count && imu_state.pitch_world_valid);
    assert(fabsf(imu_state.pitch_world - .35f) < 2e-6f);
    set_sample_pose(.8f, 2*TEST_PI/3); sample.ts = 150;
    imu_task_body();
    assert(imu_state.online && imu_state.pitch_world_valid);
    assert(fabsf(imu_state.pitch_world - 2*TEST_PI/3) < 2e-6f);
    set_sample(.25f); sample.ts = 160;
    sample.eul[machine->imu.eul_src[TEST_PITCH_IDX]] = NAN;
    imu_task_body();
    assert(imu_state.online && !imu_state.pitch_world_valid && imu_state.pitch_world == 0);
    for (i = 0; i < sizeof(invalid)/sizeof(invalid[0]); i++)
    {
        memset(sample.quat, 0, sizeof(sample.quat));
        sample.quat[0] = invalid[i]; sample.ts++;
        imu_task_body();
        assert(!imu_state.online && !imu_state.pitch_world_valid && imu_state.pitch_world == 0);
    }
    set_sample(-.2f); sample.ts = 200;
    irq_mask = 1;
    imu_task_body();
    assert(irq_mask == 1 && imu_state.pitch_world_valid);
    assert(fabsf(imu_state.pitch_world + .2f) < 2e-6f);
    set_sample(2*TEST_PI/3); sample.ts = 210;
    imu_task_body();
    assert(imu_state.online && imu_state.pitch_world_valid);
    assert(fabsf(imu_state.pitch_world - 2*TEST_PI/3) < 2e-6f);
    online = 0;
    imu_task_body();
    assert(irq_mask == 1 && !imu_state.online && !imu_state.pitch_world_valid && imu_state.pitch_world == 0);
    online = 1;
    irq_mask = 0;
    set_sample(.15f); sample.ts = 200;
    imu_task_body();
    assert(imu_state.online && imu_state.pitch_world_valid && irq_mask == 0);
    assert(fabsf(imu_state.pitch_world - .15f) < 2e-6f);
}

int main(int argc, char **argv)
{
    assert(argc == 2);
    if (strcmp(argv[1], "math") == 0) { check_math(); }
    else if (strcmp(argv[1], "full_circle") == 0) { check_full_circle(); }
    else if (strcmp(argv[1], "original_pitch") == 0) { check_original_pitch(); }
    else { assert(strcmp(argv[1], "task") == 0); check_task(); }
    return 0;
}
"""


class PitchWorldTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = os.environ.get("CC") or shutil.which("gcc")
        if not compiler:
            raise unittest.SkipTest("set CC to a host C compiler")
        cls.temp = tempfile.TemporaryDirectory(prefix="pitch-world-")
        cls.addClassCleanup(cls.temp.cleanup)
        folder = Path(cls.temp.name)
        cls.environment = os.environ.copy()
        cls.environment["PATH"] = str(Path(compiler).parent) + os.pathsep + cls.environment.get("PATH", "")
        for name, content in HEADERS.items():
            (folder / name).write_text(content, encoding="utf-8")
        harness = folder / "pitch_world_test.c"
        harness.write_text(HARNESS, encoding="utf-8")
        cls.executables = {}
        for machine in (0, 1):
            exe = folder / (f"pitch_world_{machine}" + (".exe" if os.name == "nt" else ""))
            sources = ["imcalib/task/pitch_world.c", "imcalib/task/task_imu.c",
                       "imcalib/Algorithm/Attitude_Algorithm.c", "imcalib/user-lib/machine_config.c"]
            result = subprocess.run(
                [compiler, "-std=c99", "-Wall", "-Wextra", "-Werror", f"-DMACHINE_DEFAULT={machine}",
                 "-I", str(folder), "-I", str(ROOT / "imcalib/task"),
                 "-I", str(ROOT / "imcalib/Algorithm"), "-I", str(ROOT / "imcalib/user-lib"),
                 str(harness), *[str(ROOT / p) for p in sources], "-lm", "-o", str(exe)],
                capture_output=True, text=True, encoding="utf-8", errors="replace", env=cls.environment)
            if result.returncode:
                raise AssertionError(result.stdout + result.stderr)
            cls.executables[machine] = exe

    def run_case(self, scenario):
        for machine, exe in self.executables.items():
            with self.subTest(machine=machine):
                result = subprocess.run([str(exe), scenario], capture_output=True, text=True,
                                        encoding="utf-8", errors="replace", env=self.environment)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_geometry_normalization_and_invalid_inputs(self):
        self.run_case("math")

    def test_full_circle_including_inverted_body_and_branch_crossing(self):
        self.run_case("full_circle")

    def test_uses_original_pitch_and_gravity_only_selects_branch(self):
        self.run_case("original_pitch")

    def test_real_imu_task_publication_deduplication_and_recovery(self):
        self.run_case("task")


if __name__ == "__main__":
    unittest.main()
