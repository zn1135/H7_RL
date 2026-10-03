"""Host regressions for the Leg3 force model and stateless torque adapter.

CMSIS sin/cos/sqrt are bound to libm in a temporary copy of the source. These
tests verify formula/call-chain behavior, not CMSIS lookup-table accuracy.
Set LEG3_SPRING_SOURCE to compare against the external original C file; the
fallback below preserves that file's finite-input arithmetic for portable runs.
"""

from pathlib import Path
import os
import re
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
REFERENCE_C = r"""
float Leg_SpringF(float L0)
{
    float sqrt_tmp1;
    float theta;
    L0 = fminf(fmaxf(L0, 0.05F), 0.45F);
    theta = acosf((0.1066F - L0 * L0) / 0.105F);
    arm_sqrt_f32(fmaxf(0.0473512858F -
        0.0206288453F * arm_cos_f32(theta - 0.213803F), 0.0F), &sqrt_tmp1);
    return 150.0F *
        (0.0103144227F * arm_sin_f32(theta - 0.213803F) * L0 /
        fmaxf(0.0525F * arm_sin_f32(theta) * sqrt_tmp1, 1.0E-6F));
}
"""
SHIM = r"""
#ifndef TEST_ARM_MATH_H
#define TEST_ARM_MATH_H
#include <math.h>
#define arm_sin_f32 sinf
#define arm_cos_f32 cosf
static inline int arm_sqrt_f32(float value, float *result)
{
    *result = sqrtf(value);
    return 0;
}
#endif
"""
HARNESS = r"""
#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "gas_spring.h"
#include "machine_config.h"
float Reference_Leg_SpringF(float L0);

static void near(float actual, float expected)
{
    if (!isfinite(actual) || !isfinite(expected) || fabsf(actual - expected) > 0.0001f)
    {
        fprintf(stderr, "actual=%g expected=%g\n", actual, expected);
        assert(0);
    }
}

static leg_state_t make_leg(float q_front, float q_back)
{
    leg_state_t leg;
    memset(&leg, 0, sizeof(leg));
    leg.config.configured = 1u;
    leg.config.lu = 0.21f;
    leg.config.lg = 0.25f;
    leg.input.hip_f = q_front;
    leg.input.hip_b = q_back;
    assert(Leg_Solve(&leg) && leg.output.force_valid);
    return leg;
}

static void assert_zero(const float raw[4])
{
    unsigned i;
    for (i = 0u; i < 4u; i++)
    {
        assert(raw[i] == 0.0f);
    }
}

static void model_reference_parity(void)
{
    const float extras[] = {-FLT_MAX, -1.0f, -0.0f, 0.0f, FLT_MIN,
        0.05f, 0.14f, 0.15f, 0.2f, 0.25f, 0.3f, 0.34f, 0.45f, 1.0f, FLT_MAX};
    float input, actual, reference;
    unsigned i;
    for (i = 0u; i <= 10000u; i++)
    {
        input = -0.1f + (float)i * 0.00007f;
        actual = Leg_SpringF(input);
        reference = Reference_Leg_SpringF(input);
        assert(memcmp(&actual, &reference, sizeof(actual)) == 0);
    }
    for (i = 0u; i < sizeof(extras) / sizeof(extras[0]); i++)
    {
        actual = Leg_SpringF(extras[i]);
        reference = Reference_Leg_SpringF(extras[i]);
        assert(memcmp(&actual, &reference, sizeof(actual)) == 0);
    }
}

static void model_values_and_boundaries(void)
{
    const float lengths[6] = {0.14f, 0.15f, 0.2f, 0.25f, 0.3f, 0.34f};
    const float forces[6] = {16.29536247f, 18.12935522f, 26.80774990f,
        34.66048483f, 41.77892339f, 47.16802234f};
    unsigned i;
    for (i = 0u; i < 6u; i++)
    {
        near(Leg_SpringF(lengths[i]), forces[i]);
    }
    assert(isnan(Leg_SpringF(NAN)));
    assert(isnan(Leg_SpringF(INFINITY)));
    assert(isnan(Leg_SpringF(-INFINITY)));
    assert(Leg_SpringF(-1.0f) == Leg_SpringF(0.05f));
    assert(Leg_SpringF(0.04f) == Leg_SpringF(0.05f));
    assert(Leg_SpringF(0.46f) == Leg_SpringF(0.45f));
    assert(Leg_SpringF(100.0f) == Leg_SpringF(0.45f));
}

static void force_only_and_alias(void)
{
    leg_state_t left = make_leg(2.0f, 0.4f);
    leg_state_t right = make_leg(2.2f, 0.65f);
    const leg_state_t *leg[2] = {&left, &right};
    const float base[4] = {100.0f, -100.0f, 120.0f, -120.0f};
    float raw[4], alias[4], delta[2], force, a, b, c, d, det;
    unsigned side, i;
    assert(Gas_Spring_Apply(&left, &right, base, raw));
    for (side = 0u; side < 2u; side++)
    {
        force = -Reference_Leg_SpringF(leg[side]->output.virtual_leg_length);
        a = leg[side]->output.force_map[0][0];
        b = leg[side]->output.force_map[0][1];
        c = leg[side]->output.force_map[1][0];
        d = leg[side]->output.force_map[1][1];
        det = a * d - b * c;
        delta[0] = raw[2u * side] - base[2u * side];
        delta[1] = raw[2u * side + 1u] - base[2u * side + 1u];
        near((d * delta[0] - b * delta[1]) / det, force);
        near((-c * delta[0] + a * delta[1]) / det, 0.0f);
        near(raw[2u * side], base[2u * side] + a * force);
        near(raw[2u * side + 1u], base[2u * side + 1u] + c * force);
    }
    assert(fabsf(raw[0]) > 40.0f);
    memcpy(alias, base, sizeof(alias));
    assert(Gas_Spring_Apply(&left, &right, alias, alias));
    for (i = 0u; i < 4u; i++)
    {
        assert(raw[i] == alias[i]);
    }
}

static void enabled_failures_are_atomic(void)
{
    leg_state_t left, right;
    float base[4], raw[4], force;
    unsigned scenario, i;
    for (scenario = 0u; scenario < 8u; scenario++)
    {
        left = make_leg(2.0f, 0.4f);
        right = make_leg(2.2f, 0.65f);
        for (i = 0u; i < 4u; i++)
        {
            base[i] = raw[i] = 3.0f;
        }
        switch (scenario)
        {
        case 0: right.output.valid = 0u; break;
        case 1: right.output.force_valid = 0u; break;
        case 2: right.output.virtual_leg_length = NAN; break;
        case 3: right.output.virtual_leg_length = INFINITY; break;
        case 4: right.output.force_map[1][0] = NAN; break;
        case 5: right.output.force_map[0][1] = INFINITY; break;
        case 6: base[3] = NAN; break;
        default:
            right.output.virtual_leg_length = 0.25f;
            right.output.force_map[1][0] = -FLT_MAX / 128.0f;
            base[3] = FLT_MAX;
            force = -Leg_SpringF(right.output.virtual_leg_length);
            assert(isfinite(force * right.output.force_map[1][0]));
            break;
        }
        assert(!Gas_Spring_Apply(&left, &right, base, raw));
        assert_zero(raw);
        memcpy(raw, base, sizeof(raw));
        assert(!Gas_Spring_Apply(&left, &right, raw, raw));
        assert_zero(raw);
    }
    assert(!Gas_Spring_Apply(&left, NULL, base, raw));
    assert_zero(raw);
}

static void disabled_or_small_bypass(void)
{
    const float base[4] = {-0.0f, 1.25f, -2.5f, 3.75f};
    float raw[4];
    assert(Gas_Spring_Apply(NULL, NULL, base, raw));
    assert(memcmp(base, raw, sizeof(raw)) == 0);
    assert(Gas_Spring_Apply(NULL, NULL, raw, raw));
    assert(memcmp(base, raw, sizeof(raw)) == 0);
}

static void invalid_arguments_and_base(void)
{
    const float base[4] = {0.0f, 1.0f, 2.0f, 3.0f};
    float raw[4] = {1.0f, 1.0f, 1.0f, 1.0f};
    assert(!Gas_Spring_Apply(NULL, NULL, NULL, raw));
    assert_zero(raw);
    assert(!Gas_Spring_Apply(NULL, NULL, base, NULL));
    raw[3] = INFINITY;
    assert(!Gas_Spring_Apply(NULL, NULL, raw, raw));
    assert_zero(raw);
}

int main(int argc, char **argv)
{
    assert(argc == 2);
    switch (atoi(argv[1]))
    {
    case 0: model_reference_parity(); break;
    case 1: model_values_and_boundaries(); break;
    case 2: force_only_and_alias(); break;
    case 3: enabled_failures_are_atomic(); break;
    case 4: disabled_or_small_bypass(); break;
    case 5: invalid_arguments_and_base(); break;
    default: assert(0); break;
    }
    return 0;
}
"""


class GasSpringTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = os.environ.get("CC") or shutil.which("gcc")
        if compiler is None:
            raise unittest.SkipTest("no native C compiler; set CC to host gcc")
        cls.temporary = tempfile.TemporaryDirectory(prefix="gas-spring-leg3-")
        cls.addClassCleanup(cls.temporary.cleanup)
        folder = Path(cls.temporary.name)
        cls.environment = os.environ.copy()
        resolved = shutil.which(compiler) or compiler
        cls.environment["PATH"] = str(Path(resolved).resolve().parent) + os.pathsep + cls.environment.get("PATH", "")
        (folder / "arm_math.h").write_text(SHIM, encoding="utf-8")
        gas_source = (ROOT / "imcalib/Algorithm/gas_spring.c").read_text(encoding="utf-8")
        gas_source = gas_source.replace(
            '#include "../../Drivers/CMSIS/DSP/Include/arm_math.h"', '#include "arm_math.h"')
        (folder / "gas_spring_host.c").write_text(gas_source, encoding="utf-8")
        external = os.environ.get("LEG3_SPRING_SOURCE")
        reference = Path(external).read_text(encoding="utf-8") if external else REFERENCE_C
        reference = re.sub(r"^#include[^\n]*\n", "", reference, flags=re.M)
        reference = re.sub(r"\bLeg_SpringF\(", "Reference_Leg_SpringF(", reference)
        (folder / "reference.c").write_text('#include "arm_math.h"\n' + reference, encoding="utf-8")
        (folder / "harness.c").write_text(HARNESS, encoding="utf-8")
        cls.executables = {}
        variants = {
            "default_big": ["-DMACHINE_DEFAULT=0"],
            "off_big": ["-DMACHINE_DEFAULT=0", "-DGAS_SPRING_COMP_ENABLE=0"],
            "on_big": ["-DMACHINE_DEFAULT=0", "-DGAS_SPRING_COMP_ENABLE=1"],
            "on_small": ["-DMACHINE_DEFAULT=1", "-DGAS_SPRING_COMP_ENABLE=1"],
        }
        for name, defines in variants.items():
            executable = folder / (name + (".exe" if os.name == "nt" else ""))
            result = subprocess.run([
                compiler, "-std=c99", "-Wall", "-Wextra", "-Werror", "-DLEG_TRIG_LIBM=1",
                *defines, "-I", str(folder), "-I", str(ROOT / "imcalib/Algorithm"),
                "-I", str(ROOT / "imcalib/user-lib"), str(folder / "harness.c"),
                str(folder / "gas_spring_host.c"), str(folder / "reference.c"),
                str(ROOT / "imcalib/Algorithm/leg_solver.c"), "-lm", "-o", str(executable),
            ], capture_output=True, text=True, env=cls.environment)
            if result.returncode:
                raise AssertionError(result.stdout + result.stderr)
            cls.executables[name] = executable

    def run_case(self, variant, case):
        result = subprocess.run([str(self.executables[variant]), str(case)], capture_output=True,
                                text=True, env=self.environment, timeout=15)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_finite_model_matches_original_float_arithmetic(self):
        self.run_case("on_big", 0)

    def test_model_reference_values_clamp_and_nonfinite(self):
        self.run_case("on_big", 1)

    def test_enabled_big_compensates_only_force_without_limiting_and_supports_alias(self):
        self.run_case("on_big", 2)

    def test_enabled_big_invalid_geometry_finite_checks_and_atomic_failure(self):
        self.run_case("on_big", 3)

    def test_default_enabled_explicit_disabled_and_small_machine_bypass(self):
        self.run_case("default_big", 2)
        for variant in ("off_big", "on_small"):
            with self.subTest(variant=variant):
                self.run_case(variant, 4)

    def test_all_variants_reject_invalid_base_and_output_arguments(self):
        for variant in self.executables:
            with self.subTest(variant=variant):
                self.run_case(variant, 5)


if __name__ == "__main__":
    unittest.main()
