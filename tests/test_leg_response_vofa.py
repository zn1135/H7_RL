import contextlib
import csv
import importlib.util
import io
import json
import math
import struct
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch


SCRIPT = Path(__file__).resolve().parents[1] / "tools" / "leg_response_vofa_decode.py"
SPEC = importlib.util.spec_from_file_location("leg_response_vofa_decode", SCRIPT)
VOFA = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(VOFA)


def frame(time_ms=100, side=0, flags=1023):
    return tuple(float(value) for value in
                 (time_ms, side, 0.2, -0.4, 0.24, 3.1, -3.1, -0.8,
                  0.7, 0.25, 0.1, 1.5, -2.0, 1.3, -1.8, flags,
                  0.4, -0.5, 1.25, -2.5, 0.3, -0.4, 2.0, 4.0))


def legacy_frame(time_ms=100):
    return tuple(float(value) for value in
                 (time_ms, 0, 0.2, -0.4, 1.5, -2.0, 0.3, -0.8, 0.25, 63))


class LegResponseVofaTest(unittest.TestCase):
    def test_csv_accepts_twentyfour_channels_with_optional_host_time(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "capture.csv"
            with path.open("w", newline="", encoding="utf-8-sig") as stream:
                writer = csv.writer(stream)
                writer.writerow(["Time"] + ["ch{}".format(i) for i in range(24)])
                writer.writerow(["12:00:00"] + list(frame()))
                writer.writerow(frame(102, side=1))
            values, ignored = VOFA.read_csv(path)
            self.assertEqual(values, [frame(), frame(102, side=1)])
            self.assertEqual(ignored, 1)

    def test_csv_rejects_other_channel_layouts_even_if_suffix_looks_valid(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "old-layout.csv"
            with path.open("w", newline="") as stream:
                writer = csv.writer(stream)
                writer.writerow([0.0] * 8 + list(frame()))
                writer.writerow(["12:00:00"] + [0.0] * 8 + list(frame()))
                writer.writerow([0.0] * 2 + list(frame()))
                writer.writerow(legacy_frame())
            values, ignored = VOFA.read_csv(path)
            self.assertEqual(values, [])
            self.assertEqual(ignored, 4)

    def test_output_validity_requires_can_enqueue_and_separates_observation(self):
        rows, summary = VOFA.decode([frame(flags=991), frame(102, flags=23),
                                     frame(104), frame(106, flags=1022),
                                     frame(108, flags=2047), frame(110, flags=3071)])
        self.assertEqual([row["valid"] for row in rows], [0, 0, 1, 0, 0, 0])
        self.assertEqual([row["target_tracking_valid"] for row in rows], [0, 0, 1, 0, 0, 0])
        self.assertEqual([row["observation_valid"] for row in rows], [1, 1, 1, 0, 1, 1])
        self.assertEqual(summary["invalid_frames"], 5)

    def test_channel_mapping_and_wrapped_target_error(self):
        rows, _ = VOFA.decode([frame()])
        row = rows[0]
        fields = ("vx_cmd_m_s", "yaw_rate_cmd_rad_s", "height_cmd_m",
                  "virtual_thigh_target_rad", "virtual_thigh_measured_rad",
                  "virtual_shank_target_rad", "virtual_shank_measured_rad",
                  "virtual_leg_length_m", "virtual_leg_angle_rad",
                  "hip_front_command_nm", "hip_rear_command_nm",
                  "hip_front_feedback_nm", "hip_rear_feedback_nm")
        self.assertEqual([row[field] for field in fields], list(frame()[2:15]))
        self.assertAlmostEqual(row["virtual_thigh_error_rad"], 6.2 - 2 * math.pi)
        self.assertAlmostEqual(row["virtual_shank_error_rad"], -1.5)
        reversed_angles = list(frame())
        reversed_angles[7:9] = [-3.1, 3.1]
        rows, _ = VOFA.decode([tuple(reversed_angles)])
        self.assertAlmostEqual(rows[0]["virtual_shank_error_rad"], 2 * math.pi - 6.2)

    def test_physical_fields_and_jacobian_rate_use_front_rear_order(self):
        rows, summary = VOFA.decode([frame()])
        fields = ("hip_front_pos_zero_rad", "hip_rear_pos_zero_rad",
                  "hip_front_vel_rad_s", "hip_rear_vel_rad_s",
                  "virtual_shank_jac_front", "virtual_shank_jac_rear",
                  "hip_front_feedback_age_ms", "hip_rear_feedback_age_ms")
        self.assertEqual([rows[0][field] for field in fields], list(frame()[16:24]))
        self.assertAlmostEqual(rows[0]["kinematic_shank_rate_rad_s"], 0.3 * 1.25 + -0.4 * -2.5)
        self.assertIn("not an independent measurement", summary["kinematic_rate_semantics"])
        for flags in (7, 6, 5, 3):
            with self.subTest(flags=flags):
                rows, _ = VOFA.decode([frame(flags=flags)])
                if flags == 7:
                    self.assertIsNotNone(rows[0]["kinematic_shank_rate_rad_s"])
                else:
                    self.assertIsNone(rows[0]["kinematic_shank_rate_rad_s"])

    def test_unknown_and_stale_feedback_ages_are_retained_without_becoming_flags(self):
        values = list(frame(flags=1020))
        values[22:24] = [-1.0, 25.0]
        self.assertTrue(VOFA.valid_frame(values))
        rows, _ = VOFA.decode([tuple(values)])
        self.assertEqual(rows[0]["valid_flags"], 1020)
        self.assertEqual(rows[0]["hip_front_feedback_age_ms"], -1.0)
        self.assertEqual(rows[0]["hip_rear_feedback_age_ms"], 25.0)
        self.assertIsNone(rows[0]["kinematic_shank_rate_rad_s"])

    def test_invalid_targets_or_state_leave_error_columns_empty(self):
        values = [frame(flags=1023 ^ (1 << bit)) for bit in range(10)]
        values += [frame(flags=1023 | (1 << bit)) for bit in (10, 11)]
        for index, value in enumerate(values):
            with self.subTest(changed_bit=index):
                rows, _ = VOFA.decode([value])
                self.assertFalse(rows[0]["target_tracking_valid"])
                self.assertIsNone(rows[0]["virtual_thigh_error_rad"])
                self.assertIsNone(rows[0]["virtual_shank_error_rad"])

    def test_duplicate_and_rollback_timestamps_are_invalid_until_high_watermark(self):
        rows, summary = VOFA.decode([frame(t) for t in (100, 100, 98, 100, 102)])
        self.assertEqual([row["valid"] for row in rows], [1, 0, 0, 0, 1])
        self.assertEqual([row["timestamp_valid"] for row in rows], [1, 0, 0, 0, 1])
        self.assertEqual(summary["nonincreasing_timestamp_frames"], 3)
        self.assertEqual(summary["invalid_frames"], 3)
        self.assertIsNone(rows[1]["virtual_thigh_error_rad"])

    def test_side_change_marks_boundary_and_prevents_single_side_recording(self):
        rows, summary = VOFA.decode([frame(), frame(102, 1), frame(104, 1)])
        self.assertEqual([row["side_changed"] for row in rows], [0, 1, 0])
        self.assertEqual([row["valid"] for row in rows], [1, 0, 1])
        self.assertEqual(summary["side_change_count"], 1)
        self.assertFalse(summary["single_side_recording"])
        self.assertEqual(summary["test_sides"], ["left", "right"])
        self.assertIsNone(rows[1]["virtual_shank_error_rad"])

    def test_raw_noise_invalid_frame_and_truncated_tail(self):
        packed = lambda values: struct.pack("<24f", *values) + VOFA.TAIL
        first = struct.unpack("<24f", struct.pack("<24f", *frame()))
        second = struct.unpack("<24f", struct.pack("<24f", *frame(102)))
        invalid = list(frame())
        invalid[1] = 7.0
        blob = b"bad" + packed(first) + packed(invalid) + packed(second) + b"partial"
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "capture.bin"
            path.write_bytes(blob)
            values, ignored = VOFA.read_raw(path)
            self.assertEqual(values, [first, second])
            self.assertEqual(ignored, len(blob) - 2 * VOFA.FRAME_BYTES)

    def test_invalid_nonfinite_fractional_and_unknown_flags(self):
        for index, value in ((0, -1.0), (0, 1.5), (1, 2.0),
                             (2, float("nan")), (8, float("inf")),
                             (15, 4096.0), (15, 1.5)):
            with self.subTest(index=index, value=value):
                values = list(frame())
                values[index] = value
                self.assertFalse(VOFA.valid_frame(values))

    def test_legacy_layout_requires_explicit_selection_for_csv_raw_and_cli(self):
        with tempfile.TemporaryDirectory() as directory, contextlib.redirect_stdout(io.StringIO()):
            root = Path(directory)
            source = root / "legacy.csv"
            with source.open("w", newline="") as stream:
                csv.writer(stream).writerow(legacy_frame())
            self.assertEqual(VOFA.read_csv(source), ([], 1))
            values, ignored = VOFA.read_csv(source, layout="legacy10")
            self.assertEqual((values, ignored), ([legacy_frame()], 0))
            rows, summary = VOFA.decode(values, layout="legacy10")
            self.assertEqual(rows[0]["hip_front_pos_zero_rad"], 0.2)
            self.assertTrue(rows[0]["valid"])
            self.assertNotIn("virtual_thigh_error_rad", rows[0])
            raw = root / "legacy.bin"
            raw.write_bytes(struct.pack("<10f", *legacy_frame()) + VOFA.TAIL)
            self.assertEqual(VOFA.read_raw(raw)[0], [])
            self.assertEqual(len(VOFA.read_raw(raw, layout="legacy10")[0]), 1)
            output = root / "decoded"
            with patch.object(sys, "argv", [str(SCRIPT), str(source), str(output), "--layout", "legacy10"]):
                self.assertEqual(VOFA.main(), 0)
            manifest = json.loads((output / "manifest.json").read_text(encoding="utf-8"))
            self.assertEqual(manifest["layout"], "legacy10")
            self.assertEqual(manifest["channels"], 10)

    def test_legacy_validity_timestamp_and_side_checks_are_preserved(self):
        values = [list(legacy_frame(time_ms)) for time_ms in (100, 102, 104, 104, 106)]
        values[0][-1] = 31.0
        values[1][-1] = 27.0
        values[4][1] = 1.0
        rows, summary = VOFA.decode([tuple(value) for value in values], layout="legacy10")
        self.assertEqual([row["valid"] for row in rows], [0, 0, 1, 0, 0])
        self.assertEqual([row["observation_valid"] for row in rows], [1, 1, 1, 0, 1])
        self.assertEqual(summary["nonincreasing_timestamp_frames"], 1)
        self.assertEqual(summary["side_change_count"], 1)

    def test_invalid_target_errors_are_empty_in_exported_csv(self):
        with tempfile.TemporaryDirectory() as directory, contextlib.redirect_stdout(io.StringIO()):
            root = Path(directory)
            source = root / "capture.csv"
            with source.open("w", newline="") as stream:
                writer = csv.writer(stream)
                writer.writerow(frame())
                writer.writerow(frame(102, flags=23))
            output = root / "decoded"
            with patch.object(sys, "argv", [str(SCRIPT), str(source), str(output)]):
                self.assertEqual(VOFA.main(), 0)
            with (output / "leg_response.csv").open(newline="") as stream:
                rows = list(csv.DictReader(stream))
            self.assertNotEqual(rows[0]["virtual_thigh_error_rad"], "")
            self.assertEqual(rows[1]["virtual_thigh_error_rad"], "")
            self.assertEqual(rows[1]["virtual_shank_error_rad"], "")
            self.assertEqual(rows[1]["target_tracking_valid"], "0")

    def test_intermediate_sixteen_channel_layout_is_rejected(self):
        values = legacy_frame() + (0.25, 0.0, 0.0, 0.0, 0.0, 0.0)
        self.assertFalse(VOFA.valid_frame(values))
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "intermediate.csv"
            with source.open("w", newline="") as stream:
                csv.writer(stream).writerow(values)
            self.assertEqual(VOFA.read_csv(source), ([], 1))

    def test_configuration_names_cannot_overwrite_capture_or_generated_files(self):
        for name in ("raw.bin", "vofa_plus_export.csv", "leg_response.csv", "manifest.json"):
            with self.subTest(name=name), tempfile.TemporaryDirectory() as directory, \
                    contextlib.redirect_stdout(io.StringIO()):
                root = Path(directory)
                capture = root / ("capture.csv" if name == "vofa_plus_export.csv" else "capture.bin")
                raw_copy = "vofa_plus_export.csv" if capture.suffix == ".csv" else "raw.bin"
                if capture.suffix == ".csv":
                    with capture.open("w", newline="") as stream:
                        csv.writer(stream).writerow(frame())
                else:
                    capture.write_bytes(struct.pack("<24f", *frame()) + VOFA.TAIL)
                config = root / "settings" / name
                config.parent.mkdir()
                config.write_text("configuration evidence", encoding="utf-8")
                output = root / "decoded"
                args = [str(SCRIPT), str(capture), str(output),
                        "--firmware-fingerprint", "test-build", "--config", str(config)]
                with patch.object(sys, "argv", args):
                    self.assertEqual(VOFA.main(), 0)
                manifest = json.loads((output / "manifest.json").read_text(encoding="utf-8"))
                self.assertEqual((output / raw_copy).read_bytes(), capture.read_bytes())
                self.assertEqual(manifest["input_sha256"], VOFA.sha256(output / raw_copy))
                self.assertEqual(manifest["configuration"], "configuration/" + name)
                self.assertEqual(manifest["configuration_sha256"], VOFA.sha256(config))
                self.assertEqual(manifest["firmware_fingerprint"], "test-build")
                self.assertEqual(manifest["layout"], "diagnostic24")
                self.assertEqual(manifest["channels"], 24)
                self.assertIn("firmware fingerprint", manifest["decode"]["layout_identity_limit"])
                self.assertEqual((output / manifest["configuration"]).read_bytes(), config.read_bytes())
                with (output / "leg_response.csv").open(newline="") as stream:
                    self.assertEqual(len(list(csv.DictReader(stream))), 1)

    def test_existing_output_is_not_overwritten(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            capture = root / "capture.bin"
            capture.write_bytes(struct.pack("<24f", *frame()) + VOFA.TAIL)
            with patch.object(sys, "argv", [str(SCRIPT), str(capture), str(root)]):
                with self.assertRaises(FileExistsError):
                    VOFA.main()


if __name__ == "__main__":
    unittest.main()
