import csv
import importlib.util
import struct
import tempfile
import unittest
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / "tools" / "vofa_trace_decode.py"
SPEC = importlib.util.spec_from_file_location("vofa_trace_decode", SCRIPT)
TRACE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(TRACE)


def frame(kind, seq, time_us, flags, payload, last=0):
    values = [float(kind), float(seq), float(time_us & 0xFFFF),
              float((time_us >> 16) & 0xFFFF), float((time_us >> 32) & 0xFFFF),
              float(flags)] + list(payload) + [float(last)]
    assert len(values) == 32
    return tuple(values)


class VofaTraceTest(unittest.TestCase):
    def test_timestamp_and_history_reconstruction(self):
        t0 = (7 << 32) + 123456789
        obs0 = tuple(float(i) for i in range(25))
        obs1 = tuple(float(i + 100) for i in range(25))
        aux = (0.0,) * 25
        frames = [frame(0, 1, t0, 7, obs0, 123),
                  frame(1, 1, t0 + 80, 7, aux)]
        frames += [frame(2 + i, 1, t0, 7, obs0) for i in range(5)]
        frames += [frame(0, 2, t0 + 10000, 7, obs1, 124),
                   frame(1, 2, t0 + 10080, 7, aux)]
        samples, summary = TRACE.reconstruct(frames)
        self.assertEqual(summary["history_sync_count"], 1)
        self.assertEqual(summary["history_sync_mismatch_count"], 0)
        self.assertEqual(summary["missing_sample_count"], 0)
        self.assertEqual(samples[0]["obs_time_us"], t0)
        self.assertEqual(samples[1]["history"], obs0 * 4 + obs1)
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "decoded.csv"
            TRACE.write_csv(output, samples)
            with output.open(newline="") as stream:
                rows = list(csv.reader(stream))
            self.assertEqual(len(rows[0]), len(rows[1]))
            self.assertEqual(len(rows[0]), len(rows[2]))

    def test_missing_sample_invalidates_history_until_sync(self):
        obs = (1.0,) * 25
        frames = [frame(0, 1, 10000, 7, obs),
                  *[frame(2 + i, 1, 10000, 7, obs) for i in range(5)],
                  frame(0, 3, 30000, 7, obs),
                  frame(0, 4, 40000, 7, obs),
                  *[frame(2 + i, 4, 40000, 7, obs) for i in range(5)]]
        samples, summary = TRACE.reconstruct(frames)
        self.assertEqual(summary["missing_sample_count"], 1)
        self.assertEqual(summary["unsynced_sample_count"], 1)
        self.assertIsNone(samples[1]["history"])
        self.assertIsNotNone(samples[2]["history"])

    def test_raw_and_csv_inputs(self):
        obs = (2.0,) * 25
        values = frame(0, 17, 87654321, 0, obs)
        with tempfile.TemporaryDirectory() as directory:
            raw = Path(directory) / "trace.bin"
            raw.write_bytes(b"bad" + struct.pack("<32f", *values) + TRACE.TAIL)
            decoded, ignored = TRACE.read_raw(raw)
            self.assertEqual(len(decoded), 1)
            self.assertEqual(ignored, 3)
            self.assertEqual(TRACE.decode_header(decoded[0])[1], 17)
            csv_path = Path(directory) / "trace.csv"
            with csv_path.open("w", newline="") as stream:
                writer = csv.writer(stream)
                writer.writerow(["Time"] + ["ch{}".format(i) for i in range(32)])
                writer.writerow(["12:00:00"] + list(values))
            decoded, ignored = TRACE.read_csv(csv_path)
            self.assertEqual(len(decoded), 1)
            self.assertEqual(ignored, 1)

    def test_raw_payload_tail_collision(self):
        # A zero action followed by 65791.0 contains TAIL across float boundaries.
        obs = frame(0, 6597, 65960179, 0, (0.0,) * 25, 65791)
        aux = frame(1, 6597, 65960191, 0, (0.0,) * 25)
        payload = struct.pack("<32f", *obs)
        self.assertEqual(payload.find(TRACE.TAIL), 122)
        for prefix in (b"", b"bad", b"noise" * 30):
            with self.subTest(prefix_length=len(prefix)), tempfile.TemporaryDirectory() as directory:
                raw = Path(directory) / "trace.bin"
                raw.write_bytes(prefix + payload + TRACE.TAIL
                                + struct.pack("<32f", *aux) + TRACE.TAIL)
                decoded, ignored = TRACE.read_raw(raw)
                self.assertEqual(decoded, [obs, aux])
                self.assertEqual(ignored, len(prefix))

    def test_raw_resync_after_partial_and_invalid_frames(self):
        first = frame(0, 10, 100000, 0, (2.0,) * 25)
        second = frame(0, 11, 110000, 0, (3.0,) * 25)
        third = frame(1, 11, 110020, 0, (4.0,) * 25)
        packed = lambda value: struct.pack("<32f", *value) + TRACE.TAIL
        invalid = list(first)
        invalid[0] = 99.0
        damaged = packed(first)[:47] + packed(first)[54:]
        prefix = packed(first)[71:]
        suffix = packed(third)[:65]
        data = prefix + packed(second) + damaged + packed(invalid) + packed(third) + suffix
        with tempfile.TemporaryDirectory() as directory:
            raw = Path(directory) / "trace.bin"
            raw.write_bytes(data)
            decoded, ignored = TRACE.read_raw(raw)
            self.assertEqual(decoded, [second, third])
            self.assertEqual(ignored, len(data) - 2 * 132)


if __name__ == "__main__":
    unittest.main()
