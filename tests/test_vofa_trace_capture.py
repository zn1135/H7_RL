import contextlib
import hashlib
import io
import json
from pathlib import Path
import struct
import sys
import tempfile
import unittest
from unittest.mock import patch


TOOLS = Path(__file__).resolve().parents[1] / "tools"
sys.path.insert(0, str(TOOLS))
import vofa_trace_capture as capture


class FakeSerial:
    def __init__(self, chunks, open_error=None):
        self.chunks = iter(chunks)
        self.open_error = open_error
        self.opened = False
        self.closed = False

    def open(self):
        if self.open_error:
            raise self.open_error
        self.opened = True

    def read(self, size):
        value = next(self.chunks, KeyboardInterrupt())
        if isinstance(value, BaseException):
            raise value
        return value

    def close(self):
        self.closed = True

    def write(self, data):
        raise AssertionError("passive capture must not transmit")


def fixture():
    header = [0., 6597., 30963., 1006., 0., 0.]
    obs = header + [0.] * 25 + [65791.]
    aux = [1., 6597., 30975., 1006., 0., 0.] + [0.] * 26
    return b"prefix" + struct.pack("<32f", *obs) + capture.trace.TAIL + struct.pack("<32f", *aux) + capture.trace.TAIL


class CaptureTest(unittest.TestCase):
    def test_chunked_capture_keeps_raw_and_decodes_on_ctrl_c(self):
        data = fixture()
        device = FakeSerial([data[:11], b"", data[11:131], data[131:]])
        with tempfile.TemporaryDirectory() as directory, contextlib.redirect_stdout(io.StringIO()):
            output = Path(directory) / "capture"
            self.assertEqual(capture.capture(device, output, 0, "TEST", 1152000), 0)
            self.assertEqual((output / "raw.bin").read_bytes(), data)
            meta = json.loads((output / "capture_summary.json").read_text())
            self.assertEqual(meta["raw_sha256"], hashlib.sha256(data).hexdigest())
            self.assertEqual(meta["bytes_received"], len(data))
            self.assertEqual(meta["stop_reason"], "interrupted")
            summary = json.loads((output / "summary.json").read_text())
            self.assertEqual(summary["sample_count"], 1)
            self.assertEqual(summary["missing_aux_count"], 0)
            self.assertEqual(summary["ignored_bytes_or_rows"], 6)
            self.assertTrue((output / "vofa_trace.csv").is_file())
        self.assertTrue(device.closed)

    def test_disconnect_preserves_received_bytes(self):
        data = fixture()
        device = FakeSerial([data, OSError("disconnected")])
        with tempfile.TemporaryDirectory() as directory, contextlib.redirect_stdout(io.StringIO()):
            output = Path(directory) / "capture"
            self.assertEqual(capture.capture(device, output, 0, "TEST", 1152000), 1)
            self.assertEqual((output / "raw.bin").read_bytes(), data)
            meta = json.loads((output / "capture_summary.json").read_text())
            self.assertEqual(meta["stop_reason"], "error")
            self.assertEqual(meta["error"], "disconnected")
        self.assertTrue(device.closed)

    def test_open_failure_records_error(self):
        device = FakeSerial([], open_error=OSError("port busy"))
        with tempfile.TemporaryDirectory() as directory, contextlib.redirect_stdout(io.StringIO()):
            output = Path(directory) / "capture"
            self.assertEqual(capture.capture(device, output, 1, "TEST", 1152000), 1)
            meta = json.loads((output / "capture_summary.json").read_text())
            self.assertEqual(meta["bytes_received"], 0)
            self.assertEqual(meta["error"], "port busy")
        self.assertTrue(device.closed)

    def test_duration_stops_without_interrupt(self):
        device = FakeSerial([fixture()])
        with tempfile.TemporaryDirectory() as directory, contextlib.redirect_stdout(io.StringIO()), \
                patch.object(capture.time, "monotonic", side_effect=[0., 0., 0., 0.2, 1., 1.]):
            output = Path(directory) / "capture"
            self.assertEqual(capture.capture(device, output, 1, "TEST", 1152000), 0)
            meta = json.loads((output / "capture_summary.json").read_text())
            self.assertEqual(meta["stop_reason"], "duration")

    def test_existing_directory_is_not_overwritten_or_opened(self):
        device = FakeSerial([])
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            (output / "raw.bin").write_bytes(b"original")
            with self.assertRaises(FileExistsError):
                capture.capture(device, output, 1, "TEST", 1152000)
            self.assertEqual((output / "raw.bin").read_bytes(), b"original")
        self.assertFalse(device.opened)

    def test_no_data_is_not_reported_as_success(self):
        device = FakeSerial([])
        with tempfile.TemporaryDirectory() as directory, contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(capture.capture(device, Path(directory) / "capture", 0, "TEST", 1152000), 1)


if __name__ == "__main__":
    unittest.main()
