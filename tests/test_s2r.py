"""Host protocol/mocked lifecycle checks, not an ARM firmware build."""
import importlib.util
import json
import os
from pathlib import Path
import random
import shutil
import struct
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
import zlib

ROOT = Path(__file__).resolve().parents[1]
sys.dont_write_bytecode = True
spec = importlib.util.spec_from_file_location("s2r", ROOT / "tools/s2r_capture.py")
s2r = importlib.util.module_from_spec(spec)
spec.loader.exec_module(s2r)


def frame(kind, seq=0, session=1, flags=0, payload=None, boot=42):
    if payload is None:
        payload = bytes(s2r.FORMATS[kind].size)
    head = s2r.HEADER.pack(b"S2R1", 1, kind, 40, len(payload), 0, seq, boot, session, 100, flags)
    data = head + payload
    return data + struct.pack("<I", zlib.crc32(data))


def meta(seq=0, session=1):
    data = b'{"protocol_version":1}'
    return frame(1, seq, session, payload=struct.pack("<IHHI", 1, 0, 1, len(data)) + data)


def history(seq, epoch=1, step=1):
    return frame(7, seq, flags=s2r.REQUIRED, payload=struct.pack("<III125f", step, step, epoch, *range(125)))


def policy(seq, step, epoch=1, flags=s2r.REQUIRED):
    data = struct.pack("<IIIIQQQ25f6f6f3f3f", step, step, step, epoch, 10, 20, 30,
                       *([float(step)] * 25 + [0.] * 18))
    return frame(3, seq, flags=flags, payload=data)


class ProtocolTests(unittest.TestCase):
    def test_sizes_and_endian(self):
        self.assertEqual(s2r.HEADER.size, 40)
        self.assertEqual({k: v.size for k, v in s2r.FORMATS.items()}, {2: 24, 3: 212, 4: 452, 5: 88, 6: 100, 7: 512})
        self.assertEqual(zlib.crc32(b"123456789"), 0xCBF43926)
        p = bytearray(452)
        struct.pack_into("<I", p, 4, 123)
        for offset, value in ((132, 1.), (308, 2.), (332, 3.), (356, 4.), (380, 5.), (404, 6.), (428, 7.)):
            struct.pack_into("<f", p, offset, value)
        decoded = s2r.payload_decode(4, p)
        self.assertEqual(decoded["used_policy_seq"], 123)
        for name, value in (("q_virtual_fw", 1), ("tau_motor_unclipped", 2), ("tau_motor_request", 3),
                            ("tau_motor_feedback", 4), ("q_motor", 5), ("dq_motor", 6), ("current_motor", 7)):
            self.assertEqual(decoded[name][0], value)

    def test_all_split_points_and_concatenation(self):
        data = meta() + b"".join(frame(k, k) for k in range(2, 8))
        for i in range(len(data) + 1):
            parser = s2r.Parser()
            records = parser.feed(data[:i]) + parser.feed(data[i:])
            self.assertEqual([v["frame_type"] for v in records], list(range(1, 8)))
            self.assertFalse(parser.buffer)

    def test_noise_corruption_and_bad_length(self):
        bad = bytearray(frame(4))
        bad[140] ^= 1
        invalid = bytearray(frame(4))
        struct.pack_into("<H", invalid, 8, 1000)
        data = b"boot logs\xff\n" + bad + invalid[:40] + meta(1) + frame(6, 4)
        parser, records = s2r.Parser(), []
        rng = random.Random(4)
        while data:
            count = rng.randrange(1, 24)
            records += parser.feed(data[:count])
            data = data[count:]
        self.assertEqual([v["frame_type"] for v in records], [1, 6])
        self.assertEqual(parser.stats["crc_errors"], 1)
        self.assertGreater(parser.stats["format_errors"], 0)
        self.assertEqual(parser.stats["seq_gaps"], 2)

    def test_rollover_duplicates_and_boot(self):
        parser = s2r.Parser()
        records = parser.feed(frame(2, 0xFFFFFFFF) + frame(2, 0) + frame(2, 0) + frame(2, 0xFFFFFFFF)
                              + frame(2, 1) + frame(2, 500, boot=99))
        self.assertEqual(parser.stats["duplicates"], 2)
        self.assertEqual(parser.stats["seq_gaps"], 0)
        self.assertTrue(records[2]["packet_duplicate"])

    def test_middle_attach_loss_resync_and_action_binding(self):
        with tempfile.TemporaryDirectory() as folder:
            parser, exporter = s2r.Parser(), s2r.Exporter(Path(folder))
            def accept(data):
                item = parser.feed(data)[0]
                exporter.accept(item)
                return item
            try:
                accept(meta())
                self.assertFalse(accept(policy(1, 1))["history_synchronized"])
                self.assertTrue(accept(history(2))["history_synchronized"])
                reconstructed = accept(policy(3, 2))
                self.assertEqual(reconstructed["network_history"], list(range(25, 125)) + [2.] * 25)
                p = bytearray(452)
                struct.pack_into("<I", p, 4, 2)
                self.assertTrue(accept(frame(4, 4, flags=s2r.REQUIRED, payload=p))["valid_policy_segment"])
                struct.pack_into("<I", p, 4, 999)
                self.assertFalse(accept(frame(4, 5, flags=s2r.REQUIRED, payload=p))["valid_policy_segment"])
                self.assertFalse(accept(policy(7, 3))["valid_policy_segment"])
                self.assertTrue(accept(history(8, step=3))["history_synchronized"])
                self.assertTrue(accept(policy(9, 4))["valid_policy_segment"])
                self.assertFalse(accept(policy(10, 5, epoch=2))["history_synchronized"])
            finally:
                exporter.close()

    def test_event_dedup_reset_and_invalid_segments(self):
        with tempfile.TemporaryDirectory() as folder:
            parser, exporter = s2r.Parser(), s2r.Exporter(Path(folder))
            seq = 0
            def accept(data):
                nonlocal seq
                item = parser.feed(data)[0]
                seq += 1
                exporter.accept(item)
                return item
            def event(code, eid):
                return frame(2, seq, payload=struct.pack("<IHH4I", eid, code, 0, 1, 0, 0, 0))
            try:
                accept(meta(seq))
                accept(event(1, 1))
                accept(history(seq))
                self.assertTrue(accept(event(2, 2))["history_synchronized"])
                self.assertTrue(accept(event(2, 2))["duplicate_event"])
                for flag in (256, 512, 1024):
                    self.assertFalse(accept(frame(6, seq, flags=s2r.REQUIRED | flag))["valid_policy_segment"])
                self.assertFalse(accept(event(6, 3))["history_synchronized"])
                accept(history(seq, epoch=2))
                self.assertTrue(accept(event(6, 3))["history_synchronized"])
                self.assertFalse(accept(event(7, 4))["metadata_ready"])
                accept(frame(6, seq, session=0))
                self.assertFalse(accept(frame(6, seq, session=2, flags=s2r.REQUIRED))["valid_policy_segment"])
            finally:
                exporter.close()

    def test_nan_meta_bounds_and_raw_offline(self):
        with self.assertRaises(ValueError):
            s2r.payload_decode(1, struct.pack("<IHHI", 1, 0, 500, 9000))
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            source = root / "source.bin"
            source.write_bytes(b"junk\n" + meta() + history(1))
            subprocess.run([sys.executable, str(ROOT / "tools/s2r_capture.py"), "--input", str(source),
                            "--output", str(root / "out")], check=True, stdout=subprocess.PIPE)
            self.assertEqual(source.read_bytes(), (root / "out/raw.bin").read_bytes())
            self.assertEqual(json.loads((root / "out/capture_summary.json").read_text())["statistics"]["frames"], 2)
        self.assertIsNone(s2r.clean(float("nan")))

    def test_meta_out_of_order_and_fixtures(self):
        document = json.dumps({"protocol_version": 1, "padding": "x" * 1050}).encode()
        count = (len(document) + 511) // 512
        with tempfile.TemporaryDirectory() as folder:
            parser, exporter = s2r.Parser(), s2r.Exporter(Path(folder))
            try:
                for seq, index in enumerate(reversed(range(count))):
                    chunk = document[index * 512:(index + 1) * 512]
                    item = parser.feed(frame(1, seq, payload=struct.pack("<IHHI", 8, index, count, len(document)) + chunk))[0]
                    exporter.accept(item)
                    self.assertEqual(item["metadata_ready"], seq == count - 1)
            finally:
                exporter.close()
        fixture = json.loads((ROOT / "tests/fixtures/s2r_frames.json").read_text())
        for name, encoded in fixture["frames_hex"].items():
            item = s2r.Parser().feed(bytes.fromhex(encoded))
            self.assertEqual(len(item), 1)
            self.assertEqual(item[0]["type"], name)


class NativeChecks(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory(prefix="s2r-check-")
        cls.folder = Path(cls.temp.name)
        cls.cc = os.environ.get("CC") or shutil.which("gcc")
        if not cls.cc:
            raise RuntimeError("Set CC to a native GCC compiler for host checks")
        project = next((ROOT / "MDK-ARM").glob("*.uvprojx"))
        config = ET.parse(project).getroot().find(".//Cads/VariousControls")
        cls.includes = [str((project.parent / p).resolve()) for p in config.find("IncludePath").text.split(";")]
        cls.defines = ["-D" + p for p in config.find("Define").text.split(",")]
        # Preserve port declarations/types; replace only AC5 assembly implementations.
        port = (ROOT / "Middlewares/Third_Party/FreeRTOS/Source/portable/RVDS/ARM_CM4F/portmacro.h").read_text()
        start = port.index("static portFORCE_INLINE void vPortSetBASEPRI")
        end = port.index("#ifdef __cplusplus", start)
        adapter = port[:start] + "\nvoid vPortSetBASEPRI(uint32_t);\nvoid vPortRaiseBASEPRI(void);\nuint32_t ulPortRaiseBASEPRI(void);\nBaseType_t xPortIsInsideInterrupt(void);\n" + port[end:]
        (cls.folder / "portmacro.h").write_text(adapter)
        cls.common = [cls.cc, "-std=c99", "-Wall", "-Wextra", "-Wno-pointer-to-int-cast", "-Wno-int-to-pointer-cast"]
        cls.embedded = cls.defines + ["-I" + str(cls.folder)] + ["-I" + p for p in cls.includes]

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def run_cc(self, args):
        result = subprocess.run(self.common + args, cwd=ROOT, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertEqual(result.stdout, "", "Unexpected compiler diagnostic: " + result.stdout)

    def test_c_encoder_queue_cross_decode(self):
        exe, data = self.folder / "wire", self.folder / "wire.bin"
        self.run_cc(["-Werror", "-Iimcalib/Telemetry", "imcalib/Telemetry/s2r_wire.c", "tests/s2r_wire_test.c", "-o", str(exe)])
        subprocess.run([str(exe), str(data)], check=True)
        item = s2r.Parser().feed(data.read_bytes())[0]
        self.assertEqual((item["boot_id"], item["session_id"], item["packet_seq"]), ("0102030405060708", 7, 26))
        self.assertEqual(item["payload"]["event_code"], 2)

    def test_firmware_sources_host_syntax(self):
        sources = ["Core/Src/main.c", "imcalib/Algorithm/rl_torque.c"]
        sources += ["imcalib/task/" + n + ".c" for n in ("robot_control", "task_actuation", "task_comm", "task_imu", "task_policy")]
        sources += ["imcalib/user-lib/" + n + ".c" for n in ("dm", "dji", "hi229", "uart_idle")]
        sources += ["imcalib/Telemetry/" + n + ".c" for n in ("s2r_wire", "s2r_source", "s2r_telemetry")]
        for source in sources:
            with self.subTest(source=source):
                self.run_cc(["-fsyntax-only"] + self.embedded + [source])

    def test_mocked_firmware_lifecycle_dma_and_meta(self):
        exe, data, metadata = [self.folder / n for n in ("lifecycle", "lifecycle.bin", "metadata.json")]
        self.run_cc(["-O1", "-ffunction-sections", "-fdata-sections"] + self.embedded
                    + ["tests/s2r_host_test.c", "imcalib/Telemetry/s2r_wire.c", "-Wl,--gc-sections", "-lm", "-o", str(exe)])
        subprocess.run([str(exe), str(data), str(metadata)], check=True)
        meta_doc = json.loads(metadata.read_text())
        self.assertEqual(meta_doc["baud"], 1152000)
        self.assertEqual(meta_doc["model"]["name"], "model_6000_h723.onnx")
        self.assertEqual(len(meta_doc["pid_0"]), 10)
        parser = s2r.Parser()
        records = parser.feed(data.read_bytes())
        self.assertEqual(set(v["type"] for v in records), set(s2r.TYPES.values()))
        self.assertEqual(parser.stats["crc_errors"], 0)
        self.assertEqual(parser.stats["format_errors"], 0)
        self.assertEqual(parser.stats["seq_gaps"], 0)
        # 会话 0(开机帧) + 投入/重开/配置变化/重入各一段
        self.assertGreaterEqual(len({v["session_id"] for v in records}), 4)
        # 去耦采样: 至少一帧 CONTROL 绑定到已发布的策略序号, 提交位本分支不可见 = 0
        controls = [v["payload"] for v in records if v["type"] == "CONTROL"]
        self.assertIn(1, [c["used_policy_seq"] for c in controls])
        self.assertTrue(all(c["motor_send_ok_mask"] == 0 for c in controls))
        if os.environ.get("S2R_FIXTURE_DIR"):
            destination = Path(os.environ["S2R_FIXTURE_DIR"])
            destination.mkdir(parents=True, exist_ok=True)
            raw, offset, examples = data.read_bytes(), 0, {}
            while offset < len(raw):
                kind, length = raw[offset + 5], struct.unpack_from("<H", raw, offset + 8)[0] + 44
                examples.setdefault(s2r.TYPES[kind], raw[offset:offset + length].hex())
                offset += length
            (destination / "s2r_frames.json").write_text(json.dumps(dict(
                note="Synthetic host fixture, not measurements from a board. META frame is one fragment; complete example below.",
                frames_hex=examples, complete_meta=meta_doc), indent=2) + "\n")


if __name__ == "__main__":
    unittest.main(verbosity=2)
