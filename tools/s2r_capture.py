#!/usr/bin/env python3
"""Passive S2R1 capture/decode. No serial commands or motor control."""
import argparse
import csv
import json
import math
from pathlib import Path
import struct
import time
import zlib

HEADER = struct.Struct("<4sBBHHHIQIQI")
TYPES = {1: "META", 2: "EVENT", 3: "POLICY", 4: "CONTROL", 5: "IMU", 6: "HEALTH", 7: "HISTORY"}
STARTED, ACTIVE = 1, 4
REQUIRED = STARTED | ACTIVE | 8 | 16 | 32 | 64 | 128 | 2048
SCHEMAS = {
    2: [("event_seq", "I", 1), ("event_code", "H", 1), ("reason_code", "H", 1),
        ("policy_seq", "I", 1), ("control_seq", "I", 1), ("detail", "I", 2)],
    3: [(k, "I", 1) for k in ("policy_seq", "obs_seq", "history_seq", "history_epoch")]
       + [(k, "Q", 1) for k in ("t_obs_us", "t_infer_start_us", "t_infer_end_us")]
       + [("obs", "f", 25), ("action_raw", "f", 6), ("action_published", "f", 6), ("latent", "f", 3), ("command", "f", 3)],
    4: [(k, "I", 1) for k in ("control_seq", "used_policy_seq", "state_seq", "imu_seq", "control_dt_us", "control_exec_us")]
       + [("t_dm_enqueue_us", "Q", 1), ("t_dji_enqueue_us", "Q", 1), ("motor_rx_us", "Q", 6)]
       + [(k, "I", 1) for k in ("motor_feedback_valid_mask", "motor_send_ok_mask", "motor_clamp_mask", "virtual_clamp_mask", "pd_wrap_mask")]
       + [("motor_field_valid", "I", 6)]
       + [(k, "f", n) for k, n in (("q_virtual_fw", 6), ("dq_virtual_fw", 6), ("q_target_fw", 4),
            ("wheel_speed_target_fw", 2), ("error_before_wrap_fw", 4), ("error_after_wrap_fw", 4),
            ("tau_virtual_raw_fw", 6), ("tau_virtual_limited_fw", 6), ("gas_tau_shank_fw", 2),
            ("shank_jacobian", 4), ("tau_motor_unclipped", 6), ("tau_motor_request", 6),
            ("tau_motor_feedback", 6), ("q_motor", 6), ("dq_motor", 6), ("current_motor", 6))],
    5: [("imu_seq", "I", 1), ("t_imu_rx_us", "Q", 1), ("valid_mask", "I", 1), ("filter_mask", "I", 1)]
       + [(k, "f", n) for k, n in (("quat_sensor", 4), ("gyro_sensor", 3), ("accel_sensor", 3), ("quat_body", 4), ("gyro_body", 3))],
    6: [(k, "I", 1) for k in ("window_us", "policy_attempt_count", "control_count",
        "policy_dt_min_us", "policy_dt_max_us", "policy_dt_sum_us", "policy_dt_count",
        "control_dt_min_us", "control_dt_max_us", "control_dt_sum_us", "control_dt_count",
        "infer_exec_max_us", "control_exec_max_us", "policy_overrun_count", "control_overrun_count",
        "imu_age_max_us", "motor_age_max_us", "can_submit_fail_count", "infer_fail_count",
        "telemetry_drop_total", "uart_busy_total", "telemetry_queue_peak_bytes", "motor_clamp_or_mask",
        "virtual_clamp_or_mask", "telemetry_generated_total")],
    7: [("policy_seq", "I", 1), ("history_seq", "I", 1), ("history_epoch", "I", 1), ("history", "f", 125)],
}
FORMATS = {kind: struct.Struct("<" + "".join(code * count for _, code, count in schema))
           for kind, schema in SCHEMAS.items()}


def payload_decode(kind, payload):
    if kind == 1:
        if len(payload) < 12:
            raise ValueError("short META")
        meta_id, index, count, size = struct.unpack_from("<IHHI", payload)
        if not 0 < count <= 16 or index >= count or not 0 < size <= 8192:
            raise ValueError("invalid META bounds")
        chunk = payload[12:]
        if count != (size + 511) // 512 or len(chunk) != min(512, size - index * 512):
            raise ValueError("invalid META chunk length")
        return {"meta_id": meta_id, "chunk_index": index, "chunk_count": count,
                "total_json_bytes": size, "json_chunk_hex": chunk.hex()}
    values = FORMATS[kind].unpack(payload)
    result, offset = {}, 0
    for name, _, count in SCHEMAS[kind]:
        result[name] = list(values[offset:offset + count]) if count > 1 else values[offset]
        offset += count
    return result


class Parser:
    def __init__(self):
        self.buffer = bytearray()
        self.stats = dict(frames=0, garbage_bytes=0, crc_errors=0, format_errors=0, seq_gaps=0, duplicates=0)
        self.last_seq = {}

    def feed(self, data):
        self.buffer.extend(data)
        frames = []
        while len(self.buffer) >= 4:
            offset = self.buffer.find(b"S2R1")
            if offset < 0:
                skip = max(0, len(self.buffer) - 3)
                self.stats["garbage_bytes"] += skip
                del self.buffer[:skip]
                break
            if offset:
                self.stats["garbage_bytes"] += offset
                del self.buffer[:offset]
            if len(self.buffer) < HEADER.size:
                break
            _, version, kind, header_len, size, reserved, seq, boot, session, timestamp, flags = HEADER.unpack_from(self.buffer)
            if (version != 1 or kind not in TYPES or header_len != 40 or size > 1024 or reserved
                    or (kind in FORMATS and size != FORMATS[kind].size)
                    or (kind == 1 and not 12 <= size <= 524)):
                self.stats["format_errors"] += 1
                del self.buffer[0]
                continue
            total = size + 44
            if len(self.buffer) < total:
                break
            expected = struct.unpack_from("<I", self.buffer, 40 + size)[0]
            if zlib.crc32(self.buffer[:40 + size]) != expected:
                self.stats["crc_errors"] += 1
                del self.buffer[0]
                continue
            raw_payload = bytes(self.buffer[40:40 + size])
            del self.buffer[:total]
            try:
                payload = payload_decode(kind, raw_payload)
            except (ValueError, struct.error):
                self.stats["format_errors"] += 1
                continue
            gap = 0
            duplicate = False
            if boot in self.last_seq:
                delta = (seq - self.last_seq[boot]) & 0xFFFFFFFF
                if delta == 0 or delta > 0x80000000:
                    self.stats["duplicates"] += 1
                    duplicate = True
                else:
                    gap = delta - 1
                    self.stats["seq_gaps"] += gap
            if not duplicate:
                self.last_seq[boot] = seq
            self.stats["frames"] += 1
            frames.append(dict(type=TYPES[kind], frame_type=kind, packet_seq=seq, boot_id=f"{boot:016x}",
                session_id=session, t_us=timestamp, flags=flags, packet_gap=gap, packet_duplicate=duplicate, payload=payload))
        return frames


def clean(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {k: clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean(v) for v in value]
    return value


class Exporter:
    def __init__(self, folder):
        self.folder = folder
        self.states, self.meta, self.files = {}, {}, {}
        self.event_seen = set()

    def accept(self, frame):
        key = (frame["boot_id"], frame["session_id"])
        state = self.states.setdefault(key, {"metadata_ready": False, "history": None, "policy_inputs": {}})
        p = frame["payload"]
        if frame["packet_gap"]:
            # A gap may have contained observations for this boot's active session.
            for k, s in self.states.items():
                if k[0] == key[0]:
                    s["history"] = None
                    s["policy_inputs"].clear()
        duplicate = frame.get("packet_duplicate", False)
        if not duplicate and frame["type"] == "META":
            mkey = key + (p["meta_id"],)
            chunks = self.meta.setdefault(mkey, {})
            chunks[p["chunk_index"]] = bytes.fromhex(p["json_chunk_hex"])
            if len(chunks) == p["chunk_count"]:
                try:
                    metadata = json.loads(b"".join(chunks[i] for i in range(p["chunk_count"])))
                    state["metadata_ready"] = metadata.get("protocol_version") == 1
                    frame["metadata"] = metadata
                except (ValueError, KeyError):
                    state["metadata_ready"] = False
        elif not duplicate and frame["type"] == "HISTORY":
            if frame["flags"] & 16 and all(math.isfinite(v) for v in p["history"]):
                state["history"] = (p["history_epoch"], p["history_seq"], p["history"])
                state["policy_inputs"][p["policy_seq"]] = p["history"]
            else:
                state["history"] = None
        elif not duplicate and frame["type"] == "POLICY":
            previous = state["history"]
            if (previous and previous[0] == p["history_epoch"] and frame["flags"] & 24 == 24
                    and all(math.isfinite(v) for v in p["obs"])):
                delta = (p["history_seq"] - previous[1]) & 0xFFFFFFFF
                if delta == 1:
                    state["history"] = (p["history_epoch"], p["history_seq"], previous[2][25:] + p["obs"])
                elif delta != 0 or previous[2][-25:] != p["obs"]:
                    state["history"] = None
            else:
                state["history"] = None
            if state["history"]:
                frame["network_history"] = state["history"][2]
                state["policy_inputs"][p["policy_seq"]] = state["history"][2]
        elif not duplicate and frame["type"] == "EVENT":
            ekey = key + (p["event_seq"],)
            frame["duplicate_event"] = ekey in self.event_seen
            self.event_seen.add(ekey)
            if not frame["duplicate_event"] and p["event_code"] in (1, 3, 6, 7):
                state["history"] = None
                state["policy_inputs"].clear()
            if not frame["duplicate_event"] and p["event_code"] == 7:
                state["metadata_ready"] = False
        while len(state["policy_inputs"]) > 100:
            del state["policy_inputs"][next(iter(state["policy_inputs"]))]
        if frame["type"] == "CONTROL":
            synchronized = p["used_policy_seq"] in state["policy_inputs"]
        else:
            synchronized = state["history"] is not None
        frame["policy_started"] = bool(frame["flags"] & STARTED)
        frame["policy_active"] = bool(frame["flags"] & ACTIVE)
        frame["history_synchronized"] = synchronized
        frame["metadata_ready"] = state["metadata_ready"]
        frame["valid_policy_segment"] = bool(key[1] and frame["flags"] & REQUIRED == REQUIRED
            and not duplicate and not frame["flags"] & (256 | 512 | 1024)
            and state["metadata_ready"] and synchronized)
        session_dir = self.folder / f"boot_{key[0]}" / f"session_{key[1]:06d}"
        session_dir.mkdir(parents=True, exist_ok=True)
        if key not in self.files:
            self.files[key] = (session_dir / "frames.jsonl").open("x", encoding="utf-8")
        self.files[key].write(json.dumps(clean(frame), ensure_ascii=False, allow_nan=False) + "\n")
        if "metadata" in frame:
            target = session_dir / f"meta_{p['meta_id']}.json"
            if not target.exists():
                target.write_text(json.dumps(frame["metadata"], ensure_ascii=False, indent=2) + "\n")
        ckey = key + (frame["type"],)
        row = {k: frame[k] for k in ("packet_seq", "t_us", "flags", "packet_gap", "policy_started", "policy_active", "valid_policy_segment")}
        for name, value in p.items():
            if isinstance(value, list):
                row.update({f"{name}_{i}": clean(v) for i, v in enumerate(value)})
            else:
                row[name] = clean(value)
        if ckey not in self.files:
            stream = (session_dir / (frame["type"].lower() + ".csv")).open("x", newline="")
            writer = csv.DictWriter(stream, fieldnames=list(row))
            writer.writeheader()
            self.files[ckey] = (stream, writer)
        self.files[ckey][1].writerow(row)

    def close(self):
        for value in self.files.values():
            (value[0] if isinstance(value, tuple) else value).close()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    source = ap.add_mutually_exclusive_group(required=True)
    source.add_argument("--port", help="USB serial device, e.g. /dev/ttyUSB0 or COM5")
    source.add_argument("--input", type=Path, help="decode an existing raw.bin")
    ap.add_argument("--output", type=Path, required=True, help="new output directory")
    ap.add_argument("--baud", type=int, default=1152000)
    ap.add_argument("--seconds", type=float, default=0, help="0: capture until Ctrl+C")
    args = ap.parse_args()
    if args.seconds < 0:
        ap.error("seconds must be nonnegative")
    args.output.mkdir(parents=True, exist_ok=False)
    if args.port:
        try:
            import serial
        except ImportError:
            raise SystemExit("Live capture requires pyserial: python -m pip install pyserial")
        device = serial.Serial(port=None, baudrate=args.baud, timeout=0.1, rtscts=False, dsrdtr=False)
        device.dtr = False
        device.rts = False
        device.port = args.port
        device.open()
    else:
        device = args.input.open("rb")
    parser, exporter = Parser(), Exporter(args.output)
    started = last_status = time.monotonic()
    last_frame = None
    try:
        with device, (args.output / "raw.bin").open("xb") as raw:
            while not args.seconds or time.monotonic() - started < args.seconds:
                data = device.read(4096)
                if not data and args.input:
                    break
                raw.write(data)
                for frame in parser.feed(data):
                    frame["host_rx_time_ns"] = time.time_ns()
                    exporter.accept(frame)
                    last_frame = frame
                if time.monotonic() - last_status >= 1:
                    state = {k: last_frame[k] for k in ("boot_id", "session_id", "policy_started", "policy_active", "history_synchronized", "metadata_ready")} if last_frame else {}
                    print(json.dumps(dict(state=state, parser=parser.stats)), flush=True)
                    last_status = time.monotonic()
    except KeyboardInterrupt:
        pass
    finally:
        exporter.close()
        parser.stats["trailing_bytes"] = len(parser.buffer)
        (args.output / "capture_summary.json").write_text(json.dumps(dict(
            source=args.port or str(args.input), baud=args.baud,
            elapsed_s=time.monotonic() - started, statistics=parser.stats), indent=2) + "\n")
    print(json.dumps(parser.stats))


if __name__ == "__main__":
    main()
