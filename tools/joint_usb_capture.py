#!/usr/bin/env python3
"""USB CDC 关节映射采集与限值门控的单电机力矩试验。"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path
import struct
import sys
import time

from joint_usb_protocol import (ARM, ECHO, ECHO_REPLY, MOTOR_NAMES, REPLY,
                                SAMPLE, SAMPLE_COLUMNS, SET, STATE, STATUS,
                                STOP, STREAM, Decoder, decode_sample, decode_state,
                                encode)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


class Client:
    def __init__(self, serial_port, raw, csv_file):
        self.port = serial_port
        self.raw = raw
        self.csv = csv_file
        self.decoder = Decoder()
        self.sequence = 0
        self.pending = {}
        self.phase = ""
        self.samples = 0
        self.sample_gaps = 0
        self.last_frame_seq = None
        self.last_sample = None
        self.rejection = None
        self.mapping = {}
        self.run_drop_start = None
        self.run_drop_end = None
        self.last_print = 0.0

    def send(self, kind: int, payload: bytes = b"") -> int:
        self.sequence += 1
        self.port.write(encode(kind, self.sequence, payload))
        return self.sequence

    def poll(self):
        chunk = self.port.read(min(max(self.port.in_waiting, 1), 16384))
        if not chunk:
            return
        self.raw.write(chunk)
        for kind, seq, payload in self.decoder.feed(chunk):
            if kind == SAMPLE:
                if self.last_frame_seq is not None:
                    gap = (seq - self.last_frame_seq - 1) & 0xffffffff
                    if gap < 1000000:
                        self.sample_gaps += gap
                self.last_frame_seq = seq
                row = decode_sample(payload)
                row.update(host_recv_ns=time.monotonic_ns(), phase=self.phase,
                           frame_seq=seq)
                self.csv.writerow(row)
                self.samples += 1
                self.last_sample = row
                if self.phase.startswith("run_"):
                    if self.run_drop_start is None:
                        self.run_drop_start = row["dropped_samples"]
                    self.run_drop_end = row["dropped_samples"]
                self._mapping_update(row)
                self._show(row)
            else:
                self.pending[(kind, seq)] = payload
                if kind == REPLY and payload and payload[0] != 0:
                    self.rejection = (seq, payload[0])
                if len(self.pending) > 256:
                    for key in list(self.pending)[:128]:
                        if key[0] == REPLY:
                            self.pending.pop(key, None)

    def wait(self, kind: int, seq: int, timeout: float = 2.0) -> bytes:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.poll()
            found = self.pending.pop((kind, seq), None)
            if found is not None:
                return found
            time.sleep(0.001)
        raise TimeoutError(f"no frame kind={kind} seq={seq}")

    def check_reply(self, seq: int):
        payload = self.wait(REPLY, seq)
        if payload[0] != 0:
            raise RuntimeError(f"H7 rejected seq={seq}, code={payload[0]}")
        return struct.unpack_from("<Q", payload, 1)[0]

    def _mapping_update(self, row):
        if not self.phase.startswith("move_"):
            return
        record = self.mapping.setdefault(self.phase, {
            motor: {"first": None, "last": None, "min": float("inf"),
                    "max": float("-inf")} for motor in MOTOR_NAMES
        })
        for motor in MOTOR_NAMES:
            angle = row[f"q_raw_{motor}_rad"]
            item = record[motor]
            if item["first"] is None:
                item["first"] = angle
            item["last"] = angle
            item["min"] = min(item["min"], angle)
            item["max"] = max(item["max"], angle)

    def _show(self, row):
        now = time.monotonic()
        if now - self.last_print < 0.25:
            return
        self.last_print = now
        angles = " ".join(f"{row[f'q_raw_{m}_rad']:+.3f}" for m in MOTOR_NAMES)
        velocity = " ".join(f"{row[f'dq_{m}_rad_s']:+.2f}" for m in MOTOR_NAMES)
        command = " ".join(f"{row[f'tau_cmd_{m}_nm']:+.2f}" for m in MOTOR_NAMES)
        feedback = " ".join(f"{row[f'tau_fb_{m}_nm']:+.2f}" for m in MOTOR_NAMES)
        print(f"\r{self.phase:>16} q=[{angles}] dq=[{velocity}] "
              f"τcmd=[{command}] τfb=[{feedback}] "
              f"legL/R={row['leg_len_left_m']:.3f}/{row['leg_len_right_m']:.3f} "
              f"angleL/R={row['leg_angle_left_rad']:.3f}/{row['leg_angle_right_rad']:.3f} "
              f"fault=0x{row['faults']:x} drop={row['dropped_samples']}",
              end="", flush=True)


def capture_for(client: Client, seconds: float, send_program=None):
    start = time.monotonic()
    next_send = start
    index = 0
    first_seq = None
    while time.monotonic() - start < seconds:
        now = time.monotonic()
        if send_program is not None and index < len(send_program) and now >= next_send:
            if now - next_send > 0.02:
                raise RuntimeError("host scheduling late by >20 ms; stop")
            motor, value = send_program[index]
            sent_seq = client.send(SET, struct.pack("<Bf", motor, value))
            if first_seq is None:
                first_seq = sent_seq
            index += 1
            next_send = start + index * 0.01
        client.poll()
        if client.rejection is not None:
            raise RuntimeError(f"H7 rejected command {client.rejection}")
        if (send_program is not None and client.last_sample is not None
                and first_seq is not None
                and client.last_sample["host_cmd_seq"] >= first_seq):
            flags = client.last_sample["flags"]
            if index > 0 and (flags & 2 or not flags & 1):
                raise RuntimeError("H7 disarmed or latched stop")
        time.sleep(0.001)
    if send_program is not None and index != len(send_program):
        raise RuntimeError(f"only sent {index}/{len(send_program)} setpoints")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="H7 板载 USB CDC，如 /dev/ttyACM0")
    parser.add_argument("--out", required=True, type=Path, help="新采集目录")
    parser.add_argument("--mode", choices=("echo", "passive", "map", "run"),
                        default="passive")
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--phase-seconds", type=float, default=6.0)
    parser.add_argument("--reset-seconds", type=float, default=3.0)
    parser.add_argument("--program", type=Path,
                        help="run 模式 JSON: motor_index 与 100 Hz samples_nm")
    args = parser.parse_args()
    if args.out.exists():
        parser.error("--out 必须是不存在的目录，以免覆盖采集")
    if args.duration <= 0 or args.phase_seconds <= 0 or args.reset_seconds <= 0:
        parser.error("duration 必须为正数")
    if args.mode == "run" and args.program is None:
        parser.error("run 模式需要 --program")

    try:
        import serial
    except ImportError as exc:
        raise SystemExit("请先在上位机安装 pyserial") from exc

    program = None
    if args.program is not None:
        program = json.loads(args.program.read_text(encoding="utf-8"))
        if program.get("sample_hz") != 100:
            parser.error("program.sample_hz 必须为 100")
        motor = program.get("motor_index")
        values = program.get("samples_nm")
        if type(motor) is not int or motor not in range(4) or not isinstance(values, list) or not values:
            parser.error("program 需要 motor_index=0..3 和非空 samples_nm")
        if any(type(x) not in (int, float) or not float("-inf") < x < float("inf") for x in values):
            parser.error("samples_nm 必须全部是有限数字")

    args.out.mkdir(parents=True)
    raw_path = args.out / "usb_raw.bin"
    csv_path = args.out / "joint_samples.csv"
    manifest_path = args.out / "manifest.json"
    state = None
    result = "incomplete"
    try:
        with serial.Serial(args.port, 1152000, timeout=0, write_timeout=1) as port, \
             raw_path.open("wb") as raw, csv_path.open("w", newline="", encoding="utf-8") as csv_handle:
            client = Client(port, raw, csv.DictWriter(csv_handle, fieldnames=SAMPLE_COLUMNS))
            client.csv.writeheader()
            seq = client.send(STATUS)
            state = decode_state(client.wait(STATE, seq))
            print("USB connected:", json.dumps(state, ensure_ascii=False))
            if args.mode != "echo":
                seq = client.send(STREAM, b"\x01")
                client.check_reply(seq)
            try:
                if args.mode == "echo":
                    for index in range(100):
                        payload = struct.pack("<I", index) + bytes(range(28))
                        seq = client.send(ECHO, payload)
                        if client.wait(ECHO_REPLY, seq) != payload:
                            raise RuntimeError("USB echo mismatch")
                    print("100/100 echo frames matched")
                elif args.mode in ("passive", "map"):
                    if state["motor_enabled"]:
                        raise RuntimeError("passive/map 要求 H7 电机失能")
                    if args.mode == "passive":
                        client.phase = "manual"
                        capture_for(client, args.duration)
                    else:
                        for motor in MOTOR_NAMES:
                            client.phase = "move_" + motor
                            print(f"\n请在 {args.phase_seconds:g} 秒内沿现场标记的正方向缓慢摆动实体 {motor} 支路并保持")
                            capture_for(client, args.phase_seconds)
                            client.phase = "reset"
                            print(f"\n请在 {args.reset_seconds:g} 秒内返回初始位置")
                            capture_for(client, args.reset_seconds)
                        report = {
                            phase: {m: {"delta_rad": round(v["last"] - v["first"], 5),
                                        "span_rad": round(v["max"] - v["min"], 5)}
                                    for m, v in motors.items()}
                            for phase, motors in client.mapping.items()
                        }
                        (args.out / "mapping_spans.json").write_text(
                            json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8"
                        )
                        print("\n角度带符号变化和幅度（闭链会联动，请现场核对电机标签和方向）：", report)
                else:
                    if not state["limits_approved"] or not state["physical_permit"]:
                        raise RuntimeError("H7 限值未获批准或遥控未处于左上＋右下")
                    if max(abs(x) for x in values) > state["torque_max_nm"][motor]:
                        raise RuntimeError("program 超过 H7 上报的力矩限值")
                    try:
                        seq = client.send(ARM)
                        client.check_reply(seq)
                        client.phase = f"run_{MOTOR_NAMES[motor]}"
                        capture_for(client, len(values) * 0.01,
                                    [(motor, float(value)) for value in values])
                        client.phase = "zero_after"
                        capture_for(client, 0.2, [(motor, 0.0)] * 20)
                        if (client.decoder.bad_frames
                                or client.run_drop_start is None
                                or client.sample_gaps > max(2, client.samples // 100)
                                or client.run_drop_end - client.run_drop_start
                                > max(2, client.samples // 100)):
                            raise RuntimeError("run 坏帧、无样本或丢帧超过 1%")
                    finally:
                        seq = client.send(STOP)
                        client.check_reply(seq)
            finally:
                if args.mode != "echo":
                    seq = client.send(STREAM, b"\x00")
                    client.check_reply(seq)
            result = "complete"
            print(f"\n样本 {client.samples}，序号缺口 {client.sample_gaps}，"
                  f"错误帧 {client.decoder.bad_frames}")
    except Exception as exc:
        print(f"\n采集失败：{exc}", file=sys.stderr)
        result = f"failed: {exc}"
        raise
    finally:
        manifest = {"mode": args.mode, "result": result, "port": args.port,
                    "state_at_start": state, "program": program,
                    "samples": client.samples if "client" in locals() else 0,
                    "sample_gaps": client.sample_gaps if "client" in locals() else 0,
                    "bad_frames": client.decoder.bad_frames if "client" in locals() else 0,
                    "raw_sha256": sha256(raw_path) if raw_path.exists() else None,
                    "csv_sha256": sha256(csv_path) if csv_path.exists() else None}
        manifest_path.write_text(json.dumps(manifest, indent=2, ensure_ascii=False),
                                 encoding="utf-8")


if __name__ == "__main__":
    main()
