#!/usr/bin/env python3
"""解码普通 VOFA 单侧腿诊断 24 通道；旧 10 通道须显式选择 legacy10。"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import shutil
import struct
import sys
from pathlib import Path
from typing import Sequence

CHANNELS = 24
TAIL = b"\x00\x00\x80\x7f"
FRAME_BYTES = CHANNELS * 4 + len(TAIL)
DIAGNOSTIC_FIELDS = (
    "vx_cmd_m_s", "yaw_rate_cmd_rad_s", "height_cmd_m",
    "virtual_thigh_target_rad", "virtual_thigh_measured_rad",
    "virtual_shank_target_rad", "virtual_shank_measured_rad",
    "virtual_leg_length_m", "virtual_leg_angle_rad",
    "hip_front_command_nm", "hip_rear_command_nm",
    "hip_front_feedback_nm", "hip_rear_feedback_nm",
    "hip_front_pos_zero_rad", "hip_rear_pos_zero_rad",
    "hip_front_vel_rad_s", "hip_rear_vel_rad_s",
    "virtual_shank_jac_front", "virtual_shank_jac_rear",
    "hip_front_feedback_age_ms", "hip_rear_feedback_age_ms",
)
LEGACY_FIELDS = (
    "hip_front_pos_zero_rad", "hip_rear_pos_zero_rad",
    "virtual_thigh_torque_nm", "virtual_shank_torque_nm",
    "rl_thigh_pos_rad", "rl_shank_pos_rad", "virtual_leg_length_m",
)


def layout_channels(layout: str) -> int:
    if layout == "diagnostic24":
        return CHANNELS
    if layout == "legacy10":
        return 10
    raise ValueError("unknown layout: {}".format(layout))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="VOFA+ CSV 或被动抓取的 raw .bin")
    parser.add_argument("output", type=Path, help="必须不存在的新输出目录")
    parser.add_argument("--firmware-fingerprint", default="unknown")
    parser.add_argument("--config", type=Path, help="已确认测试配置副本")
    parser.add_argument("--layout", choices=("diagnostic24", "legacy10"), default="diagnostic24",
                        help="必须匹配录制固件，不自动识别同列数的旧格式")
    return parser.parse_args()


def valid_frame(values: Sequence[float], layout: str = "diagnostic24") -> bool:
    channels = layout_channels(layout)
    max_flags = 4095 if layout == "diagnostic24" else 63
    flag_index = 15 if layout == "diagnostic24" else 9
    return (len(values) == channels and all(math.isfinite(value) for value in values)
            and values[1] in (0.0, 1.0) and values[0] >= 0.0
            and values[0].is_integer() and 0.0 <= values[flag_index] <= max_flags
            and values[flag_index].is_integer())


def read_raw(path: Path, layout: str = "diagnostic24") -> tuple[list[tuple[float, ...]], int]:
    channels = layout_channels(layout)
    payload_bytes = channels * 4
    frame_bytes = payload_bytes + len(TAIL)
    blob = path.read_bytes()
    frames: list[tuple[float, ...]] = []
    offset = 0
    ignored = 0
    while offset + frame_bytes <= len(blob):
        values = struct.unpack("<{}f".format(channels), blob[offset:offset + payload_bytes])
        if blob[offset + payload_bytes:offset + frame_bytes] == TAIL and valid_frame(values, layout):
            frames.append(values)
            offset += frame_bytes
        else:
            offset += 1
            ignored += 1
    return frames, ignored + len(blob) - offset


def read_csv(path: Path, layout: str = "diagnostic24") -> tuple[list[tuple[float, ...]], int]:
    channels = layout_channels(layout)
    frames: list[tuple[float, ...]] = []
    ignored = 0
    with path.open("r", newline="", encoding="utf-8-sig") as stream:
        for row in csv.reader(stream):
            # VOFA may prepend one host timestamp, but no other channels.
            if len(row) not in (channels, channels + 1):
                ignored += 1
                continue
            try:
                values = tuple(float(value.strip()) for value in row[-channels:])
            except ValueError:
                values = ()
            if valid_frame(values, layout):
                frames.append(values)
            else:
                ignored += 1
    return frames, ignored


def decode(frames: list[tuple[float, ...]], layout: str = "diagnostic24") -> tuple[list[dict[str, object]], dict[str, object]]:
    channels = layout_channels(layout)
    diagnostic = layout == "diagnostic24"
    field_names = DIAGNOSTIC_FIELDS if diagnostic else LEGACY_FIELDS
    observation_mask = 0x17 if diagnostic else 0x1b
    required_flags = 0x3ff if diagnostic else 0x3f
    rows: list[dict[str, object]] = []
    invalid = 0
    invalid_observations = 0
    nonincreasing = 0
    side_changes = 0
    latest_timestamp = -1
    previous_side = None
    timestamps = []
    for value in frames:
        if not valid_frame(value, layout):
            raise ValueError("invalid frame for explicitly selected layout {}".format(layout))
        timestamp = int(value[0])
        flags = int(value[15] if diagnostic else value[9])
        side = "left" if value[1] == 0.0 else "right"
        timestamp_valid = timestamp > latest_timestamp
        latest_timestamp = max(latest_timestamp, timestamp)
        side_changed = previous_side is not None and side != previous_side
        previous_side = side
        observation_valid = (flags & observation_mask) == observation_mask and timestamp_valid
        valid = flags == required_flags and timestamp_valid and not side_changed
        invalid += not valid
        invalid_observations += not observation_valid
        nonincreasing += not timestamp_valid
        side_changes += side_changed
        timestamps.append(timestamp)
        row = {
            "mcu_time_ms": timestamp,
            "test_side": side,
            **dict(zip(field_names, value[2:15] + value[16:24] if diagnostic else value[2:9])),
            "valid_flags": flags,
            "timestamp_valid": int(timestamp_valid),
            "side_changed": int(side_changed),
            "observation_valid": int(observation_valid),
            "valid": int(valid),
        }
        if diagnostic:
            row["target_tracking_valid"] = int(valid)
            row["virtual_thigh_error_rad"] = math.atan2(math.sin(value[5] - value[6]), math.cos(value[5] - value[6])) if valid else None
            row["virtual_shank_error_rad"] = math.atan2(math.sin(value[7] - value[8]), math.cos(value[7] - value[8])) if valid else None
            row["kinematic_shank_rate_rad_s"] = value[20] * value[18] + value[21] * value[19] if (flags & 7) == 7 and timestamp_valid else None
        rows.append(row)
    intervals = [b - a for a, b in zip(timestamps, timestamps[1:]) if b >= a]
    return rows, {
        "frame_count": len(rows), "invalid_frames": invalid,
        "invalid_observation_frames": invalid_observations,
        "nonincreasing_timestamp_frames": nonincreasing,
        "side_change_count": side_changes,
        "test_sides": sorted({row["test_side"] for row in rows}),
        "single_side_recording": bool(rows) and side_changes == 0,
        "timestamp_resolution": "MCU milliseconds; a 500 Hz stream normally has 2 ms intervals",
        "timestamp_limit": "duplicate or backward timestamps remain invalid until the previous maximum is exceeded; no wrap or reset is inferred",
        "period_ms_min": min(intervals) if intervals else None,
        "period_ms_max": max(intervals) if intervals else None,
        "integrity_limit": "ordinary {}-channel JustFloat has no sequence number, frame type or CRC; raw resynchronization is heuristic and cannot prove mode identity, frame loss or payload integrity".format(channels),
        "layout_identity_limit": "requires the matching firmware fingerprint; diagnostic24 rejects legacy 10/16-channel CSV layouts, but raw resynchronization cannot reliably identify mixed layouts; never combine firmware layouts in one recording; a supplied fingerprint is external evidence, not verified from these frames",
        "validity_limit": "valid requires flags {}, increasing time and no side-change boundary; bit5 only reports latest CAN enqueue success, not motor torque application; mixed-side recordings must be split before response analysis".format(hex(required_flags)),
        "observation_mask": hex(observation_mask),
        "target_error_semantics": "wrapped target minus measured angle in firmware virtual coordinates, radians; empty unless target_tracking_valid" if diagnostic else "unavailable in legacy10",
        "kinematic_rate_semantics": "jac_front * front_velocity + jac_rear * rear_velocity, radians/s; requires valid geometry and both feedbacks plus increasing time; Jacobian prediction, not an independent measurement or a polarity calibration" if diagnostic else "unavailable in legacy10",
    }


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def main() -> int:
    args = parse_args()
    if not args.input.is_file():
        raise FileNotFoundError(args.input)
    if args.output.exists():
        raise FileExistsError("拒绝覆盖已有目录: {}".format(args.output))
    if args.config and not args.config.is_file():
        raise FileNotFoundError(args.config)
    args.output.mkdir(parents=True)
    raw = args.input.suffix.lower() in {".bin", ".raw"}
    copied = args.output / ("raw.bin" if raw else "vofa_plus_export.csv")
    shutil.copy2(args.input, copied)
    frames, ignored = read_raw(copied, args.layout) if raw else read_csv(copied, args.layout)
    rows, summary = decode(frames, args.layout)
    config_copy = None
    if args.config:
        config_copy = args.output / "configuration" / args.config.name
        config_copy.parent.mkdir()
        shutil.copy2(args.config, config_copy)
    fields = list(rows[0]) if rows else ["mcu_time_ms", "test_side"]
    with (args.output / "leg_response.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)
    manifest = {
        "protocol": "ordinary-vofa-single-leg-diagnostic-v2" if args.layout == "diagnostic24" else "ordinary-vofa-single-leg-v1",
        "layout": args.layout, "channels": layout_channels(args.layout), "input": args.input.name,
        "raw_copy": copied.name, "input_sha256": sha256(copied),
        "firmware_fingerprint": args.firmware_fingerprint,
        "configuration": config_copy.relative_to(args.output).as_posix() if config_copy else "unknown",
        "configuration_sha256": sha256(config_copy) if config_copy else None,
        "ignored_bytes_or_rows": ignored, "decode": summary,
        "replay_limit": "diagnostic24 仅含选中腿两台电机反馈力矩及其接收年龄，没有同时四电机反馈记录；legacy10 连反馈力矩也没有。两种布局均不满足原四电机 MuJoCo 力矩回放契约，不能将命令或虚拟力矩当作反馈力矩，也不能将 CAN 接收时间当作电机内部生效时间。",
    }
    (args.output / "manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False))
    return 0 if rows else 2


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print("error: {}".format(exc), file=sys.stderr)
        raise SystemExit(2)
