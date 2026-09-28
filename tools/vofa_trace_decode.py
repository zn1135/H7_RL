#!/usr/bin/env python3
"""Decode 32-channel VOFA JustFloat policy trace from a raw serial dump or VOFA CSV."""

import argparse
import csv
import json
import struct
from pathlib import Path


CHANNELS = 32
TAIL = b"\x00\x00\x80\x7f"
SEQ_MASK = 0xFFFFFF
OBS_NAMES = (
    "gyro_x", "gyro_y", "gyro_z", "gravity_x", "gravity_y", "gravity_z",
    "cmd_vx", "cmd_yaw_rate", "cmd_height", "left_thigh", "left_shank",
    "right_thigh", "right_shank", "left_thigh_vel", "left_shank_vel",
    "left_wheel_vel", "right_thigh_vel", "right_shank_vel", "right_wheel_vel",
    "last_action_0", "last_action_1", "last_action_2", "last_action_3",
    "last_action_4", "last_action_5",
)
AUX_NAMES = (
    "imu_gyro_rad_s_x", "imu_gyro_rad_s_y", "imu_gyro_rad_s_z", "quat_w", "quat_x", "quat_y",
    "quat_z", "acc_g_x", "acc_g_y", "acc_g_z", "euler_pitch",
    "euler_roll", "euler_yaw", "action_0", "action_1", "action_2",
    "action_3", "action_4", "action_5", "dm_cmd_0", "dm_cmd_1",
    "dm_cmd_2", "dm_cmd_3", "wheel_cmd_0", "wheel_cmd_1", "inference_us",
)
STATUS_BITS = (
    "engaged", "obs_valid", "history_ready", "rl_ready", "imu_online",
    "motor_enabled", "fallen", "policy_ready", "fault_imu", "fault_rc",
    "fault_motor", "fault_can", "fault_action", "dm_sent", "dji_sent",
    "rc_online", "dm_0_online", "dm_1_online", "dm_2_online", "dm_3_online",
    "reserved_20", "reserved_21", "dji_0_online", "dji_1_online",
)


def decode_header(values):
    if len(values) != CHANNELS:
        raise ValueError("expected 32 channels")
    kind = int(values[0])
    seq = int(values[1])
    parts = tuple(int(values[i]) for i in (2, 3, 4))
    flags = int(values[5])
    if values[0] != kind or kind < 0 or kind > 6:
        raise ValueError("invalid frame kind")
    if values[1] != seq or seq < 0 or seq > SEQ_MASK:
        raise ValueError("invalid sequence")
    if any(values[i] != parts[i - 2] or parts[i - 2] < 0 or parts[i - 2] > 65535
           for i in (2, 3, 4)):
        raise ValueError("invalid timestamp")
    if values[5] != flags or flags < 0 or flags > SEQ_MASK:
        raise ValueError("invalid flags")
    return kind, seq, parts[0] | parts[1] << 16 | parts[2] << 32, flags


def read_raw(path):
    data = path.read_bytes()
    cursor = 0
    ignored = 0
    frames = []
    while True:
        end = data.find(TAIL, cursor)
        if end < 0:
            ignored += len(data) - cursor
            break
        if end - cursor < CHANNELS * 4:
            ignored += end + len(TAIL) - cursor
            cursor = end + len(TAIL)
            continue
        start = end - CHANNELS * 4
        ignored += start - cursor
        values = struct.unpack("<32f", data[start:end])
        try:
            decode_header(values)
        except (ValueError, OverflowError):
            ignored += end + len(TAIL) - start
        else:
            frames.append(values)
        cursor = end + len(TAIL)
    return frames, ignored


def read_csv(path):
    frames = []
    ignored = 0
    with path.open(newline="", encoding="utf-8-sig") as stream:
        for row in csv.reader(stream):
            if len(row) < CHANNELS:
                ignored += 1
                continue
            try:
                values = tuple(float(value) for value in row[-CHANNELS:])
                decode_header(values)
            except (ValueError, OverflowError):
                ignored += 1
                continue
            frames.append(values)
    return frames, ignored


def reconstruct(frames):
    samples = []
    by_seq = {}
    orphan = 0
    for values in frames:
        kind, seq, time_us, flags = decode_header(values)
        if kind == 0:
            sample = {
                "seq": seq, "obs_time_us": time_us, "flags": flags,
                "obs": tuple(values[6:31]), "sensor_time_ms24": int(values[31]),
                "aux": None, "history_parts": {}, "drop_count": None,
            }
            samples.append(sample)
            by_seq[seq] = sample
            continue
        sample = by_seq.get(seq)
        if sample is None:
            orphan += 1
            continue
        if kind == 1:
            sample["aux"] = (time_us, tuple(values[6:32]))
        else:
            sample["history_parts"][kind - 2] = tuple(values[6:31])
            sample["drop_count"] = int(values[31])

    previous_history = None
    previous_seq = None
    gaps = 0
    sync_count = 0
    sync_mismatches = 0
    unsynced = 0
    missing_aux = 0
    for sample in samples:
        seq = sample["seq"]
        contiguous = previous_seq is not None and seq == (previous_seq + 1) & SEQ_MASK
        if previous_seq is not None and not contiguous:
            gaps += (seq - previous_seq - 1) & SEQ_MASK
            previous_history = None
        previous_seq = seq
        if sample["aux"] is None:
            missing_aux += 1
        flags = sample["flags"]
        ready = (flags & 0x7) == 0x7
        parts = sample["history_parts"]
        if ready and len(parts) == 5:
            history = tuple(value for i in range(5) for value in parts[i])
            sync_count += 1
            if history[-25:] != sample["obs"]:
                sync_mismatches += 1
            if previous_history is not None and contiguous:
                expected = previous_history[25:] + sample["obs"]
                if history != expected:
                    sync_mismatches += 1
            previous_history = history
        elif ready and previous_history is not None and contiguous:
            previous_history = previous_history[25:] + sample["obs"]
        else:
            previous_history = None
        sample["history"] = previous_history
        if ready and previous_history is None:
            unsynced += 1
    return samples, {
        "frame_count": len(frames), "sample_count": len(samples),
        "missing_sample_count": gaps, "missing_aux_count": missing_aux,
        "orphan_frame_count": orphan, "history_sync_count": sync_count,
        "history_sync_mismatch_count": sync_mismatches,
        "unsynced_sample_count": unsynced,
    }


def write_csv(path, samples):
    columns = ["seq", "obs_time_us", "output_time_us", "sensor_time_ms24",
               "history_synced", "drop_count_sync"]
    columns += list(STATUS_BITS)
    columns += ["obs_" + name for name in OBS_NAMES]
    columns += list(AUX_NAMES)
    columns += ["hist_{}_{}".format(frame, name)
                for frame in range(5) for name in OBS_NAMES]
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(columns)
        for sample in samples:
            aux = sample["aux"]
            history = sample["history"]
            row = [sample["seq"], sample["obs_time_us"],
                   aux[0] if aux else "", sample["sensor_time_ms24"],
                   int(history is not None),
                   sample["drop_count"] if sample["drop_count"] is not None else ""]
            row += [int(bool(sample["flags"] & (1 << bit)))
                    for bit in range(8)]
            row += [int(bool(sample["flags"] & (1 << bit)))
                    for bit in range(8, 24)]
            row += list(sample["obs"])
            row += list(aux[1]) if aux else [""] * len(AUX_NAMES)
            row += list(history) if history else [""] * 125
            writer.writerow(row)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="VOFA raw .bin or exported 32-channel .csv")
    parser.add_argument("output_dir", type=Path, help="new output directory")
    args = parser.parse_args()
    frames, ignored = (read_csv(args.input) if args.input.suffix.lower() == ".csv"
                       else read_raw(args.input))
    samples, summary = reconstruct(frames)
    summary["ignored_bytes_or_rows"] = ignored
    args.output_dir.mkdir(parents=True, exist_ok=False)
    write_csv(args.output_dir / "vofa_trace.csv", samples)
    (args.output_dir / "summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False))


if __name__ == "__main__":
    main()
