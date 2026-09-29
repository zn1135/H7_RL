"""JID1 USB CDC 帧与 500 Hz 关节快照。"""

from __future__ import annotations

import struct
import zlib

MAGIC = b"JID1"
VERSION = 1
MAX_PAYLOAD = 192
STATUS, ARM, SET, STOP, ECHO, STREAM = 1, 2, 3, 4, 5, 6
REPLY, STATE, SAMPLE, ECHO_REPLY = 0x81, 0x82, 0x83, 0x85
MOTOR_NAMES = ("front_left", "rear_left", "front_right", "rear_right")
SAMPLE_COLUMNS = (
    "host_recv_ns", "phase", "frame_seq", "t_sample_ns", "t_usb_rx_ns",
    "t_can_queue_ns", "host_cmd_seq", "flags", "faults", "dropped_samples",
    "selected_motor", *(f"tau_cmd_{n}_nm" for n in MOTOR_NAMES),
    *(f"q_raw_{n}_rad" for n in MOTOR_NAMES),
    *(f"q_zero_{n}_rad" for n in MOTOR_NAMES),
    *(f"dq_{n}_rad_s" for n in MOTOR_NAMES),
    *(f"tau_fb_{n}_nm" for n in MOTOR_NAMES),
    *(f"t_can_rx_{n}_ns" for n in MOTOR_NAMES),
    "leg_len_left_m", "leg_angle_left_rad", "leg_len_right_m",
    "leg_angle_right_rad", "thigh_left_rad", "shank_left_rad",
    "thigh_right_rad", "shank_right_rad", "bad_frames",
)


def encode(frame_type: int, sequence: int, payload: bytes = b"") -> bytes:
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("payload too long")
    header = struct.pack("<4sBBHI", MAGIC, VERSION, frame_type, len(payload), sequence)
    body = header + payload
    return body + struct.pack("<I", zlib.crc32(body))


class Decoder:
    def __init__(self):
        self.buffer = bytearray()
        self.bad_frames = 0

    def feed(self, data: bytes):
        self.buffer.extend(data)
        result = []
        while len(self.buffer) >= 4:
            if self.buffer[:4] != MAGIC:
                del self.buffer[0]
                continue
            if len(self.buffer) < 12:
                break
            magic, version, kind, length, sequence = struct.unpack_from(
                "<4sBBHI", self.buffer
            )
            if version != VERSION or length > MAX_PAYLOAD:
                self.bad_frames += 1
                del self.buffer[0]
                continue
            total = 16 + length
            if len(self.buffer) < total:
                break
            expected = struct.unpack_from("<I", self.buffer, total - 4)[0]
            if zlib.crc32(self.buffer[: total - 4]) != expected:
                self.bad_frames += 1
                del self.buffer[0]
                continue
            result.append((kind, sequence, bytes(self.buffer[12 : total - 4])))
            del self.buffer[:total]
        return result


def decode_state(payload: bytes) -> dict:
    if len(payload) != 148:
        raise ValueError(f"state length {len(payload)} != 148")
    data = dict(zip(("machine", "limits_approved", "physical_permit", "armed",
                     "latched", "s1", "s2", "motor_enabled"), payload[:8]))
    data.update(zip(("faults", "rx_overflow", "bad_frames"),
                    struct.unpack_from("<III", payload, 8)))
    data["source_sha256"] = payload[20:84].decode("ascii")
    for name, start in (("torque_max_nm", 84), ("speed_max_rad_s", 100),
                        ("pos_min_rad", 116), ("pos_max_rad", 132)):
        data[name] = list(struct.unpack_from("<4f", payload, start))
    return data


def decode_sample(payload: bytes) -> dict:
    if len(payload) != 192:
        raise ValueError(f"sample length {len(payload)} != 192")
    value = struct.unpack_from("<QQQIIII", payload)
    data = dict(zip(("t_sample_ns", "t_usb_rx_ns", "t_can_queue_ns",
                     "host_cmd_seq", "flags", "faults", "dropped_samples"), value))
    data["selected_motor"] = payload[40]
    for prefix, offset in (("tau_cmd", 44), ("q_raw", 60), ("q_zero", 76),
                           ("dq", 92), ("tau_fb", 108)):
        for motor, x in zip(MOTOR_NAMES, struct.unpack_from("<4f", payload, offset)):
            unit = "_rad_s" if prefix == "dq" else "_nm" if prefix.startswith("tau") else "_rad"
            data[f"{prefix}_{motor}{unit}"] = x
    for motor, x in zip(MOTOR_NAMES, struct.unpack_from("<4Q", payload, 124)):
        data[f"t_can_rx_{motor}_ns"] = x
    for column, x in zip(SAMPLE_COLUMNS[-9:-1], struct.unpack_from("<8f", payload, 156)):
        data[column] = x
    data["bad_frames"] = struct.unpack_from("<I", payload, 188)[0]
    return data
