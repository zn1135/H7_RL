"""JID1 主机端流式解码与字段布局。"""

import pathlib
import struct
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "tools"))

from joint_usb_protocol import (Decoder, SAMPLE_COLUMNS, STATUS, decode_sample,
                                decode_state, encode)


class JointUsbProtocolTest(unittest.TestCase):
    def test_split_and_corrupt_frame(self):
        valid = encode(STATUS, 7)
        corrupt = bytearray(encode(STATUS, 6))
        corrupt[-1] ^= 0x80
        decoder = Decoder()
        self.assertEqual(decoder.feed(b"noise" + valid[:5]), [])
        self.assertEqual(decoder.feed(valid[5:] + corrupt + valid),
                         [(STATUS, 7, b""), (STATUS, 7, b"")])
        self.assertEqual(decoder.bad_frames, 1)

    def test_state_and_sample_layout(self):
        state = bytearray(148)
        state[1] = 1
        state[20:84] = b"a" * 64
        struct.pack_into("<4f", state, 84, 1, 2, 3, 4)
        self.assertEqual(decode_state(state)["torque_max_nm"], [1, 2, 3, 4])
        sample = bytearray(192)
        struct.pack_into("<QQQIIII", sample, 0, 100, 80, 90, 9, 8, 0, 1)
        struct.pack_into("<4f", sample, 44, 0.5, 0, 0, 0)
        struct.pack_into("<8f", sample, 156, *range(8))
        row = decode_sample(sample)
        self.assertEqual(row["host_cmd_seq"], 9)
        self.assertEqual(row["tau_cmd_front_left_nm"], 0.5)
        self.assertEqual(row["shank_right_rad"], 7)
        self.assertEqual(set(SAMPLE_COLUMNS) - set(row),
                         {"host_recv_ns", "phase", "frame_seq"})


if __name__ == "__main__":
    unittest.main()
