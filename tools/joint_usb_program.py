#!/usr/bin/env python3
"""生成 100 Hz 单电机力矩程序；幅值由台架作者提供，板端再次校验。"""

import argparse
import json
import math
from pathlib import Path


def zeros(seconds):
    return [0.0] * round(seconds * 100)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--motor", type=int, required=True, choices=range(4))
    parser.add_argument("--amplitude-nm", type=float, required=True)
    parser.add_argument("--kind", choices=("step", "chirp"), required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--duration-s", type=float, default=8.0,
                        help="chirp 激励段时长")
    parser.add_argument("--start-hz", type=float, help="chirp 起始频率")
    parser.add_argument("--end-hz", type=float, help="chirp 结束频率")
    args = parser.parse_args()
    if args.out.exists():
        parser.error("--out 已存在，拒绝覆盖")
    if not math.isfinite(args.amplitude_nm) or args.amplitude_nm <= 0:
        parser.error("amplitude-nm 必须为正的有限数")
    if args.kind == "step":
        samples = zeros(2)
        for _ in range(3):
            samples += [args.amplitude_nm] * 30 + zeros(1)
            samples += [-args.amplitude_nm] * 30 + zeros(1)
        samples += zeros(2)
    else:
        if (args.start_hz is None or args.end_hz is None
                or not 0 < args.start_hz < args.end_hz <= 10
                or not 0 < args.duration_s <= 30):
            parser.error("chirp 需指定 0 < start-hz < end-hz <= 10 与 0 < duration-s <= 30")
        count = round(args.duration_s * 100)
        sweep = [args.amplitude_nm * math.sin(
            2 * math.pi * (args.start_hz * (i / 100)
                            + (args.end_hz - args.start_hz) * (i / 100)**2
                            / (2 * args.duration_s))) for i in range(count)]
        samples = zeros(2) + sweep + zeros(2)
    args.out.write_text(json.dumps({"sample_hz": 100, "motor_index": args.motor,
                                    "samples_nm": samples}, indent=2), encoding="utf-8")
    print(f"{args.out}: {len(samples)} samples, {len(samples) / 100:.2f} s")


if __name__ == "__main__":
    main()
