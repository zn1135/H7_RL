#!/usr/bin/env python3
"""Passively save UART or USB CDC policy trace bytes, then decode without VOFA+."""

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import time

import vofa_trace_decode as trace


def capture(device, output, seconds, port, baud):
    """The device is configured but unopened; all received bytes go to raw.bin."""
    output.mkdir(parents=True, exist_ok=False)
    started = time.monotonic()
    last_status = started
    digest = hashlib.sha256()
    metadata = {
        "port": port, "baud": baud, "format": "8N1", "requested_seconds": seconds,
        "started_utc": datetime.now(timezone.utc).isoformat(),
        "bytes_received": 0, "stop_reason": "duration", "error": None,
    }
    try:
        with (output / "raw.bin").open("xb") as raw:
            device.open()
            started = last_status = time.monotonic()
            print("Receiving {} at {} baud; Ctrl+C stops and decodes.".format(port, baud), flush=True)
            while not seconds or time.monotonic() - started < seconds:
                data = device.read(4096)
                if data:
                    raw.write(data)
                    digest.update(data)
                    metadata["bytes_received"] += len(data)
                now = time.monotonic()
                if now - last_status >= 1.0:
                    raw.flush()
                    print("{:.1f}s: {} bytes saved".format(now - started, metadata["bytes_received"]), flush=True)
                    last_status = now
    except KeyboardInterrupt:
        metadata["stop_reason"] = "interrupted"
    except OSError as error:
        metadata["stop_reason"] = "error"
        metadata["error"] = str(error)
    finally:
        device.close()
        metadata["elapsed_s"] = time.monotonic() - started
        metadata["raw_sha256"] = digest.hexdigest()
        (output / "capture_summary.json").write_text(
            json.dumps(metadata, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    # Decode after reception, so parsing and CSV formatting cannot delay reads.
    frames, ignored = trace.read_raw(output / "raw.bin")
    samples, summary = trace.reconstruct(frames)
    summary["ignored_bytes_or_rows"] = ignored
    trace.write_csv(output / "vofa_trace.csv", samples)
    (output / "summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False), flush=True)
    if metadata["error"]:
        print("Capture error: {}. Saved bytes are retained in {}".format(metadata["error"], output))
        return 1
    if not samples:
        print("No policy observations decoded. Check the COM port, baud and 32-channel policy trace mode.")
        return 1
    print("Saved raw.bin, vofa_trace.csv, summary.json and capture_summary.json in {}".format(output))
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--port", help="UART wireless adapter COM port (or STM32 USB CDC), e.g. COM5")
    source.add_argument("--list-ports", action="store_true", help="list ports without opening them")
    parser.add_argument("--output", type=Path, help="new output directory; existing directories are refused")
    parser.add_argument("--baud", type=int, default=1152000)
    parser.add_argument("--seconds", type=float, default=30.0, help="duration; 0: receive until Ctrl+C")
    args = parser.parse_args(argv)
    if not math.isfinite(args.seconds) or args.seconds < 0 or args.baud <= 0:
        parser.error("seconds must be finite and nonnegative; baud must be positive")
    if args.port and args.output is None:
        parser.error("--output is required with --port")
    try:
        import serial
    except ImportError:
        parser.exit(1, "Install pyserial with: py -3 -m pip install pyserial\n")
    if args.list_ports:
        from serial.tools.list_ports import comports
        for port in sorted(comports(), key=lambda item: item.device):
            print("{}: {} [{}]".format(port.device, port.description, port.hwid))
        return 0

    device = serial.Serial(port=None, baudrate=args.baud, timeout=0.1,
                           bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                           stopbits=serial.STOPBITS_ONE, xonxoff=False,
                           rtscts=False, dsrdtr=False)
    device.dtr = False
    device.rts = False
    device.port = args.port
    try:
        return capture(device, args.output, args.seconds, args.port, args.baud)
    except (OSError, ValueError, OverflowError) as error:
        parser.exit(1, "Capture/decode failed: {}. Any saved raw.bin is retained.\n".format(error))


if __name__ == "__main__":
    raise SystemExit(main())
