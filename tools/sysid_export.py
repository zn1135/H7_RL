#!/usr/bin/env python3
"""
sysid_export — 将 VOFA+ 保存的 sysid 数据文件导出为训练端契约 CSV。

支持两种输入格式：
  1. VOFA+ 导出的文本 CSV（31列浮点，可能有表头、NaN/空行）
  2. 原始二进制流（按 4字节帧尾 00 00 80 7F 扫描重同步）

用法：
  python tools/sysid_export.py input.bin
  python tools/sysid_export.py input.csv --format csv --out data/sysid/chuanliantui/
  python tools/sysid_export.py --selftest
"""

import argparse
import csv
import hashlib
import math
import os
import struct
import sys
import tempfile
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

# ─────────────────────────────────────────────────────────────────────────────
# 常量（与 sysid_log.h 帧布局一致）
# ─────────────────────────────────────────────────────────────────────────────
FRAME_FLOATS = 37
FRAME_TAIL = b"\x00\x00\x80\x7f"  # float32 0x7F800000 = +inf
FRAME_SIZE = FRAME_FLOATS * 4 + len(FRAME_TAIL)  # 152 B
FLOAT_FMT = f"<{FRAME_FLOATS}f"

KIND_CMD_LEG = 1
KIND_CMD_WHEEL = 3
KIND_MARKER = 5

C620_CMD_ID = 0x200
C620_FB_ID_L = 0x201
C620_FB_ID_R = 0x202


# ─────────────────────────────────────────────────────────────────────────────
# 工具函数
# ─────────────────────────────────────────────────────────────────────────────
def ts_ns(hi: float, lo: float) -> int:
    return int(hi) * 1048576 + int(lo)


def _sha256_file(p: Path) -> str:
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for chunk in iter(lambda: f.read(8192), b""):
            h.update(chunk)
    return h.hexdigest()


def sha256_for(paths: List[Path]) -> str:
    h = hashlib.sha256()
    for p in sorted(paths):
        with open(p, "rb") as f:
            for chunk in iter(lambda: f.read(8192), b""):
                h.update(chunk)
    return h.hexdigest()


def fmt_f(v: float, prec: int = 10) -> str:
    if math.isnan(v) or math.isinf(v):
        return ""
    return f"{v:.{prec}g}"


def _fmt_scalar(v: Any) -> str:
    if isinstance(v, float):
        return f"{v:g}"
    return str(v)


# ─────────────────────────────────────────────────────────────────────────────
# 数据类
# ─────────────────────────────────────────────────────────────────────────────
@dataclass
class RunMeta:
    run_idx: int = -1
    test_id: int = 0
    total_phases: int = 0
    valid: bool = True
    reasons: List[str] = field(default_factory=list)
    stats: Optional["RunStats"] = None


@dataclass
class RunStats:
    n_total: int = 0
    n_leg: int = 0
    n_wheel: int = 0
    n_markers: int = 0
    seq_first: int = -1
    seq_last: int = -1
    seq_drops: int = 0
    t_min: int = 0
    t_max: int = 0
    n_nan: int = 0
    events: List[int] = field(default_factory=list)


# ─────────────────────────────────────────────────────────────────────────────
# 帧读取与解析
# ─────────────────────────────────────────────────────────────────────────────
class FrameReader:
    """读取 VOFA+ 二进制或 CSV 格式数据，返回帧列表。"""

    def parse_binary(self, data: bytes) -> List[Dict[str, Any]]:
        frames: List[Dict[str, Any]] = []
        tail = FRAME_TAIL
        i = 0
        n = len(data)
        prev_end = -1

        while i <= n - len(tail):
            pos = data.find(tail, i)
            if pos < 0:
                break
            data_start = pos - FRAME_FLOATS * 4
            if data_start < 0:
                i = pos + 1
                continue
            if data_start < prev_end:
                i = pos + 1
                continue

            chunk = data[data_start : data_start + FRAME_FLOATS * 4]
            try:
                vals = list(struct.unpack(FLOAT_FMT, chunk))
            except struct.error:
                i = pos + 1
                continue

            frames.append(
                {
                    "kind": int(vals[0]) & 0xFF,
                    "seq": int(vals[1]) & 0xFFFF,
                    "phase_or_event": int(vals[2]),
                    "vals": vals,
                }
            )
            prev_end = data_start + FRAME_SIZE
            i = data_start + FRAME_SIZE

        return frames

    def parse_csv_text(self, text: str) -> List[Dict[str, Any]]:
        frames: List[Dict[str, Any]] = []
        lines = text.splitlines()
        for line in lines:
            line = line.strip()
            if not line:
                continue
            sep = "," if "," in line else "\t"
            parts = line.split(sep)
            if len(parts) < FRAME_FLOATS:
                continue
            try:
                vals = [float(x) for x in parts[:FRAME_FLOATS]]
            except ValueError:
                continue
            if any(math.isnan(v) or math.isinf(v) for v in vals[:3]):
                if any(math.isnan(v) for v in vals[:3]):
                    continue
            frames.append(
                {
                    "kind": int(vals[0]) & 0xFF,
                    "seq": int(vals[1]) & 0xFFFF,
                    "phase_or_event": int(vals[2]),
                    "vals": vals,
                }
            )
        return frames

    def detect_format(self, buf: bytes) -> str:
        has_tail = FRAME_TAIL in buf
        if has_tail:
            return "bin"
        printable = sum(1 for b in buf[:2048] if 32 <= b < 127 or b in (9, 10, 13))
        return "csv" if len(buf[:2048]) > 0 and printable / len(buf[:2048]) > 0.7 else "bin"

    def read(
        self, path: str, fmt: str = "auto"
    ) -> List[Dict[str, Any]]:
        buf = Path(path).read_bytes()
        if not buf:
            return []
        if fmt == "auto":
            fmt = self.detect_format(buf)
        if fmt == "bin":
            return self.parse_binary(buf)
        return self.parse_csv_text(buf.decode("utf-8", errors="replace"))


# ─────────────────────────────────────────────────────────────────────────────
# Run 切分与有效性判定
# ─────────────────────────────────────────────────────────────────────────────
class RunSplitter:
    """按标记行和事件切分数据为独立 run，并判定有效性。"""

    def __init__(self, rows: List[Dict[str, Any]]):
        self.rows = rows

    def split(self, out: Any = None) -> List[RunMeta]:
        if out is None:
            out = sys.stdout

        cur: List[Dict[str, Any]] = []
        cur_meta = RunMeta()
        runs: List[RunMeta] = []
        run_counter = 0

        for row in self.rows:
            cur.append(row)
            ev = row["phase_or_event"]

            if ev == -1:
                if cur_meta.run_idx < 0:
                    cur_meta.run_idx = run_counter
                cur_meta.test_id = int(row["vals"][6]) if len(row["vals"]) > 6 else 0

            if ev == -2 or ev == -3:
                if cur_meta.run_idx < 0:
                    cur_meta.run_idx = run_counter
                if len(cur) > 0:
                    self._finalize_run(cur_meta, cur)
                    runs.append(cur_meta)
                cur = []
                cur_meta = RunMeta()
                run_counter += 1

        if cur:
            if cur_meta.run_idx < 0:
                cur_meta.run_idx = run_counter
            self._finalize_run(cur_meta, cur)
            runs.append(cur_meta)

        valid_runs = 0
        for r in runs:
            if r.stats:
                r.stats.n_nan = sum(
                    1
                    for row in r.rows_data
                    if any(math.isnan(v) for v in row["vals"])
                )
                t_ns_list = [
                    ts_ns(row["vals"][3], row["vals"][4])
                    for row in r.rows_data
                    if ts_ns(row["vals"][3], row["vals"][4]) > 0
                ]
                t_rx_list = [
                    ts_ns(row["vals"][21], row["vals"][22])
                    for row in r.rows_data
                    if ts_ns(row["vals"][21], row["vals"][22]) > 0
                ]
                all_t = t_ns_list + t_rx_list
                if all_t:
                    r.stats.t_min = min(all_t)
                    r.stats.t_max = max(all_t)

                prev = -1
                drops = 0
                for row in r.rows_data:
                    s = row["seq"]
                    if prev >= 0 and s > prev + 1:
                        drops += s - prev - 1
                    prev = s
                r.stats.seq_drops = drops
                if r.stats.seq_drops > 0:
                    r.valid = False
                    r.reasons.append(f"seq 不连续（丢 {drops} 帧）")

                if r.stats.n_nan > 0:
                    r.valid = False
                    r.reasons.append(f"{r.stats.n_nan} 行含 NaN")

                events = r.stats.events
                if -3 in events:
                    r.valid = False
                    r.reasons.append("含中止事件 (-3)")
                if -1 in events and -2 not in events and -3 not in events:
                    r.valid = False
                    r.reasons.append("有开始 (-1) 无结束 (-2)")

            if r.valid:
                valid_runs += 1

        out.write(
            f"  切分出 {len(runs)} 个 run：{valid_runs} 有效，"
            f"{len(runs) - valid_runs} 无效\n"
        )
        return runs

    def _finalize_run(
        self,
        meta: RunMeta,
        rows: List[Dict[str, Any]],
    ):
        meta.rows_data = rows
        s = RunStats()
        s.n_total = len(rows)

        events_set: List[int] = []
        for row in rows:
            ev = row["phase_or_event"]
            k = row["kind"]
            if k == KIND_CMD_LEG:
                s.n_leg += 1
            elif k == KIND_CMD_WHEEL:
                s.n_wheel += 1
            elif k == KIND_MARKER:
                s.n_markers += 1
            if ev < 0:
                events_set.append(ev)
        s.events = events_set

        if rows:
            s.seq_first = rows[0]["seq"]
            s.seq_last = rows[-1]["seq"]

        meta.stats = s

        if meta.run_idx < 0:
            meta.run_idx = 0
        if meta.total_phases == 0:
            max_phase = 0
            for row in rows:
                p = row["phase_or_event"]
                if p > max_phase:
                    max_phase = p
            meta.total_phases = max_phase


# ─────────────────────────────────────────────────────────────────────────────
# 导出
# ─────────────────────────────────────────────────────────────────────────────
LEG_COLUMNS = [
    "kind",
    "seq",
    "t_cmd_can_tx_ns",
    "tau_lf0_Nm",
    "tau_lf00_Nm",
    "leg_length_left_m",
    "leg_pitch_left_rad",
    "tau_rf0_Nm",
    "tau_rf00_Nm",
    "leg_length_right_m",
    "leg_pitch_right_rad",
]

WHL_CMD_COLUMNS = [
    "kind",
    "seq",
    "run_id",
    "test_id",
    "phase",
    "t_cmd_can_tx_ns",
    "sequence_id",
    "can_id",
    "cmd_test_wheel_raw",
]

WHL_FB_COLUMNS = [
    "kind",
    "seq",
    "run_id",
    "test_id",
    "phase",
    "t_fb_can_rx_ns",
    "can_id",
    "motor_name",
    "ecd_raw",
    "speed_rpm",
    "torque_current_raw",
    "temperature_c",
]


class SysidExporter:
    def __init__(self, out_dir: str):
        self.out_root = Path(out_dir)
        self.today = datetime.now().strftime("%Y-%m-%d")
        self.device = "chuanliantui"
        self.runs: List[RunMeta] = []

    def run(
        self, rows: List[Dict[str, Any]], out: Any = None
    ) -> int:
        if out is None:
            out = sys.stdout
        if not rows:
            out.write("  没有有效行，跳过导出\n")
            return 0

        out.write("Step 1: 切分 run...\n")
        splitter = RunSplitter(rows)
        self.runs = splitter.split(out)

        out.write("Step 2: 创建目录...\n")
        day_dir = self.out_root / self.device / self.today
        day_dir.mkdir(parents=True, exist_ok=True)

        out.write("Step 3: 导出 CSV...\n")
        for rmeta in self.runs:
            if rmeta.valid:
                self._export_run(day_dir, rmeta, out)
            else:
                self._export_run(day_dir, rmeta, out)

        out.write("Step 4: 写顶层文件...\n")
        all_paths = self._write_toplevel(day_dir, out)

        out.write("Step 5: 写校验和...\n")
        cksum = sha256_for(all_paths)
        ck_path = day_dir / "checksum.sha256"
        ck_path.write_text(cksum + "\n")
        out.write(f"  {ck_path.relative_to(self.out_root)}: {cksum[:16]}...\n")

        out.write(f"\n导出完成：{len(self.runs)} 个 run\n")
        return len(self.runs)

    def _export_run(
        self,
        day_dir: Path,
        rmeta: RunMeta,
        out: Any,
    ):
        rows = rmeta.rows_data
        leg_rows = [r for r in rows if r["kind"] == KIND_CMD_LEG]
        whl_rows = [r for r in rows if r["kind"] == KIND_CMD_WHEEL]

        if not leg_rows and not whl_rows:
            out.write(f"  run {rmeta.run_idx}: 无数据行，跳过\n")
            return

        if leg_rows:
            d = day_dir / f"joint-torque-{rmeta.run_idx}"
            d.mkdir(parents=True, exist_ok=True)
            self._write_leg_csv(d, leg_rows)
            self._write_run_readme(d, rmeta, "leg")
            self._write_run_checksum(d)

        if whl_rows:
            d = day_dir / f"wheel-l-{rmeta.run_idx}"
            d.mkdir(parents=True, exist_ok=True)
            self._write_wheel_csv_cmd(d, whl_rows, rmeta, "l")
            self._write_wheel_csv_fb(d, whl_rows, rmeta, "l")
            self._write_run_readme(d, rmeta, "wheel")
            self._write_run_checksum(d)

            d2 = day_dir / f"wheel-r-{rmeta.run_idx}"
            d2.mkdir(parents=True, exist_ok=True)
            self._write_wheel_csv_cmd(d2, whl_rows, rmeta, "r")
            self._write_wheel_csv_fb(d2, whl_rows, rmeta, "r")
            self._write_run_readme(d2, rmeta, "wheel")
            self._write_run_checksum(d2)

    def _write_leg_csv(self, d: Path, rows: List[Dict[str, Any]]):
        csv_path = d / "joint_torque_snapshot.csv"
        with open(csv_path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(LEG_COLUMNS)
            for r in rows:
                v = r["vals"]
                t = ts_ns(v[3], v[4])
                w.writerow(
                    [
                        int(v[0]),
                        int(v[1]),
                        t,
                        fmt_f(v[5]),
                        fmt_f(v[6]),
                        fmt_f(v[17]),
                        fmt_f(v[18]),
                        fmt_f(v[7]),
                        fmt_f(v[8]),
                        fmt_f(v[19]),
                        fmt_f(v[20]),
                    ]
                )

    def _write_wheel_csv_cmd(
        self,
        d: Path,
        rows: List[Dict[str, Any]],
        rmeta: RunMeta,
        side: str,
    ):
        csv_path = d / "c620_command_raw.csv"
        idx = 0 if side == "l" else 1
        with open(csv_path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(WHL_CMD_COLUMNS)
            for r in rows:
                v = r["vals"]
                t = ts_ns(v[3], v[4])
                row_base = [
                    int(v[0]),
                    int(v[1]),
                    rmeta.run_idx,
                    rmeta.test_id,
                    int(v[2]),
                    t,
                    int(v[1]),
                    C620_CMD_ID,
                    fmt_f(v[5 + idx]),
                ]
                w.writerow(row_base)

    def _write_wheel_csv_fb(
        self,
        d: Path,
        rows: List[Dict[str, Any]],
        rmeta: RunMeta,
        side: str,
    ):
        csv_path = d / "c620_feedback_raw.csv"
        can_id = C620_FB_ID_L if side == "l" else C620_FB_ID_R
        motor_name = "lwheel" if side == "l" else "rwheel"
        idx = 0 if side == "l" else 1

        with open(csv_path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(WHL_FB_COLUMNS)
            for r in rows:
                v = r["vals"]
                t_rx = ts_ns(v[21], v[22])
                w.writerow(
                    [
                        int(v[0]),
                        int(v[1]),
                        rmeta.run_idx,
                        rmeta.test_id,
                        int(v[2]),
                        t_rx,
                        can_id,
                        motor_name,
                        fmt_f(v[23 + idx]),
                        fmt_f(v[25 + idx]),
                        fmt_f(v[27 + idx]),
                        "",
                    ]
                )

    def _write_run_readme(
        self,
        d: Path,
        rmeta: RunMeta,
        kind: str,
    ):
        s = rmeta.stats
        md_lines: List[str] = []

        md_lines.append(f"# Run {rmeta.run_idx} 概况\n")
        md_lines.append(f"- **类型**：{'腿力矩' if kind == 'leg' else '轮电流'}\n")
        md_lines.append(
            f"- **状态**：{'[OK] 有效' if rmeta.valid else '[!!] 无效'}\n"
        )
        md_lines.append(f"- **test_id**：{rmeta.test_id}\n")
        md_lines.append(f"- **总段数**：{rmeta.total_phases}\n")
        md_lines.append(f"- **总帧数**：{s.n_total}\n")

        dur_ns = s.t_max - s.t_min
        dur_s = dur_ns / 1e9
        md_lines.append(f"- **时间跨度**：{dur_ns} ns ({dur_s:.3f} s)\n")

        md_lines.append(f"- **seq 范围**：{s.seq_first} ~ {s.seq_last}\n")
        md_lines.append(f"- **丢帧数**：{s.seq_drops}\n")

        events_str = ", ".join(str(e) for e in s.events) if s.events else "无"
        md_lines.append(f"- **事件**：{events_str}\n")

        md_lines.append(f"- **NaN 行数**：{s.n_nan}\n")

        md_lines.append(f"- **腿命令行数**：{s.n_leg}\n")
        md_lines.append(f"- **轮命令行数**：{s.n_wheel}\n")
        md_lines.append(f"- **标记行数**：{s.n_markers}\n")

        if not rmeta.valid:
            md_lines.append("\n## 有效性判定\n\n")
            for reason in rmeta.reasons:
                md_lines.append(f"- [!!] {reason}\n")

        md_lines.append("\n## 温度说明\n\n")
        md_lines.append("本帧未采集温度数据（temperature_c 列为空）。\n")

        readme_path = d / "README.md"
        readme_path.write_text("".join(md_lines))

    def _write_run_checksum(self, d: Path):
        files = sorted(f for f in d.iterdir() if f.name != "checksum.sha256")
        cksum = sha256_for(files)
        (d / "checksum.sha256").write_text(cksum + "\n")

    def _write_toplevel(
        self,
        day_dir: Path,
        out: Any,
    ) -> List[Path]:
        all_files: List[Path] = []

        manifest = day_dir / "manifest.yaml"
        lines = [
            "# sysid manifest — 自动生成，请手动填写 <待填> 项\n",
            f"timestamp: {self.today}\n",
            f"device: {self.device}\n",
            "frame_format: justfloat32_128b_500hz\n",
            "sample_rate_hz: 500\n",
            "\n",
            "# 安全限值（必须在实机签字后填写）\n",
            "limits:\n",
            "  dm_pos_max: <待填>\n",
            "  dm_vel_max: <待填>\n",
            "  dm_trq_max: <待填>\n",
            "  dji_current_max_raw: <待填>\n",
            "\n",
            "runs:\n",
        ]
        for rmeta in self.runs:
            s = rmeta.stats
            typ = "leg" if s.n_leg > s.n_wheel else "wheel"
            lines.extend(
                [
                    f"  - index: {rmeta.run_idx}\n",
                    f"    test_id: {rmeta.test_id}\n",
                    f"    type: {typ}\n",
                    f"    valid: {'true' if rmeta.valid else 'false'}\n",
                    f"    n_frames: {s.n_total}\n",
                    f"    seq_drops: {s.seq_drops}\n",
                ]
            )
            if not rmeta.valid:
                lines.append("    invalid_reasons:\n")
                for reason in rmeta.reasons:
                    lines.append(f"      - \"{reason}\"\n")
        manifest.write_text("".join(lines))
        all_files.append(manifest)
        out.write(f"  {manifest.relative_to(self.out_root)}\n")

        valid_cnt = sum(1 for r in self.runs if r.valid)
        inv_cnt = len(self.runs) - valid_cnt
        total_frames = sum(r.stats.n_total for r in self.runs)
        total_drops = sum(r.stats.seq_drops for r in self.runs)

        readme = day_dir / "README.md"
        md_lines = [
            "# sysid 数据采集总览\n\n",
            f"- **日期**：{self.today}\n",
            f"- **设备**：{self.device}\n",
            f"- **总 run 数**：{len(self.runs)}\n",
            f"  - 有效：{valid_cnt}\n",
            f"  - 无效：{inv_cnt}\n",
            f"- **总帧数**：{total_frames}\n",
            f"- **总丢帧**：{total_drops}\n",
            "\n",
            "## Run 清单\n\n",
            "| run | 类型 | test_id | 状态 | 帧数 | 丢帧 |\n",
            "|-----|------|---------|------|------|------|\n",
        ]
        for rmeta in self.runs:
            s = rmeta.stats
            typ = "leg" if s.n_leg > s.n_wheel else "wheel"
            status = "[OK]" if rmeta.valid else "[!!]"
            md_lines.append(
                f"| {rmeta.run_idx} | {typ} | {rmeta.test_id} "
                f"| {status} | {s.n_total} | {s.seq_drops} |\n"
            )
        if inv_cnt > 0:
            md_lines.append("\n## 无效 Run 详情\n\n")
            for rmeta in self.runs:
                if not rmeta.valid:
                    md_lines.append(f"### Run {rmeta.run_idx}\n\n")
                    for reason in rmeta.reasons:
                        md_lines.append(f"- {reason}\n")
        readme.write_text("".join(md_lines))
        all_files.append(readme)
        out.write(f"  {readme.relative_to(self.out_root)}\n")

        return all_files


# ─────────────────────────────────────────────────────────────────────────────
# 内置自测 (--selftest)
# ─────────────────────────────────────────────────────────────────────────────
class SelfTest:
    @staticmethod
    def _mk(kind, seq, phase, tau, pos, vel, ll, lp, whl):
        v = [0.0] * FRAME_FLOATS
        v[0] = float(kind)
        v[1] = float(seq)
        v[2] = float(phase)
        for i in range(4):
            v[5 + i] = tau[i]
            v[9 + i] = pos[i]
            v[13 + i] = vel[i]
        v[17] = ll[0]
        v[18] = lp[0]
        v[19] = ll[1]
        v[20] = lp[1]
        t_ns_val = (kind * 1000000 + seq * 10000) & 0xFFFFFFFFFF
        v[3] = float(t_ns_val >> 20)
        v[4] = float(t_ns_val & 0xFFFFF)
        for i in range(2):
            v[21 + i] = v[3 + i]
        v[23] = whl[0]
        v[24] = whl[1]
        v[25] = whl[2]
        v[26] = whl[3]
        v[27] = whl[4]
        v[28] = whl[5]
        return struct.pack(FLOAT_FMT, *v) + FRAME_TAIL

    @staticmethod
    def _mk_bins():
        tau1 = [1.5, 2.0, -1.5, -2.0]
        pos1 = [0.1, 0.2, -0.1, -0.2]
        vel1 = [1.0, 2.0, -1.0, -2.0]
        whl1 = [1000.0, 2000.0, 500.0, 600.0, 1000.0, 1200.0]

        tau2 = [0.5, 1.0, -0.5, -1.0]
        pos2 = [0.15, 0.25, -0.15, -0.25]
        vel2 = [0.5, 1.0, -0.5, -1.0]
        whl2 = [1500.0, 2500.0, 300.0, 400.0, 800.0, 900.0]

        tau3 = [2.0, 3.0, -2.0, -3.0]
        pos3 = [0.2, 0.3, -0.2, -0.3]
        vel3 = [2.0, 3.0, -2.0, -3.0]
        whl3 = [2000.0, 3000.0, 700.0, 800.0, 1500.0, 1600.0]

        mk = SelfTest._mk
        bins = [
            b"\xff" * 5
            + mk(KIND_MARKER, 0, -1, tau1, pos1, vel1, [0.2, 0.2], [0.1, 0.1], whl1)
            + mk(KIND_CMD_LEG, 1, 0, tau1, pos1, vel1, [0.2, 0.2], [0.1, 0.1], whl1)
            + mk(
                KIND_CMD_WHEEL, 2, 0, tau2, pos2, vel2, [0.25, 0.25], [0.15, 0.15], whl2
            )
            + mk(
                KIND_CMD_WHEEL, 5, 1, tau2, pos2, vel2, [0.25, 0.25], [0.15, 0.15], whl2
            )
            + mk(KIND_CMD_LEG, 6, 1, tau3, pos3, vel3, [0.3, 0.3], [0.2, 0.2], whl3)
            + mk(KIND_MARKER, 7, -2, tau3, pos3, vel3, [0.3, 0.3], [0.2, 0.2], whl3)
        ]
        return b"".join(bins)

    @staticmethod
    def run(out: Any = None) -> bool:
        if out is None:
            out = sys.stdout

        out.write("=" * 60 + "\n")
        out.write("sysid_export 自测开始\n")
        out.write("=" * 60 + "\n\n")

        tmp = Path(tempfile.mkdtemp(prefix="sysid_test_"))
        out.write(f"临时目录：{tmp}\n")

        try:
            out.write("\n--- 测试 1：二进制帧解析 ---\n")
            bins = SelfTest._mk_bins()
            reader = FrameReader()
            rows = reader.parse_binary(bins)
            assert len(rows) == 6, f"期望 6 行，实际 {len(rows)}"
            out.write(f"  解析帧数：{len(rows)}\n")

            for i, (ek, es) in enumerate(
                [
                    (KIND_MARKER, 0),
                    (KIND_CMD_LEG, 1),
                    (KIND_CMD_WHEEL, 2),
                    (KIND_CMD_WHEEL, 5),
                    (KIND_CMD_LEG, 6),
                    (KIND_MARKER, 7),
                ]
            ):
                assert rows[i]["kind"] == ek, f"行 {i} kind：期望 {ek}，实际 {rows[i]['kind']}"
                assert rows[i]["seq"] == es, f"行 {i} seq：期望 {es}，实际 {rows[i]['seq']}"

            assert abs(rows[1]["vals"][5] - 1.5) < 1e-5
            assert abs(rows[1]["vals"][17] - 0.2) < 1e-5
            assert abs(rows[2]["vals"][23] - 1500.0) < 1.0
            out.write("  [OK] 二进制解析正确\n")

            out.write("\n--- 测试 2：run 切分 ---\n")
            splitter = RunSplitter(rows)
            runs = splitter.split(out)
            assert len(runs) == 1, f"期望 1 个 run，实际 {len(runs)}"
            r0 = runs[0]
            assert r0.run_idx == 0
            assert r0.valid is False, "有 seq gap 应判无效"
            assert r0.stats.seq_drops == 2, f"期望丢 2 帧，实际 {r0.stats.seq_drops}"
            assert r0.stats.n_leg == 2, f"期望 2 条腿行，实际 {r0.stats.n_leg}"
            assert r0.stats.n_wheel == 2, f"期望 2 条轮行，实际 {r0.stats.n_wheel}"
            out.write("  [OK] run 切分正确\n")

            out.write("\n--- 测试 3：完整导出 ---\n")
            export_dir = tmp / "export"
            exp = SysidExporter(str(export_dir))
            exp.run(rows, out)

            day_dir = export_dir / "chuanliantui" / exp.today
            assert day_dir.exists(), "日期目录不存在"

            leg_csv = day_dir / "joint-torque-0" / "joint_torque_snapshot.csv"
            assert leg_csv.exists(), "leg CSV 不存在"
            with open(leg_csv) as f:
                reader_csv = csv.DictReader(f)
                leg_data = list(reader_csv)
            assert len(leg_data) == 2, f"leg CSV 期望 2 行，实际 {len(leg_data)}"
            assert abs(float(leg_data[0]["tau_lf0_Nm"]) - 1.5) < 1e-4, f"tau_lf0 期望 ~1.5，实际 {leg_data[0]['tau_lf0_Nm']}"
            assert abs(float(leg_data[0]["leg_length_left_m"]) - 0.2) < 1e-4
            out.write(f"  [OK] leg CSV：{len(leg_data)} 行，数据正确\n")

            whl_cmd = day_dir / "wheel-l-0" / "c620_command_raw.csv"
            assert whl_cmd.exists(), "wheel cmd CSV 不存在"
            with open(whl_cmd) as f:
                cmd_data = list(csv.DictReader(f))
            assert len(cmd_data) == 2, f"wheel cmd 期望 2 行，实际 {len(cmd_data)}"
            out.write(f"  [OK] wheel cmd CSV：{len(cmd_data)} 行\n")

            whl_fb = day_dir / "wheel-l-0" / "c620_feedback_raw.csv"
            assert whl_fb.exists(), "wheel fb CSV 不存在"
            with open(whl_fb) as f:
                fb_data = list(csv.DictReader(f))
            assert len(fb_data) == 2
            assert fb_data[0]["can_id"] == str(C620_FB_ID_L)
            assert fb_data[0]["motor_name"] == "lwheel"
            assert fb_data[0]["temperature_c"] == "", "温度应为空"
            out.write(f"  [OK] wheel fb CSV：{len(fb_data)} 行，温度为空\n")

            whl_r = day_dir / "wheel-r-0" / "c620_feedback_raw.csv"
            assert whl_r.exists(), "wheel-r 目录不存在"
            with open(whl_r) as f:
                fb_r = list(csv.DictReader(f))
            assert fb_r[0]["can_id"] == str(C620_FB_ID_R)
            assert fb_r[0]["motor_name"] == "rwheel"
            out.write("  [OK] wheel-r CSV：can_id/电机名正确\n")

            assert (day_dir / "manifest.yaml").exists()
            assert (day_dir / "README.md").exists()
            assert (day_dir / "checksum.sha256").exists()
            assert (day_dir / "joint-torque-0" / "README.md").exists()
            assert (day_dir / "joint-torque-0" / "checksum.sha256").exists()
            assert (day_dir / "wheel-l-0" / "README.md").exists()
            assert (day_dir / "wheel-l-0" / "checksum.sha256").exists()
            out.write("  [OK] 所有目录、文件齐全\n")

            out.write("\n" + "=" * 60 + "\n")
            out.write("自测全部通过 [OK]\n")
            out.write("=" * 60 + "\n")
            return True

        except Exception as e:
            out.write(f"\n自测失败 [ERR]：{e}\n")
            import traceback
            traceback.print_exc(file=out)
            return False
        finally:
            import shutil
            shutil.rmtree(tmp, ignore_errors=True)


# ─────────────────────────────────────────────────────────────────────────────
# 主入口
# ─────────────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(
        description="将 VOFA+ 保存的 sysid 数据导出为训练端契约 CSV"
    )
    parser.add_argument("input", nargs="?", help="输入文件路径（VOFA+ CSV 或二进制）")
    parser.add_argument(
        "--format",
        choices=["auto", "csv", "bin"],
        default="auto",
        help="输入格式（默认自动检测）",
    )
    parser.add_argument(
        "--out",
        default="data/sysid/chuanliantui",
        help="输出根目录（默认 data/sysid/chuanliantui）",
    )
    parser.add_argument(
        "--selftest",
        action="store_true",
        help="运行内置自测（合成帧 → 完整导出 → 断言）",
    )

    args = parser.parse_args()

    if args.selftest:
        ok = SelfTest.run()
        sys.exit(0 if ok else 1)

    if not args.input:
        parser.error("请指定输入文件，或使用 --selftest 运行自测")

    if not os.path.isfile(args.input):
        print(f"错误：文件不存在 — {args.input}", file=sys.stderr)
        sys.exit(1)

    print(f"读取：{args.input}")
    buf = Path(args.input).read_bytes()
    if not buf:
        print("文件为空", file=sys.stderr)
        sys.exit(1)

    fmt = args.format
    if fmt == "auto":
        fmt = FrameReader().detect_format(buf)
    print(f"格式：{fmt}")

    reader = FrameReader()
    if fmt == "bin":
        rows = reader.parse_binary(buf)
    else:
        rows = reader.parse_csv_text(buf.decode("utf-8", errors="replace"))

    print(f"解析行数：{len(rows)}")
    if not rows:
        print("没有有效帧", file=sys.stderr)
        sys.exit(1)

    n_invalid = sum(
        1
        for r in rows
        if any(math.isnan(v) or math.isinf(v) for v in r["vals"][:3])
    )
    if n_invalid:
        print(f"  注意：{n_invalid} 行含 NaN/Inf")

    exporter = SysidExporter(args.out)
    n = exporter.run(rows)
    print(f"校验和：{sha256_for([Path(args.out) / exporter.device / exporter.today / 'checksum.sha256'])[:16]}...")
    print("完成")


if __name__ == "__main__":
    main()
