#!/usr/bin/env python3
"""校验 config.yaml 里的 default_dof_pos 与实测电机原始角是否自洽。

用法（原始电机轴角度，顺序 m0..m11，即 test_motor_raw 打印的 Motor Shaft 列）:
    python3 scripts/check_default_dof_pos.py -r -3.004692 8.856267 3.326637 5.650623 \
        -5.821279 3.109003 0.150714 -0.797097 4.630907 3.916262 -0.349173 11.869779

也可以直接吃 test_motor_raw 的输出文件:
    python3 scripts/check_default_dof_pos.py --from-log /tmp/motor.txt
"""
from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "policy/wdblh/himloco/config.yaml"
CFG_KEY = "wdblh/himloco"
GEAR = 6.33
NAMES = ["FL_hip", "FL_thigh", "FL_calf", "FR_hip", "FR_thigh", "FR_calf",
         "RL_hip", "RL_thigh", "RL_calf", "RR_hip", "RR_thigh", "RR_calf"]


def raw_to_joint(raw, mapping, direction):
    """joint[i] = raw[mapping[i]] / GEAR * direction[i]"""
    return [raw[mapping[i]] / GEAR * direction[i] for i in range(12)]


def parse_log(path: Path):
    """从 test_motor_raw 输出里抓最后一帧 Motor Shaft 的 Pos(rad)。"""
    rows = {}
    for line in path.read_text(errors="ignore").splitlines():
        m = re.match(r"\s*(\d{1,2})\s+\S+\s+\d+\s+([+-]?\d+\.\d+)", line)
        if m:
            idx, pos = int(m.group(1)), float(m.group(2))
            if 0 <= idx <= 11:
                rows.setdefault(idx, []).append(pos)
    if len(rows) != 12:
        sys.exit(f"只解析到 {len(rows)} 个电机，需要 12 个")
    return [rows[i][-1] for i in range(12)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-r", "--raw", nargs=12, type=float, metavar="A")
    ap.add_argument("--from-log", type=Path)
    ap.add_argument("--yaml", type=Path, default=CONFIG)
    args = ap.parse_args()

    if args.from_log:
        raw = parse_log(args.from_log)
    elif args.raw:
        raw = args.raw
    else:
        ap.error("要么给 -r 12 个数，要么给 --from-log")

    cfg = yaml.safe_load(open(args.yaml))[CFG_KEY]
    default = cfg["default_dof_pos"]
    mapping = cfg["joint_mapping"]
    direction = cfg["motor_direction"]

    measured = raw_to_joint(raw, mapping, direction)

    print(f"config: {args.yaml}  (gear={GEAR})\n")
    print(f"{'J#':>2}  {'joint':10s} {'default':>9s} {'measured':>9s} {'diff':>8s}  {'raw[m]':>9s}")
    worst = 0.0
    for i in range(12):
        d = measured[i] - default[i]
        worst = max(worst, abs(d))
        print(f"{i:2d}  {NAMES[i]:10s} {default[i]:+9.4f} {measured[i]:+9.4f} {d:+8.4f}  "
              f"{raw[mapping[i]]:+9.3f}")

    print(f"\n最大偏差 {worst:.4f} rad ({worst*57.2958:.1f} deg)")
    if worst > 0.1:
        print("⚠ 超过 0.1 rad：真机 GetUp 会走插值（有动作），确认此时姿态是否就是目标站姿")
    else:
        print("✓ 在 0.1 rad 以内：GetUp 会直接判定到位（percent_getup=1），不产生动作")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
