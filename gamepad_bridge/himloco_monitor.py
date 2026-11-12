#!/usr/bin/env python3
"""Monitor Web Gamepad commands in the format consumed by himloco.

This program is intentionally independent from rl_real_wdblh: it only binds
the localhost gamepad UDP port and never opens robot, motor, or IMU devices.
"""

from __future__ import annotations

import argparse
import math
import socket
import struct
import sys
import time
from dataclasses import dataclass


PACKET = struct.Struct("<4sI4fI")
MAGIC = b"GPB1"
DEFAULT_PORT = 9877
TIMEOUT_SECONDS = 1.2
HEIGHT_MIN = 0.18
HEIGHT_MAX = 0.35
HEIGHT_SPEED = 0.1  # m/s, identical to rl_real_wdblh

BUTTON_NAMES = (
    "A", "B", "X", "Y", "LB", "RB", "LT", "RT", "Back", "Start",
    "LS", "RS", "DPadUp", "DPadDown", "DPadLeft", "DPadRight", "Home",
)
BUTTON_DPAD_UP = 1 << 12
BUTTON_DPAD_DOWN = 1 << 13


def clamp_axis(value: float) -> float:
    """Match the original /dev/input/js* linear axis mapping."""
    return max(-1.0, min(1.0, value))


def decode_packet(data: bytes) -> tuple[int, tuple[float, ...], int] | None:
    if len(data) != PACKET.size:
        return None
    magic, sequence, *values = PACKET.unpack(data)
    axes = tuple(values[:4])
    buttons = int(values[4])
    if magic != MAGIC or not all(math.isfinite(axis) for axis in axes):
        return None
    return sequence, axes, buttons


def pressed_button_names(buttons: int) -> list[str]:
    return [name for index, name in enumerate(BUTTON_NAMES)
            if buttons & (1 << index)]


@dataclass
class MonitorState:
    sequence: int = 0
    axes: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)
    buttons: int = 0
    height: float = HEIGHT_MAX
    last_packet_time: float = 0.0

    def connected(self, now: float) -> bool:
        return (self.last_packet_time > 0.0
                and now - self.last_packet_time <= TIMEOUT_SECONDS)

    def update_height(self, elapsed: float, connected: bool) -> None:
        if not connected:
            return
        if self.buttons & BUTTON_DPAD_UP:
            self.height += HEIGHT_SPEED * elapsed
        elif self.buttons & BUTTON_DPAD_DOWN:
            self.height -= HEIGHT_SPEED * elapsed
        self.height = max(HEIGHT_MIN, min(HEIGHT_MAX, self.height))

    def himloco_values(self, connected: bool) -> tuple[list[float], list[float]]:
        if connected:
            left_x = clamp_axis(self.axes[0])
            left_y = clamp_axis(self.axes[1])
            right_x = clamp_axis(self.axes[2])
            commands = [-left_y, -left_x, -right_x * 2.0]
        else:
            commands = [0.0, 0.0, 0.0]

        # First four values of each 46-D himloco observation after YAML scaling:
        # commands_scale=[2.0, 2.0, 0.25], height_scale=2.0.
        observation_head = [
            commands[0] * 2.0,
            commands[1] * 2.0,
            commands[2] * 0.25,
            self.height * 2.0,
        ]
        return commands, observation_head


def format_state(state: MonitorState, now: float) -> str:
    connected = state.connected(now)
    commands, observation_head = state.himloco_values(connected)
    buttons = pressed_button_names(state.buttons) if connected else []
    button_text = ",".join(buttons) if buttons else "None"
    status = "LIVE" if connected else "WAIT/STALE"
    return (
        f"[{status}] seq:{state.sequence:<10d} "
        f"commands:[{commands[0]:+.3f}, {commands[1]:+.3f}, {commands[2]:+.3f}] "
        f"height:[{state.height:.3f}] "
        f"obs[0:4]:[{observation_head[0]:+.3f}, {observation_head[1]:+.3f}, "
        f"{observation_head[2]:+.3f}, {observation_head[3]:+.3f}] "
        f"buttons:[{button_text}]"
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="只监视网页手柄转换后的 himloco 输入，不启动机器人控制",
    )
    parser.add_argument("--bind", default="127.0.0.1", help="UDP 监听地址")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help="UDP 监听端口")
    parser.add_argument("--rate", type=float, default=5.0, help="终端刷新频率 Hz")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.rate <= 0:
        raise SystemExit("--rate 必须大于 0")

    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        receiver.bind((args.bind, args.port))
    except OSError as error:
        receiver.close()
        print(
            f"无法监听 {args.bind}:{args.port}：{error}\n"
            "请先停止 rl_real_wdblh，确保这个端口没有被运动控制程序占用。",
            file=sys.stderr,
        )
        raise SystemExit(1) from error

    receiver.setblocking(False)
    state = MonitorState()
    tick = 1.0 / args.rate
    previous_time = time.monotonic()
    next_print = previous_time

    print(f"正在监听 UDP {args.bind}:{args.port}，不会启动机器人控制（Ctrl+C 退出）")
    print("commands=[x, y, yaw], height=[m], obs[0:4]=缩放后的 himloco 观测前4维")

    try:
        while True:
            now = time.monotonic()
            elapsed = min(now - previous_time, 0.1)
            previous_time = now

            while True:
                try:
                    data, _source = receiver.recvfrom(256)
                except BlockingIOError:
                    break
                decoded = decode_packet(data)
                if decoded is None:
                    continue
                state.sequence, axes, state.buttons = decoded
                state.axes = (axes[0], axes[1], axes[2], axes[3])
                state.last_packet_time = now

            connected = state.connected(now)
            state.update_height(elapsed, connected)
            if now >= next_print:
                print("\r\033[K" + format_state(state, now), end="", flush=True)
                next_print = now + tick
            time.sleep(min(tick / 4.0, 0.02))
    except KeyboardInterrupt:
        print("\n监视已停止。")
    finally:
        receiver.close()


if __name__ == "__main__":
    main()
