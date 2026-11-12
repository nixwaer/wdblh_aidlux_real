#!/usr/bin/env python3
# Modified 2026-11-11: portable paths, explicit serial overrides and baud configuration.
"""Minimal root-side WDBLH process supervisor for the local web bridge."""

from __future__ import annotations

import fcntl
import json
import os
import pwd
import signal
import socketserver
import struct
import subprocess
import termios
import threading
import time
import tty
from glob import glob
from pathlib import Path
from typing import Any


# Portable defaults; shell launchers load config/wdblh.env into the environment.
PROJECT_ROOT = Path(os.environ.get("WDBLH_PROJECT", str(Path(__file__).resolve().parents[1])))
# 运行 gamepad bridge 的非特权用户，daemon 需要把控制 socket 的组交给它
BRIDGE_USER = os.environ.get("WDBLH_BRIDGE_USER", os.environ.get("SUDO_USER", pwd.getpwuid(os.getuid()).pw_name))

CONTROL_BIN = Path(os.environ.get(
    "WDBLH_CONTROL_BIN",
    str(PROJECT_ROOT / "cmake_build_cpu/bin/rl_real_wdblh"),
))
CONTROL_LOG = Path(os.environ.get("WDBLH_CONTROL_LOG", str(PROJECT_ROOT / "logs/controller.log")))

# 串口：本机 CH343 电机口没有 by-id 软链，且自动探测只扫 ttyACM*/ttyUSB*，
# 会漏掉 /dev/ttyCH343USB0，所以必须显式传参。
IMU_PORT = os.environ.get("WDBLH_IMU_PORT")
MOTOR_PORT = os.environ.get("WDBLH_MOTOR_PORT")
IMU_BAUD = os.environ.get("WDBLH_IMU_BAUD", "921600")
MOTOR_BAUD = os.environ.get("WDBLH_MOTOR_BAUD", "4000000")

SOCKET_PATH = Path("/run/wdblh-control.sock")
LOCK_PATH = Path("/run/wdblh-control.lock")


# USB VID:PID —— 用来在没装 udev 规则时也能认准设备（换口/重插后编号变了也能找到）
IMU_USB_ID = ("6877", "4d55")    # DM-Tech DM-IMU-L1
MOTOR_USB_ID = ("1a86", "55d3")  # QinHeng CH343


def _find_tty_by_usb_id(vid: str, pid: str) -> str | None:
    """扫 /sys/class/tty，按 USB VID:PID 找真实的 /dev/tty* 节点。

    纯 sysfs 读取，不需要 udev 规则、不需要 root。
    """
    for tty in sorted(glob("/sys/class/tty/tty*")):
        dev = Path(tty) / "device"
        if not dev.exists():
            continue
        # 设备节点可能挂在 usb 层的父节点上，向上找 idVendor/idProduct
        cur = dev.resolve()
        for _ in range(6):
            try:
                v = (cur / "idVendor").read_text().strip()
                p = (cur / "idProduct").read_text().strip()
            except OSError:
                v = p = ""
            if v and p:
                if v.lower() == vid.lower() and p.lower() == pid.lower():
                    return "/dev/" + Path(tty).name
                break
            if cur.parent == cur:
                break
            cur = cur.parent
    return None


def _resolve_port(link: str, usb_id: tuple[str, str], fallback: str) -> str:
    """解析顺序：udev 软链 → sysfs 按 VID:PID 探测 → 写死的默认值。"""
    if Path(link).exists():
        return link
    found = _find_tty_by_usb_id(*usb_id)
    if found:
        return found
    return fallback


def control_args() -> list[str]:
    """rl_real_wdblh 的命令行参数：<imu_port> <motor_port> <imu_baud> <motor_baud>。

    端口解析优先级：
      1. 显式 WDBLH_IMU_PORT / WDBLH_MOTOR_PORT
      2. /dev/wdblh_imu、/dev/wdblh_motor（装了 udev 规则时存在）
      3. sysfs 按 USB VID:PID 自动找到的实际节点（没装 udev 也能用，号码变了也能用）
      4. 原平台串口名称回退值
    """
    return [
        IMU_PORT or _resolve_port("/dev/wdblh_imu", IMU_USB_ID, "/dev/ttyACM0"),
        MOTOR_PORT or _resolve_port("/dev/wdblh_motor", MOTOR_USB_ID, "/dev/ttyCH343USB0"),
        IMU_BAUD,
        MOTOR_BAUD,
    ]


def float32(value: float) -> float:
    """Round an intermediate exactly like the WDBLH C++ float operations."""
    return struct.unpack("<f", struct.pack("<f", value))[0]


GEAR_RATIO = float32(6.33)
JOINT_DAMPING_KD = float32(3.0)
MOTOR_DAMPING_KD = float32(
    JOINT_DAMPING_KD / float32(GEAR_RATIO * GEAR_RATIO)
)
DAMPING_RATE_HZ = 500
managed_process: subprocess.Popen[bytes] | None = None
damping_thread: threading.Thread | None = None
damping_stop_event = threading.Event()
damping_ready_event = threading.Event()
damping_error: str | None = None


def crc8(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def idle_packet() -> bytes:
    """Build the same 292-byte all-idle packet as pc_make_idle()."""
    packet = bytearray((0xAA,))
    for motor_id in range(12):
        packet.extend(struct.pack("<HHfffff", motor_id, 0, 0, 0, 0, 0, 0))
    packet.append(crc8(packet))
    packet.extend((0, 0))
    if len(packet) != 292:
        raise RuntimeError(f"idle packet size is {len(packet)}, expected 292")
    return bytes(packet)


def damping_packet() -> bytes:
    """Build WDBLH-equivalent Kp=0, dq=0, fixed-Kd damping commands."""
    packet = bytearray((0xAA,))
    for motor_id in range(12):
        packet.extend(struct.pack(
            "<HHfffff",
            motor_id,
            1,
            0.0,
            0.0,
            0.0,
            0.0,
            MOTOR_DAMPING_KD,
        ))
    packet.append(crc8(packet))
    packet.extend((0, 0))
    if len(packet) != 292:
        raise RuntimeError(f"damping packet size is {len(packet)}, expected 292")
    return bytes(packet)


def find_motor_port() -> str:
    """解析电机口：udev 软链 → sysfs VID:PID → by-id → 报错。

    本机 CH343 没有 by-id 软链（USB Single Serial 未上报），所以必须优先用
    udev 软链或按 VID:PID 扫 sysfs，否则故障急停的阻尼控制会打不开串口。
    """
    if MOTOR_PORT:
        if not Path(MOTOR_PORT).exists():
            raise FileNotFoundError(f"Configured motor port does not exist: {MOTOR_PORT}")
        return MOTOR_PORT
    if Path("/dev/wdblh_motor").exists():
        return "/dev/wdblh_motor"
    found = _find_tty_by_usb_id(*MOTOR_USB_ID)
    if found:
        return found
    patterns = (
        "/dev/serial/by-id/*USB_Single_Serial*",
        "/dev/serial/by-id/*1a86*",
        "/dev/serial/by-id/*STM*",
    )
    for pattern in patterns:
        matches = sorted(glob(pattern))
        if matches:
            return os.path.realpath(matches[0])
    if Path("/dev/ttyCH343USB0").exists():
        return "/dev/ttyCH343USB0"
    raise FileNotFoundError("找不到 WDBLH 电机串口")


def open_motor_serial() -> int:
    port = find_motor_port()
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    tty.setraw(fd, termios.TCSANOW)
    attrs = termios.tcgetattr(fd)
    attrs[2] &= ~(termios.PARENB | termios.CSTOPB | termios.CSIZE)
    if hasattr(termios, "CRTSCTS"):
        attrs[2] &= ~termios.CRTSCTS
    if hasattr(termios, "HUPCL"):
        attrs[2] &= ~termios.HUPCL
    attrs[2] |= termios.CREAD | termios.CLOCAL | termios.CS8
    speed_name = f"B{int(MOTOR_BAUD)}"
    if not hasattr(termios, speed_name):
        os.close(fd)
        raise ValueError(f"Unsupported motor baud: {MOTOR_BAUD}")
    speed = getattr(termios, speed_name)
    attrs[4] = speed
    attrs[5] = speed
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    return fd


def write_packet(fd: int, packet: bytes) -> None:
    view = memoryview(packet)
    while view:
        try:
            written = os.write(fd, view)
            if written <= 0:
                raise OSError("电机命令包写入失败")
            view = view[written:]
        except BlockingIOError:
            time.sleep(0.0002)


def damping_loop() -> None:
    global damping_error
    fd = -1
    try:
        fd = open_motor_serial()
        packet = damping_packet()
        next_send = time.monotonic()
        while not damping_stop_event.is_set():
            write_packet(fd, packet)
            damping_ready_event.set()
            # Drain status frames so the USB receive queue cannot fill up.
            try:
                while os.read(fd, 8192):
                    pass
            except BlockingIOError:
                pass
            next_send += 1.0 / DAMPING_RATE_HZ
            delay = next_send - time.monotonic()
            if delay > 0:
                damping_stop_event.wait(delay)
            else:
                next_send = time.monotonic()
    except Exception as error:
        damping_error = str(error)
        damping_ready_event.set()
    finally:
        if fd >= 0:
            # Leave the bus in idle only when damping is intentionally handed
            # back to WDBLH during restart.
            try:
                packet = idle_packet()
                for _ in range(5):
                    write_packet(fd, packet)
                    time.sleep(0.002)
            except OSError:
                pass
            os.close(fd)


def stop_damping() -> None:
    global damping_thread
    if damping_thread is None:
        return
    damping_stop_event.set()
    damping_thread.join(timeout=2.0)
    if damping_thread.is_alive():
        raise RuntimeError("阻尼线程未能停止")
    damping_thread = None


def start_damping() -> None:
    global damping_thread, damping_error
    stop_damping()
    damping_error = None
    damping_stop_event.clear()
    damping_ready_event.clear()
    damping_thread = threading.Thread(
        target=damping_loop,
        name="wdblh-damping",
        daemon=True,
    )
    damping_thread.start()
    if not damping_ready_event.wait(timeout=1.5):
        stop_damping()
        raise RuntimeError("阻尼控制启动超时")
    if damping_error is not None:
        error = damping_error
        stop_damping()
        raise RuntimeError(error)


def damping_active() -> bool:
    return bool(
        damping_thread is not None
        and damping_thread.is_alive()
        and damping_ready_event.is_set()
        and damping_error is None
    )


def controller_pids() -> list[int]:
    expected = str(CONTROL_BIN.resolve())
    pids: list[int] = []
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        try:
            if os.readlink(entry / "exe") == expected:
                pids.append(int(entry.name))
        except (FileNotFoundError, PermissionError, OSError):
            continue
    return sorted(pids)


def stop_controller() -> None:
    global managed_process
    for pid in controller_pids():
        os.kill(pid, signal.SIGINT)
    deadline = time.monotonic() + 6.0
    while controller_pids() and time.monotonic() < deadline:
        time.sleep(0.1)
    for pid in controller_pids():
        os.kill(pid, signal.SIGKILL)
    if managed_process is not None:
        try:
            managed_process.wait(timeout=1.0)
        except subprocess.TimeoutExpired:
            pass
        managed_process = None


def start_controller() -> int:
    global managed_process
    stop_damping()
    pids = controller_pids()
    if pids:
        return pids[0]
    if not CONTROL_BIN.is_file():
        raise FileNotFoundError(f"找不到 WDBLH 程序：{CONTROL_BIN}")
    CONTROL_LOG.parent.mkdir(parents=True, exist_ok=True)
    log_file = CONTROL_LOG.open("ab", buffering=0)
    managed_process = subprocess.Popen(
        [str(CONTROL_BIN), *control_args()],
        cwd=PROJECT_ROOT,
        stdin=subprocess.DEVNULL,
        stdout=log_file,
        stderr=subprocess.STDOUT,
        start_new_session=True,
        close_fds=True,
    )
    log_file.close()
    time.sleep(1.0)
    if managed_process.poll() is not None:
        raise RuntimeError(
            f"WDBLH 启动后立即退出，退出码 {managed_process.returncode}"
        )
    return managed_process.pid


def execute(command: str) -> dict[str, Any]:
    if command == "status":
        pids = controller_pids()
        return {
            "ok": True,
            "running": bool(pids),
            "pid": pids[0] if pids else None,
            "damping": damping_active(),
            "dampingError": damping_error,
        }
    if command == "stop":
        stop_controller()
        try:
            start_damping()
        except Exception as error:
            return {
                "ok": False,
                "running": False,
                "pid": None,
                "damping": False,
                "message": f"WDBLH 已终止，但阻尼控制启动失败：{error}",
            }
        return {
            "ok": True,
            "running": False,
            "pid": None,
            "damping": True,
            "message": "故障急停完成：WDBLH 已先终止，电机正持续保持阻尼状态，手柄输出已锁定",
        }
    if command == "restart":
        stop_controller()
        pid = start_controller()
        return {
            "ok": True,
            "running": True,
            "pid": pid,
            "damping": False,
            "message": f"WDBLH 已重新启动（PID {pid}）",
        }
    return {"ok": False, "message": "不支持的控制命令"}


class ControlHandler(socketserver.StreamRequestHandler):
    def handle(self) -> None:
        try:
            request = json.loads(self.rfile.readline(4096))
            command = request.get("command") if isinstance(request, dict) else None
            response = execute(command)
        except Exception as error:
            response = {"ok": False, "message": str(error)}
        self.wfile.write(
            json.dumps(response, ensure_ascii=False).encode() + b"\n"
        )


class ControlServer(socketserver.UnixStreamServer):
    pass


def main() -> None:
    if os.geteuid() != 0:
        raise SystemExit("wdblh_control_daemon.py 必须以 root 运行")
    lock_file = LOCK_PATH.open("w")
    try:
        fcntl.flock(lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        raise SystemExit("wdblh_control_daemon.py 已经在运行")
    SOCKET_PATH.unlink(missing_ok=True)
    try:
        with ControlServer(str(SOCKET_PATH), ControlHandler) as server:
            try:
                bridge_user = pwd.getpwnam(BRIDGE_USER)
            except KeyError:
                raise SystemExit(
                    f"wdblh_control_daemon.py: 用户 {BRIDGE_USER!r} 不存在，"
                    "请用 WDBLH_BRIDGE_USER 指定运行 gamepad bridge 的用户"
                )
            if bridge_user.pw_uid == 0:
                raise SystemExit("WDBLH_BRIDGE_USER must be a non-root account")
            os.chown(SOCKET_PATH, 0, bridge_user.pw_gid)
            os.chmod(SOCKET_PATH, 0o660)
            server.serve_forever()
    finally:
        stop_damping()
        SOCKET_PATH.unlink(missing_ok=True)


if __name__ == "__main__":
    main()
