#!/usr/bin/env python3
# Modified 2026-11-11: configurable listener/certificates and private key permissions.
"""HTTPS/WebSocket relay for forwarding a phone's gamepad state over LAN."""

from __future__ import annotations

import argparse
import asyncio
import io
import ipaddress
import json
import math
import os
import secrets
import socket
import ssl
import struct
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Any

from aiohttp import WSMsgType, web
import qrcode
from cryptography import x509
from cryptography.hazmat.primitives import hashes, serialization
from cryptography.hazmat.primitives.asymmetric import rsa
from cryptography.x509.oid import ExtendedKeyUsageOID, NameOID


BASE_DIR = Path(__file__).resolve().parent
STATIC_DIR = BASE_DIR / "static"
CERT_DIR = Path(os.environ.get("WDBLH_CERT_DIR", str(BASE_DIR / ".certs")))
MAX_MESSAGE_BYTES = 64 * 1024
ROBOT_GAMEPAD_ADDRESS = ("127.0.0.1", 9877)
ROBOT_GAMEPAD_PACKET = struct.Struct("<4sI4fI")
STANDARD_BUTTON_COUNT = 17
WDBLH_CONTROL_SOCKET = Path("/run/wdblh-control.sock")
WDBLH_GETDOWN_SECONDS = 2.8


def local_ip() -> str:
    """Return the LAN address chosen by the OS, without sending traffic."""
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.connect(("192.0.2.1", 9))
        return str(probe.getsockname()[0])
    except OSError:
        try:
            return socket.gethostbyname(socket.gethostname())
        except OSError:
            return "127.0.0.1"
    finally:
        probe.close()


def _write_private_key(path: Path, key: rsa.RSAPrivateKey) -> None:
    # Create keys with owner-only permissions even under a permissive umask.
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
    os.fchmod(fd, 0o600)
    with os.fdopen(fd, "wb") as stream:
        stream.write(key.private_bytes(
            serialization.Encoding.PEM,
            serialization.PrivateFormat.TraditionalOpenSSL,
            serialization.NoEncryption(),
        ))


def ensure_certificates(hosts: list[str]) -> tuple[Path, Path, Path]:
    """Create a private local CA and a server certificate for the given hosts."""
    CERT_DIR.mkdir(parents=True, exist_ok=True, mode=0o700)
    ca_key_path = CERT_DIR / "ca-key.pem"
    ca_cert_path = CERT_DIR / "gamepad-bridge-ca.crt"
    server_key_path = CERT_DIR / "server-key.pem"
    server_cert_path = CERT_DIR / "server-cert.pem"

    if not ca_key_path.exists() or not ca_cert_path.exists():
        now = datetime.now(timezone.utc)
        ca_key = rsa.generate_private_key(public_exponent=65537, key_size=2048)
        subject = x509.Name([
            x509.NameAttribute(NameOID.ORGANIZATION_NAME, "Gamepad Bridge"),
            x509.NameAttribute(NameOID.COMMON_NAME, "Gamepad Bridge Local CA"),
        ])
        ca_cert = (
            x509.CertificateBuilder()
            .subject_name(subject)
            .issuer_name(subject)
            .public_key(ca_key.public_key())
            .serial_number(x509.random_serial_number())
            .not_valid_before(now - timedelta(days=1))
            .not_valid_after(now + timedelta(days=3650))
            .add_extension(x509.BasicConstraints(ca=True, path_length=0), critical=True)
            .add_extension(
                x509.KeyUsage(
                    digital_signature=True, key_encipherment=False,
                    content_commitment=False, data_encipherment=False,
                    key_agreement=False, key_cert_sign=True, crl_sign=True,
                    encipher_only=False, decipher_only=False,
                ),
                critical=True,
            )
            .sign(ca_key, hashes.SHA256())
        )
        _write_private_key(ca_key_path, ca_key)
        ca_cert_path.write_bytes(ca_cert.public_bytes(serialization.Encoding.PEM))

    ca_key = serialization.load_pem_private_key(
        ca_key_path.read_bytes(), password=None)
    ca_cert = x509.load_pem_x509_certificate(ca_cert_path.read_bytes())

    wanted_names: list[x509.GeneralName] = []
    for host in dict.fromkeys([*hosts, "localhost", "127.0.0.1"]):
        try:
            wanted_names.append(x509.IPAddress(ipaddress.ip_address(host)))
        except ValueError:
            wanted_names.append(x509.DNSName(host))

    regenerate = True
    if server_key_path.exists() and server_cert_path.exists():
        try:
            old_cert = x509.load_pem_x509_certificate(server_cert_path.read_bytes())
            old_names = set(old_cert.extensions.get_extension_for_class(
                x509.SubjectAlternativeName).value)
            regenerate = not set(wanted_names).issubset(old_names)
        except (ValueError, x509.ExtensionNotFound):
            regenerate = True

    if regenerate:
        now = datetime.now(timezone.utc)
        server_key = rsa.generate_private_key(public_exponent=65537, key_size=2048)
        subject = x509.Name([
            x509.NameAttribute(NameOID.ORGANIZATION_NAME, "Gamepad Bridge"),
            x509.NameAttribute(NameOID.COMMON_NAME, hosts[0]),
        ])
        server_cert = (
            x509.CertificateBuilder()
            .subject_name(subject)
            .issuer_name(ca_cert.subject)
            .public_key(server_key.public_key())
            .serial_number(x509.random_serial_number())
            .not_valid_before(now - timedelta(days=1))
            .not_valid_after(now + timedelta(days=825))
            .add_extension(x509.SubjectAlternativeName(wanted_names), critical=False)
            .add_extension(
                x509.ExtendedKeyUsage([ExtendedKeyUsageOID.SERVER_AUTH]),
                critical=False,
            )
            .sign(ca_key, hashes.SHA256())
        )
        _write_private_key(server_key_path, server_key)
        server_cert_path.write_bytes(
            server_cert.public_bytes(serialization.Encoding.PEM))

    return server_cert_path, server_key_path, ca_cert_path


class Relay:
    def __init__(self, public_url: str, ca_path: Path) -> None:
        self.public_url = public_url
        self.ca_path = ca_path
        self.senders: set[web.WebSocketResponse] = set()
        self.receivers: set[web.WebSocketResponse] = set()
        self.latest: dict[str, Any] | None = None
        self.robot_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.robot_sequence = 0
        self.robot_control_locked = False
        self.control_busy = False
        self.control_token = secrets.token_urlsafe(24)

    @staticmethod
    def _axis_value(axes: Any, index: int) -> float:
        if not isinstance(axes, list) or index >= len(axes):
            return 0.0
        value = axes[index]
        if not isinstance(value, (int, float)) or not math.isfinite(value):
            return 0.0
        return max(-1.0, min(1.0, float(value)))

    @staticmethod
    def _button_pressed(buttons: Any, index: int) -> bool:
        if not isinstance(buttons, list) or index >= len(buttons):
            return False
        button = buttons[index]
        if not isinstance(button, dict):
            return False
        value = button.get("value", 0)
        return bool(button.get("pressed")) or (
            isinstance(value, (int, float)) and value >= 0.5)

    def send_to_robot(self, payload: dict[str, Any] | None) -> None:
        """Convert the first browser gamepad to the fixed localhost UDP packet."""
        gamepad: dict[str, Any] = {}
        if payload is not None:
            gamepads = payload.get("gamepads")
            if isinstance(gamepads, list) and gamepads and isinstance(gamepads[0], dict):
                gamepad = gamepads[0]

        axes = gamepad.get("axes")
        buttons = gamepad.get("buttons")
        button_bits = 0
        for index in range(STANDARD_BUTTON_COUNT):
            if self._button_pressed(buttons, index):
                button_bits |= 1 << index

        self.robot_sequence = (self.robot_sequence + 1) & 0xFFFFFFFF
        packet = ROBOT_GAMEPAD_PACKET.pack(
            b"GPB1",
            self.robot_sequence,
            *(self._axis_value(axes, index) for index in range(4)),
            button_bits,
        )
        self.robot_socket.sendto(packet, ROBOT_GAMEPAD_ADDRESS)

    def send_robot_buttons(self, button_bits: int) -> None:
        """Send a synthetic neutral gamepad packet with selected buttons."""
        self.robot_sequence = (self.robot_sequence + 1) & 0xFFFFFFFF
        packet = ROBOT_GAMEPAD_PACKET.pack(
            b"GPB1", self.robot_sequence, 0.0, 0.0, 0.0, 0.0, button_bits,
        )
        self.robot_socket.sendto(packet, ROBOT_GAMEPAD_ADDRESS)

    async def controller_command(self, command: str) -> dict[str, Any]:
        """Ask the root-only local supervisor to control WDBLH."""
        try:
            reader, writer = await asyncio.wait_for(
                asyncio.open_unix_connection(str(WDBLH_CONTROL_SOCKET)), 2.0
            )
        except (OSError, asyncio.TimeoutError) as error:
            raise web.HTTPServiceUnavailable(
                text=json.dumps(
                    {"ok": False, "message": f"控制服务不可用：{error}"},
                    ensure_ascii=False,
                ),
                content_type="application/json",
            ) from error
        try:
            writer.write(json.dumps({"command": command}).encode() + b"\n")
            await writer.drain()
            raw = await asyncio.wait_for(reader.readline(), 12.0)
            if not raw:
                raise RuntimeError("控制服务未返回结果")
            result = json.loads(raw)
            if not isinstance(result, dict):
                raise RuntimeError("控制服务返回格式错误")
            return result
        except (OSError, ValueError, asyncio.TimeoutError, RuntimeError) as error:
            raise web.HTTPServiceUnavailable(
                text=json.dumps(
                    {"ok": False, "message": str(error)}, ensure_ascii=False
                ),
                content_type="application/json",
            ) from error
        finally:
            writer.close()
            await writer.wait_closed()

    async def request_getdown(self) -> None:
        """Trigger WDBLH's existing B-button GetDown transition and let it finish."""
        self.send_robot_buttons(0)
        await asyncio.sleep(0.08)
        # Repeat while B is held so UDP loss cannot miss the emergency request.
        deadline = asyncio.get_running_loop().time() + 0.35
        while asyncio.get_running_loop().time() < deadline:
            self.send_robot_buttons(1 << 1)  # standard mapping: B
            await asyncio.sleep(0.04)
        self.send_robot_buttons(0)
        await asyncio.sleep(WDBLH_GETDOWN_SECONDS)

    async def broadcast(self, payload: dict[str, Any]) -> None:
        if not self.receivers:
            return
        message = json.dumps(payload, separators=(",", ":"), ensure_ascii=False)
        stale: list[web.WebSocketResponse] = []
        for peer in self.receivers:
            try:
                await peer.send_str(message)
            except (ConnectionError, RuntimeError):
                stale.append(peer)
        self.receivers.difference_update(stale)

    async def websocket(self, request: web.Request) -> web.WebSocketResponse:
        role = request.query.get("role", "receiver")
        if role not in {"sender", "receiver"}:
            raise web.HTTPBadRequest(text="role must be sender or receiver")

        ws = web.WebSocketResponse(heartbeat=20, max_msg_size=MAX_MESSAGE_BYTES)
        await ws.prepare(request)
        peers = self.senders if role == "sender" else self.receivers
        peers.add(ws)
        if role == "receiver" and self.latest is not None:
            await ws.send_json(self.latest)
        await self.broadcast({
            "type": "peers", "senders": len(self.senders),
            "receivers": len(self.receivers),
        })

        try:
            async for message in ws:
                if message.type != WSMsgType.TEXT or role != "sender":
                    continue
                try:
                    payload = json.loads(message.data)
                except json.JSONDecodeError:
                    await ws.send_json({"type": "error", "message": "invalid JSON"})
                    continue
                if not isinstance(payload, dict) or payload.get("type") != "gamepads":
                    continue
                payload["receivedAt"] = int(
                    datetime.now(timezone.utc).timestamp() * 1000)
                self.latest = payload
                if not self.robot_control_locked:
                    self.send_to_robot(payload)
                await self.broadcast(payload)
        finally:
            peers.discard(ws)
            if role == "sender":
                if not self.senders and not self.robot_control_locked:
                    self.send_to_robot(None)
                await self.broadcast({
                    "type": "peers", "senders": len(self.senders),
                    "receivers": len(self.receivers),
                })
        return ws

    async def index(self, request: web.Request) -> web.FileResponse:
        return web.FileResponse(STATIC_DIR / "sender.html")

    async def monitor(self, request: web.Request) -> web.FileResponse:
        return web.FileResponse(STATIC_DIR / "monitor.html")

    async def config(self, request: web.Request) -> web.Response:
        return web.json_response({
            "phoneUrl": self.public_url,
            "controlToken": self.control_token,
        })

    async def robot_status(self, request: web.Request) -> web.Response:
        result = await self.controller_command("status")
        if result.get("damping"):
            self.robot_control_locked = True
        result["locked"] = self.robot_control_locked
        result["busy"] = self.control_busy
        return web.json_response(result)

    async def robot_control(self, request: web.Request) -> web.Response:
        if request.headers.get("Origin") != f"{request.scheme}://{request.host}":
            raise web.HTTPForbidden(text="invalid origin")
        if request.headers.get("X-Control-Token") != self.control_token:
            raise web.HTTPForbidden(text="invalid control token")
        try:
            body = await request.json()
        except (json.JSONDecodeError, ValueError):
            raise web.HTTPBadRequest(text="invalid JSON")
        action = body.get("action") if isinstance(body, dict) else None
        if action not in {"restart", "emergency_stop"}:
            raise web.HTTPBadRequest(text="invalid action")
        if self.control_busy:
            raise web.HTTPConflict(text="another control action is running")

        self.control_busy = True
        try:
            status = await self.controller_command("status")
            # Lock regular phone gamepad traffic only after the privileged
            # supervisor has answered. A missing supervisor must not disable
            # a controller that is already running.
            self.robot_control_locked = True
            if action == "restart" and status.get("running"):
                await self.request_getdown()
            command = "restart" if action == "restart" else "stop"
            result = await self.controller_command(command)
            if action == "restart" and result.get("ok"):
                self.robot_control_locked = False
            result["locked"] = self.robot_control_locked
            return web.json_response(
                result, status=200 if result.get("ok") else 500
            )
        finally:
            self.control_busy = False

    async def ca_certificate(self, request: web.Request) -> web.FileResponse:
        response = web.FileResponse(self.ca_path)
        response.headers["Content-Disposition"] = (
            'attachment; filename="gamepad-bridge-ca.crt"')
        return response

    async def qr(self, request: web.Request) -> web.Response:
        image = qrcode.make(self.public_url)
        buffer = io.BytesIO()
        image.save(buffer, format="PNG")
        return web.Response(body=buffer.getvalue(), content_type="image/png")


def create_app(public_url: str, ca_path: Path) -> web.Application:
    relay = Relay(public_url, ca_path)
    app = web.Application(client_max_size=MAX_MESSAGE_BYTES)
    app.add_routes([
        web.get("/", relay.index),
        web.get("/monitor", relay.monitor),
        web.get("/ws", relay.websocket),
        web.get("/api/config", relay.config),
        web.get("/api/robot/status", relay.robot_status),
        web.post("/api/robot/control", relay.robot_control),
        web.get("/ca.crt", relay.ca_certificate),
        web.get("/qr.png", relay.qr),
        web.static("/static", STATIC_DIR),
    ])
    return app


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="局域网手机手柄数据桥接")
    parser.add_argument("--bind", default=os.environ.get("WDBLH_BRIDGE_BIND", "0.0.0.0"), help="监听地址")
    parser.add_argument("--host", default=os.environ.get("WDBLH_BRIDGE_HOST"), help="手机访问的电脑地址，默认自动检测")
    parser.add_argument("--port", type=int, default=int(os.environ.get("WDBLH_BRIDGE_PORT", "8443")), help="HTTPS 端口")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    host = args.host or local_ip()
    cert_path, key_path, ca_path = ensure_certificates(
        [host, socket.gethostname()])
    public_url = f"https://{host}:{args.port}/"
    ssl_context = ssl.create_default_context(ssl.Purpose.CLIENT_AUTH)
    ssl_context.load_cert_chain(cert_path, key_path)
    print(f"手机发送端: {public_url}")
    print(f"电脑监视端: {public_url}monitor")
    print(f"手机需信任 CA: {public_url}ca.crt")
    print("机器人手柄输出: UDP 127.0.0.1:9877")
    web.run_app(
        create_app(public_url, ca_path), host=args.bind, port=args.port,
        ssl_context=ssl_context, print=None,
    )


if __name__ == "__main__":
    main()
