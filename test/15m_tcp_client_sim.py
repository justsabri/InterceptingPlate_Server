#!/usr/bin/env python3
"""15m TCP protocol client simulator.

This script sends 15m protocol frames to tcp_server for commissioning checks.
It intentionally supports normal frames, half-packet writes, sticky-packet
writes, invalid frames, and disconnect/reconnect behavior.
"""

import argparse
import socket
import struct
import sys
import time


MSG_CONTROL_COMMAND = 0
MSG_INTERCEPTOR_STATUS = 1
MSG_SHIP_STATUS = 2

CONTROL_PAYLOAD_LENGTH = 20
SHIP_STATUS_PAYLOAD_LENGTH = 52


def header(msg_type: int, payload_length: int) -> bytes:
    return struct.pack(">HH", msg_type, payload_length)


def control_command(
    control_mode: int = 1,
    auto_mode_param: int = 31,
    left_extend: float = 0.25,
    right_extend: float = 0.75,
    timestamp_ms: int = 1786680000000,
) -> bytes:
    payload = struct.pack(
        ">HHffQ",
        control_mode,
        auto_mode_param,
        left_extend,
        right_extend,
        timestamp_ms,
    )
    return header(MSG_CONTROL_COMMAND, CONTROL_PAYLOAD_LENGTH) + payload


def ship_status(
    roll: float = 1.0,
    pitch: float = -2.0,
    rudder: float = 3.0,
    speed: float = 22.0,
    longitude: float = 121.1234567,
    latitude: float = 31.7654321,
    left_engine_speed: float = 1200.0,
    right_engine_speed: float = 1300.0,
    left_engine_gear: int = 1,
    right_engine_gear: int = -1,
    timestamp: float = 1786680000.5,
) -> bytes:
    payload = struct.pack(
        ">ffffddffhhd",
        roll,
        pitch,
        rudder,
        speed,
        longitude,
        latitude,
        left_engine_speed,
        right_engine_speed,
        left_engine_gear,
        right_engine_gear,
        timestamp,
    )
    return header(MSG_SHIP_STATUS, SHIP_STATUS_PAYLOAD_LENGTH) + payload


def invalid_frame() -> bytes:
    return header(99, 4) + b"bad!"


def recv_status(sock: socket.socket, timeout: float) -> bytes:
    sock.settimeout(timeout)
    try:
        data = sock.recv(1024)
    except socket.timeout:
        return b""
    return data


def print_frame_info(prefix: str, data: bytes) -> None:
    if len(data) < 4:
        print(f"{prefix}: no complete header received")
        return

    msg_type, payload_length = struct.unpack(">HH", data[:4])
    print(f"{prefix}: bytes={len(data)} msg_type={msg_type} payload_length={payload_length}")
    if msg_type == MSG_INTERCEPTOR_STATUS and len(data) >= 46:
        current_speed = struct.unpack(">f", data[4:8])[0]
        motor_count = struct.unpack(">H", data[24:26])[0]
        imu_status = struct.unpack(">H", data[34:36])[0]
        slave_status = struct.unpack(">H", data[36:38])[0]
        print(
            f"{prefix}: status speed={current_speed:.3f} "
            f"motor_count={motor_count} imu={imu_status} slave={slave_status}"
        )


def run_once(host: str, port: int, scenario: str, recv_timeout: float) -> None:
    with socket.create_connection((host, port), timeout=5.0) as sock:
        print(f"connected to {host}:{port}")

        control = control_command()
        ship = ship_status()

        if scenario == "normal":
            sock.sendall(control)
            sock.sendall(ship)
        elif scenario == "half":
            split = len(control) // 2
            sock.sendall(control[:split])
            time.sleep(0.1)
            sock.sendall(control[split:])
            sock.sendall(ship)
        elif scenario == "sticky":
            sock.sendall(control + ship)
        elif scenario == "invalid":
            sock.sendall(invalid_frame())
            sock.sendall(control + ship)
        elif scenario == "disconnect":
            sock.sendall(control[:8])
            print("sent partial frame, closing connection")
            return
        else:
            raise ValueError(f"unsupported scenario: {scenario}")

        for index in range(3):
            data = recv_status(sock, recv_timeout)
            if not data:
                print(f"recv[{index}]: timeout/no data")
                continue
            print_frame_info(f"recv[{index}]", data)


def main() -> int:
    parser = argparse.ArgumentParser(description="15m TCP protocol client simulator")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument(
        "--scenario",
        choices=["normal", "half", "sticky", "invalid", "disconnect"],
        default="normal",
    )
    parser.add_argument("--recv-timeout", type=float, default=1.0)
    args = parser.parse_args()

    try:
        run_once(args.host, args.port, args.scenario, args.recv_timeout)
    except OSError as exc:
        print(f"connection failed: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())