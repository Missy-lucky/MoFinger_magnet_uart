"""Collect Mosense MLX90393 raw frames from LPUART1.

The host accepts every valid 40-byte MLX-only frame and classifies it by
sensor_type + sensor_id.  It does not reject a frame merely because the sensor
type is new; frame header, length, payload length, CRC and tail are still
checked so the payload positions remain reliable.
"""
from __future__ import annotations

import argparse
import csv
import signal
import struct
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

try:
    import serial
    from serial.tools import list_ports
except ImportError as exc:  # pragma: no cover - exercised on the target PC
    raise SystemExit("缺少 pyserial，请先运行: python -m pip install pyserial") from exc

HEADER = b"\x1a\x2b"
TAIL = 0x3C
FRAME_SIZE = 40
PAYLOAD_SIZE = 24
SENSOR_COUNT = 3
SENSOR_TYPE_NAMES = {
    0x01: "指尖磁铁",
    0x02: "指腹磁铁",
    0x03: "指尖线圈",
    0x04: "指腹线圈",
}


def choose_port(requested: str | None) -> str:
    if requested:
        return requested

    ports = sorted(list_ports.comports(), key=lambda port: port.device)
    if not ports:
        raise SystemExit("未检测到串口。请连接 USB 串口设备后重试。")

    usb_ports = [port for port in ports if "USB" in f"{port.description} {port.hwid}".upper()]
    if len(usb_ports) == 1:
        selected = usb_ports[0]
        print(f"自动选择：{selected.device} - {selected.description}")
        return selected.device
    if len(ports) == 1:
        selected = ports[0]
        print(f"自动选择：{selected.device} - {selected.description}")
        return selected.device

    print("检测到多个串口：")
    for index, port in enumerate(ports, start=1):
        print(f"  {index}. {port.device} - {port.description}")
    if not sys.stdin.isatty():
        raise SystemExit("无法交互选择，请使用 --port COMx 指定串口。")
    while True:
        choice = input("请选择串口编号：").strip()
        if choice.isdigit() and 1 <= int(choice) <= len(ports):
            return ports[int(choice) - 1].device
        print("输入无效，请输入列表中的编号。")


def crc8_ccitt(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def parse_frame(frame: bytes) -> dict[str, int | float | str]:
    if len(frame) != FRAME_SIZE or frame[:2] != HEADER or frame[-1] != TAIL:
        raise ValueError("帧头、帧尾或长度错误")
    if crc8_ccitt(frame[:-2]) != frame[-2]:
        raise ValueError("CRC 错误")
    sensor_domain = frame[2]
    sensor_id = frame[3]
    sensor_type = frame[4]
    if struct.unpack_from(">H", frame, 6)[0] != FRAME_SIZE:
        raise ValueError("协议帧长度字段错误")
    tick = struct.unpack_from(">I", frame, 8)[0]
    if struct.unpack_from(">H", frame, 12)[0] != PAYLOAD_SIZE:
        raise ValueError("载荷长度字段错误")
    values = struct.unpack_from(">12h", frame, 14)
    result: dict[str, int | float | str] = {
        "tick_ms": tick,
        "sensor_domain": sensor_domain,
        "sensor_id": sensor_id,
        "sensor_type": sensor_type,
        "sensor_type_name": SENSOR_TYPE_NAMES.get(sensor_type, f"未知类型0x{sensor_type:02X}"),
        "gain_sel": frame[5] & 0x07,
    }
    # Protocol order is board order U4, U6, U8.  Business names are mlx0, mlx2, mlx1.
    business_names = ("mlx0", "mlx2", "mlx1")
    for index, name in enumerate(business_names):
        temperature_delta, x, y, z = values[index * 4:index * 4 + 4]
        temperature_c = 25.0 + temperature_delta / 45.2
        result.update({f"{name}_t_c": round(temperature_c, 2),
                       f"{name}_x": x, f"{name}_y": y, f"{name}_z": z})
    return result


def read_frames(ser: serial.Serial, stop_event: threading.Event | None = None):
    buffer = bytearray()
    while True:
        if stop_event is not None and stop_event.is_set():
            return
        chunk = ser.read(ser.in_waiting or 1)
        if chunk:
            buffer.extend(chunk)
        while True:
            start = buffer.find(HEADER)
            if start < 0:
                # Preserve a trailing 0x1A in case the two-byte header is split
                # across consecutive serial reads.
                keep_header_prefix = bool(buffer) and buffer[-1] == HEADER[0]
                buffer[:] = buffer[-1:] if keep_header_prefix else b""
                break
            if start:
                del buffer[:start]
            if len(buffer) < FRAME_SIZE:
                break
            candidate = bytes(buffer[:FRAME_SIZE])
            try:
                row = parse_frame(candidate)
            except ValueError:
                # Keep remaining bytes and search for the next possible header.
                del buffer[:1]
                continue
            del buffer[:FRAME_SIZE]
            yield row


def main() -> int:
    if len(sys.argv) == 1:
        from mlx90393_collector_gui import main as gui_main
        gui_main()
        return 0
    parser = argparse.ArgumentParser(description="采集三颗 MLX90393 原始值")
    parser.add_argument("--gui", action="store_true", help="打开可视化界面")
    parser.add_argument("--port", help="串口，例如 COM10；不填写时自动检测")
    parser.add_argument("--baud", type=int, default=921600)
    parser.add_argument("--output", default="mlx90393_capture.csv")
    parser.add_argument("--seconds", type=float, default=0, help="采集秒数，0 表示持续到 Ctrl+C")
    args = parser.parse_args()
    if args.gui:
        from mlx90393_collector_gui import main as gui_main
        gui_main()
        return 0
    port = choose_port(args.port)
    fields = ["host_time_iso", "tick_ms", "sensor_domain", "sensor_id",
              "sensor_type", "sensor_type_name", "gain_sel"] + [
        f"{name}_{axis}" for name, _ in (("mlx0", "U4"), ("mlx1", "U8"), ("mlx2", "U6"))
        for axis in ("t_c", "x", "y", "z")
    ]
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    count = 0
    deadline = time.monotonic() + args.seconds if args.seconds > 0 else None
    print(f"连接 {port}，{args.baud} baud，8N1")
    print(f"保存到：{output.resolve()}")
    print("等待二进制数据，按 Ctrl+C 停止。")
    stop_event = threading.Event()
    signal.signal(signal.SIGINT, lambda _signum, _frame: stop_event.set())
    try:
        with serial.Serial(port, args.baud, timeout=0.2) as ser, output.open("w", newline="", encoding="utf-8-sig") as handle:
            writer = csv.DictWriter(handle, fieldnames=fields)
            writer.writeheader()
            for row in read_frames(ser, stop_event):
                row = {"host_time_iso": datetime.now().isoformat(timespec="milliseconds"), **row}
                writer.writerow(row)
                handle.flush()
                count += 1
                if count % 100 == 0:
                    print(f"已采集 {count} 帧")
                if deadline is not None and time.monotonic() >= deadline:
                    break
    except serial.SerialException as exc:
        raise SystemExit(f"无法打开或读取 {port}：{exc}\n请关闭占用该串口的其他程序后重试。") from exc
    print(f"完成：{count} 帧 -> {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
