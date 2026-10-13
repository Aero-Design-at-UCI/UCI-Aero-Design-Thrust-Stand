#!/usr/bin/env python3
"""Plot and record one or two ESP32 thrust stands over USB or Wi-Fi."""

from __future__ import annotations

import argparse
import csv
import math
import queue
import socket
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.figure import Figure
from matplotlib.widgets import Button, TextBox
import serial
from serial.tools import list_ports


BAUD = 115200
DEFAULT_WIFI_HOST = "192.168.4.1"
DEFAULT_SECOND_HOST = "192.168.4.2"
DEFAULT_TCP_PORT = 3333
STANDARD_GRAVITY = 9.80665
NEWTONS_PER_LBF = 4.4482216152605
SOURCE_COLORS = ("#1f77b4", "#ff7f0e")


class SocketTransport:
    def __init__(self, host: str, port: int) -> None:
        self.sock = socket.create_connection((host, port), timeout=8.0)
        self.sock.settimeout(0.2)
        self.buffer = bytearray()

    def readline(self) -> bytes:
        while b"\n" not in self.buffer:
            try:
                chunk = self.sock.recv(4096)
            except socket.timeout:
                return b""
            if not chunk:
                raise ConnectionError("ESP32 closed the wireless connection")
            self.buffer.extend(chunk)
        line, _, remainder = self.buffer.partition(b"\n")
        self.buffer = bytearray(remainder)
        return line + b"\n"

    def write(self, data: bytes) -> None:
        self.sock.sendall(data)

    def close(self) -> None:
        self.sock.close()


@dataclass
class SourceState:
    number: int
    label: str
    endpoint: str
    transport: Any
    color: str
    raw_line: Any
    filtered_line: Any
    times: deque[float] = field(default_factory=deque)
    totals: deque[float] = field(default_factory=deque)
    filtered: deque[float] = field(default_factory=deque)
    record_times: list[float] = field(default_factory=list)
    record_totals: list[float] = field(default_factory=list)
    record_filtered: list[float] = field(default_factory=list)


def choose_port(requested: str | None) -> str:
    if requested:
        return requested
    ports = list(list_ports.comports())
    likely = [
        p for p in ports
        if any(tag in (p.description or "").lower()
               for tag in ("cp210", "ch340", "usb serial", "usb jtag", "uart"))
    ]
    if len(likely) == 1:
        return likely[0].device
    if not ports:
        raise RuntimeError("No serial ports found. Connect the ESP32 and try again.")
    details = "\n".join(f"  {p.device}: {p.description}" for p in ports)
    raise RuntimeError(f"Could not choose one ESP32 port. Use --port.\n{details}")


def parse_sample(parts: list[str]) -> dict[str, float | int] | None:
    if not parts or parts[0] != "DATA":
        return None
    if len(parts) == 9:  # dual-cell V3/V4
        esp_ms = int(parts[1])
        raw1, raw2 = int(parts[2]), int(parts[3])
        kgf1, kgf2, total_kgf, force_n, filtered_n = map(float, parts[4:9])
    elif len(parts) == 6:  # backward-compatible single-cell firmware
        esp_ms, raw1 = int(parts[1]), int(parts[2])
        raw2 = 0
        kgf1, force_n, filtered_n = map(float, parts[3:6])
        kgf2 = 0.0
        total_kgf = kgf1
    else:
        return None

    return {
        "esp_ms": esp_ms,
        "raw1": raw1,
        "raw2": raw2,
        "kgf1": kgf1,
        "kgf2": kgf2,
        "total_kgf": total_kgf,
        "force_n": force_n,
        "filtered_n": filtered_n,
        "cell1_lbf": kgf1 * STANDARD_GRAVITY / NEWTONS_PER_LBF,
        "cell2_lbf": kgf2 * STANDARD_GRAVITY / NEWTONS_PER_LBF,
        "total_lbf": force_n / NEWTONS_PER_LBF,
        "filtered_lbf": filtered_n / NEWTONS_PER_LBF,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="USB serial port (auto-detected when unambiguous)")
    parser.add_argument("--baud", type=int, default=BAUD)
    parser.add_argument("--wifi", action="store_true",
                        help="Connect ESP 1 over Wi-Fi instead of USB")
    parser.add_argument("--host", default=DEFAULT_WIFI_HOST,
                        help=f"ESP 1 Wi-Fi address (default: {DEFAULT_WIFI_HOST})")
    parser.add_argument("--tcp-port", type=int, default=DEFAULT_TCP_PORT)
    parser.add_argument("--window", type=float, default=30.0,
                        help="Visible plot window in seconds (default: 30)")
    parser.add_argument("--output", type=Path,
                        help="CSV path (default: logs/thrust_YYYYmmdd_HHMMSS.csv)")
    args = parser.parse_args()

    output = args.output or (Path(__file__).resolve().parent.parent / "logs" /
                             f"thrust_{datetime.now():%Y%m%d_%H%M%S}.csv")
    output.parent.mkdir(parents=True, exist_ok=True)

    try:
        if args.wifi:
            initial_transport = SocketTransport(args.host, args.tcp_port)
            initial_endpoint = f"Wi-Fi {args.host}:{args.tcp_port}"
        else:
            port = choose_port(args.port)
            initial_transport = serial.Serial(port, args.baud, timeout=0.2)
            initial_endpoint = f"USB {port}"
            time.sleep(1.5)
            initial_transport.reset_input_buffer()
    except Exception as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 2

    csv_file = output.open("w", newline="", encoding="utf-8")
    writer = csv.writer(csv_file)
    writer.writerow(["host_time_iso", "source", "endpoint", "esp_time_ms",
                     "raw1", "raw2", "kgf1", "kgf2", "total_kgf",
                     "total_force_N", "filtered_total_force_N", "cell1_lbf",
                     "cell2_lbf", "total_force_lbf", "filtered_total_force_lbf"])
    csv_file.flush()
    csv_lock = threading.Lock()
    rows_written = 0

    incoming: queue.Queue[tuple] = queue.Queue()
    stop = threading.Event()
    reader_threads: list[threading.Thread] = []
    sources: list[SourceState] = []
    connecting = {"active": False}
    plot_started = time.monotonic()
    recording = False
    record_started_mono = 0.0
    record_started_at: datetime | None = None
    peak = -math.inf

    fig, ax = plt.subplots(figsize=(12, 7))
    plt.subplots_adjust(bottom=0.31)
    ax.axhline(0, color="black", linewidth=0.7)
    ax.set_title("ESP32 Multi-Stand Thrust Plot")
    ax.set_xlabel("Computer time (s)")
    ax.set_ylabel("Total thrust (lbf)")
    ax.grid(True, alpha=0.25)
    peak_text = ax.text(0.99, 0.98, "Peak: -- lbf", transform=ax.transAxes,
                        ha="right", va="top", fontsize=11)
    status_text = fig.text(0.01, 0.01, f"CSV: {output}", fontsize=9)

    def read_source(source: SourceState) -> None:
        nonlocal rows_written
        while not stop.is_set():
            try:
                line = source.transport.readline().decode("utf-8", errors="replace").strip()
                if not line:
                    continue
                if line.startswith("#"):
                    incoming.put(("status", f"{source.label}: {line[1:].strip()}"))
                    continue
                sample = parse_sample(line.split(","))
                if sample is None:
                    continue
                host_iso = datetime.now().astimezone().isoformat(timespec="milliseconds")
                with csv_lock:
                    writer.writerow([
                        host_iso, source.label, source.endpoint, sample["esp_ms"],
                        sample["raw1"], sample["raw2"], sample["kgf1"],
                        sample["kgf2"], sample["total_kgf"], sample["force_n"],
                        sample["filtered_n"], sample["cell1_lbf"],
                        sample["cell2_lbf"], sample["total_lbf"],
                        sample["filtered_lbf"],
                    ])
                    rows_written += 1
                    if rows_written % 10 == 0:
                        csv_file.flush()
                incoming.put(("data", source, time.monotonic(), sample))
            except ValueError as exc:
                incoming.put(("status", f"{source.label}: skipped invalid data ({exc})"))
            except (serial.SerialException, OSError) as exc:
                if not stop.is_set():
                    incoming.put(("status", f"{source.label}: connection lost ({exc})"))
                break

    def register_source(transport: Any, endpoint: str) -> SourceState:
        number = len(sources) + 1
        label = f"ESP {number}"
        color = SOURCE_COLORS[number - 1]
        raw_line, = ax.plot([], [], color=color, linewidth=0.8, alpha=0.35,
                            label=f"{label} raw")
        filtered_line, = ax.plot([], [], color=color, linewidth=2.0,
                                 label=f"{label} filtered")
        source = SourceState(number, label, endpoint, transport, color,
                             raw_line, filtered_line)
        sources.append(source)
        ax.legend(loc="upper left", ncols=2)
        thread = threading.Thread(target=read_source, args=(source,), daemon=True)
        reader_threads.append(thread)
        thread.start()
        return source

    register_source(initial_transport, initial_endpoint)

    tare_ax = fig.add_axes([0.02, 0.16, 0.15, 0.055])
    force_ax = fig.add_axes([0.24, 0.16, 0.10, 0.055])
    cal1_ax = fig.add_axes([0.36, 0.16, 0.14, 0.055])
    cal2_ax = fig.add_axes([0.52, 0.16, 0.14, 0.055])
    invert_ax = fig.add_axes([0.68, 0.16, 0.13, 0.055])
    clear_ax = fig.add_axes([0.83, 0.16, 0.14, 0.055])
    target_ax = fig.add_axes([0.10, 0.075, 0.06, 0.055])
    host_ax = fig.add_axes([0.25, 0.075, 0.13, 0.055])
    add_ax = fig.add_axes([0.40, 0.075, 0.12, 0.055])
    record_ax = fig.add_axes([0.57, 0.075, 0.17, 0.055])
    stop_record_ax = fig.add_axes([0.76, 0.075, 0.21, 0.055])

    tare_button = Button(tare_ax, "Zero target ESP")
    force_box = TextBox(force_ax, "Force lbf ", initial="2.20462")
    cal1_button = Button(cal1_ax, "Calibrate cell 1")
    cal2_button = Button(cal2_ax, "Calibrate cell 2")
    invert_button = Button(invert_ax, "Invert target")
    clear_button = Button(clear_ax, "Clear plot")
    target_box = TextBox(target_ax, "Target ESP ", initial="1")
    host_box = TextBox(host_ax, "Add host ", initial=DEFAULT_SECOND_HOST)
    add_button = Button(add_ax, "+ Add ESP")
    record_button = Button(record_ax, "Start recording")
    stop_record_button = Button(stop_record_ax, "Stop & save plot")

    def selected_source() -> SourceState | None:
        try:
            number = int(target_box.text)
        except ValueError:
            status_text.set_text("Target ESP must be 1 or 2")
            return None
        if number < 1 or number > len(sources):
            status_text.set_text(f"ESP {number} is not connected")
            return None
        return sources[number - 1]

    def send_target(command: str) -> None:
        source = selected_source()
        if source is None:
            return
        try:
            source.transport.write((command + "\n").encode("ascii"))
        except (serial.SerialException, OSError) as exc:
            status_text.set_text(f"{source.label}: write failed ({exc})")

    tare_button.on_clicked(lambda _event: send_target("TARE"))

    def calibrate(cell_number: int) -> None:
        try:
            force_lbf = float(force_box.text)
            if not 0 < force_lbf <= 44.0925:
                raise ValueError
        except ValueError:
            status_text.set_text("Calibration force must be > 0 and <= 44.0925 lbf")
            return
        send_target(f"CALLBF{cell_number} {force_lbf:.6f}")

    cal1_button.on_clicked(lambda _event: calibrate(1))
    cal2_button.on_clicked(lambda _event: calibrate(2))
    invert_button.on_clicked(lambda _event: send_target("INVERT"))

    def clear_plot(_event=None) -> None:
        nonlocal plot_started, peak
        plot_started = time.monotonic()
        peak = -math.inf
        for source in sources:
            source.times.clear()
            source.totals.clear()
            source.filtered.clear()

    clear_button.on_clicked(clear_plot)

    def add_esp(_event=None) -> None:
        if len(sources) >= 2:
            status_text.set_text("Two ESPs are already connected")
            return
        if connecting["active"]:
            status_text.set_text("ESP connection is already in progress")
            return
        host = host_box.text.strip()
        if not host:
            status_text.set_text("Enter the second ESP address")
            return
        connecting["active"] = True
        status_text.set_text(f"Connecting to second ESP at {host}:{args.tcp_port}...")

        def connect() -> None:
            try:
                transport = SocketTransport(host, args.tcp_port)
                incoming.put(("new_source", transport,
                              f"Wi-Fi {host}:{args.tcp_port}"))
            except OSError as exc:
                incoming.put(("connect_error", host, str(exc)))

        threading.Thread(target=connect, daemon=True).start()

    add_button.on_clicked(add_esp)

    def start_recording(_event=None) -> None:
        nonlocal recording, record_started_mono, record_started_at
        for source in sources:
            source.record_times.clear()
            source.record_totals.clear()
            source.record_filtered.clear()
        record_started_mono = time.monotonic()
        record_started_at = datetime.now()
        recording = True
        record_button.color = "#b7e4c7"
        record_ax.set_facecolor("#b7e4c7")
        status_text.set_text("Recording all connected ESPs...")

    def stop_and_save_plot(_event=None) -> None:
        nonlocal recording
        recording = False
        record_button.color = "0.85"
        record_ax.set_facecolor("0.85")
        if not any(source.record_times for source in sources):
            status_text.set_text("No samples were captured; start recording first")
            return

        stamp = (record_started_at or datetime.now()).strftime("%Y%m%d_%H%M%S")
        image_path = output.parent / f"thrust_plot_{stamp}.png"
        saved_fig = Figure(figsize=(12, 7), dpi=150)
        FigureCanvasAgg(saved_fig)
        saved_ax = saved_fig.add_subplot(1, 1, 1)
        recorded_values: list[float] = []
        for source in sources:
            if not source.record_times:
                continue
            saved_ax.plot(source.record_times, source.record_totals,
                          color=source.color, linewidth=0.8, alpha=0.35,
                          label=f"{source.label} raw")
            saved_ax.plot(source.record_times, source.record_filtered,
                          color=source.color, linewidth=2.0,
                          label=f"{source.label} filtered")
            recorded_values.extend(v for v in source.record_filtered if math.isfinite(v))
        saved_ax.axhline(0, color="black", linewidth=0.7)
        saved_ax.set_title("Recorded Multi-ESP Thrust Run")
        saved_ax.set_xlabel("Computer time (s)")
        saved_ax.set_ylabel("Total thrust per ESP (lbf)")
        saved_ax.grid(True, alpha=0.25)
        saved_ax.legend(loc="upper left", ncols=2)
        if recorded_values:
            saved_ax.text(0.99, 0.98, f"Peak: {max(recorded_values):.3f} lbf",
                          transform=saved_ax.transAxes, ha="right", va="top")
        saved_fig.tight_layout()
        saved_fig.savefig(image_path, bbox_inches="tight")
        status_text.set_text(f"Saved plot for {len(sources)} ESP(s): {image_path}")

    record_button.on_clicked(start_recording)
    stop_record_button.on_clicked(stop_and_save_plot)

    def update(_frame):
        nonlocal peak
        newest_status = None
        while True:
            try:
                item = incoming.get_nowait()
            except queue.Empty:
                break

            if item[0] == "status":
                newest_status = item[1]
                continue
            if item[0] == "connect_error":
                connecting["active"] = False
                newest_status = f"Could not connect to {item[1]}: {item[2]}"
                continue
            if item[0] == "new_source":
                connecting["active"] = False
                register_source(item[1], item[2])
                target_box.set_val(str(len(sources)))
                newest_status = f"ESP {len(sources)} connected: {item[2]}"
                continue

            _, source, host_mono, sample = item
            t = host_mono - plot_started
            total_lbf = float(sample["total_lbf"])
            filtered_lbf = float(sample["filtered_lbf"])
            source.times.append(t)
            source.totals.append(total_lbf)
            source.filtered.append(filtered_lbf)
            if recording:
                source.record_times.append(host_mono - record_started_mono)
                source.record_totals.append(total_lbf)
                source.record_filtered.append(filtered_lbf)
            if math.isfinite(filtered_lbf):
                peak = max(peak, filtered_lbf)
            while source.times and t - source.times[0] > args.window:
                source.times.popleft()
                source.totals.popleft()
                source.filtered.popleft()

        if newest_status:
            status_text.set_text(newest_status)

        finite_y: list[float] = []
        latest = 0.0
        artists: list[Any] = [peak_text, status_text]
        for source in sources:
            source.raw_line.set_data(source.times, source.totals)
            source.filtered_line.set_data(source.times, source.filtered)
            artists.extend((source.raw_line, source.filtered_line))
            if source.times:
                latest = max(latest, source.times[-1])
            finite_y.extend(v for v in list(source.totals) + list(source.filtered)
                            if math.isfinite(v))

        if latest:
            right = max(args.window, latest)
            ax.set_xlim(max(0, right - args.window), right)
        if finite_y:
            low, high = min(finite_y), max(finite_y)
            pad = max(0.5, (high - low) * 0.12)
            ax.set_ylim(low - pad, high + pad)
        peak_text.set_text(f"Peak: {peak:.3f} lbf" if math.isfinite(peak)
                           else "Peak: -- lbf")
        return tuple(artists)

    animation = FuncAnimation(fig, update, interval=100, blit=False,
                              cache_frame_data=False)
    try:
        plt.show()
    finally:
        stop.set()
        for source in sources:
            try:
                source.transport.close()
            except OSError:
                pass
        for thread in reader_threads:
            thread.join(timeout=1.0)
        with csv_lock:
            csv_file.flush()
            csv_file.close()
        print(f"Saved {rows_written} samples from {len(sources)} ESP(s) to {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
