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
EMA_ALPHA = 0.30
TOTAL_FRESHNESS_SECONDS = 1.0


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
    raw_lines: tuple[Any, Any]
    filtered_lines: tuple[Any, Any]
    cell_times: list[deque[float]] = field(default_factory=lambda: [deque(), deque()])
    cells: list[deque[float]] = field(default_factory=lambda: [deque(), deque()])
    filtered_cells: list[deque[float]] = field(default_factory=lambda: [deque(), deque()])
    filter_state: list[float] = field(default_factory=lambda: [math.nan, math.nan])
    force_filter_state: list[float] = field(default_factory=lambda: [math.nan, math.nan])
    cell_units: list[str | None] = field(default_factory=lambda: [None, None])
    latest_cells: list[float] = field(default_factory=lambda: [math.nan, math.nan])
    latest_filtered_cells: list[float] = field(default_factory=lambda: [math.nan, math.nan])
    latest_cell_times: list[float] = field(default_factory=lambda: [0.0, 0.0])
    record_cell_times: list[list[float]] = field(default_factory=lambda: [[], []])
    record_cells: list[list[float]] = field(default_factory=lambda: [[], []])
    record_filtered_cells: list[list[float]] = field(default_factory=lambda: [[], []])


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

    total_times: deque[float] = deque()
    total_values: deque[float] = deque()
    total_filtered: deque[float] = deque()
    record_total_times: list[float] = []
    record_total_values: list[float] = []
    record_total_filtered: list[float] = []

    fig = plt.figure(figsize=(13, 9))
    grid = fig.add_gridspec(3, 2, left=0.07, right=0.98, top=0.94,
                            bottom=0.30, hspace=0.58, wspace=0.22,
                            height_ratios=(1.0, 1.0, 1.15))
    cell_axes = [
        [fig.add_subplot(grid[0:2, 0]), fig.add_subplot(grid[0:2, 1])],
        [fig.add_subplot(grid[1, 0]), fig.add_subplot(grid[1, 1])],
    ]
    total_ax = fig.add_subplot(grid[2, :])
    for esp_index, axes in enumerate(cell_axes):
        for cell_index, cell_ax in enumerate(axes):
            cell_ax.axhline(0, color="black", linewidth=0.7)
            cell_ax.set_title(f"ESP {esp_index + 1} — Load cell {cell_index + 1}")
            cell_ax.set_ylabel("Thrust (lbf)")
            cell_ax.grid(True, alpha=0.25)
            if esp_index == 1:
                cell_ax.set_visible(False)
    total_ax.axhline(0, color="black", linewidth=0.7)
    total_ax.set_title("Total thrust — all available load cells")
    total_ax.set_xlabel("Computer time (s)")
    total_ax.set_ylabel("Total thrust (lbf)")
    total_ax.grid(True, alpha=0.25)
    total_raw_line, = total_ax.plot([], [], color="#555555", linewidth=0.9,
                                    alpha=0.40, label="Combined raw")
    total_filtered_line, = total_ax.plot([], [], color="#111111", linewidth=2.2,
                                         label="Combined filtered")
    total_ax.legend(loc="upper left")
    peak_text = total_ax.text(0.99, 0.98, "Peak: -- lbf", transform=total_ax.transAxes,
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
        if number == 2:
            for cell_index in range(2):
                cell_axes[0][cell_index].set_subplotspec(grid[0, cell_index])
            fig.canvas.draw_idle()
        raw_lines = []
        filtered_lines = []
        for cell_index, cell_ax in enumerate(cell_axes[number - 1]):
            cell_ax.set_visible(True)
            raw_line, = cell_ax.plot([], [], color=color, linewidth=0.8, alpha=0.35,
                                     label="Raw")
            filtered_line, = cell_ax.plot([], [], color=color, linewidth=2.0,
                                          label="Filtered")
            cell_ax.legend(loc="upper left")
            raw_lines.append(raw_line)
            filtered_lines.append(filtered_line)
        source = SourceState(number, label, endpoint, transport, color,
                             tuple(raw_lines), tuple(filtered_lines))
        sources.append(source)
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
        total_times.clear()
        total_values.clear()
        total_filtered.clear()
        for source in sources:
            for cell_index in range(2):
                source.cell_times[cell_index].clear()
                source.cells[cell_index].clear()
                source.filtered_cells[cell_index].clear()
                source.latest_cells[cell_index] = math.nan
                source.latest_filtered_cells[cell_index] = math.nan
                source.latest_cell_times[cell_index] = 0.0
                source.filter_state[cell_index] = math.nan
                source.force_filter_state[cell_index] = math.nan
                source.cell_units[cell_index] = None
                cell_axes[source.number - 1][cell_index].set_ylabel("Thrust (lbf)")

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
        record_total_times.clear()
        record_total_values.clear()
        record_total_filtered.clear()
        for source in sources:
            for cell_index in range(2):
                source.record_cell_times[cell_index].clear()
                source.record_cells[cell_index].clear()
                source.record_filtered_cells[cell_index].clear()
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
        if not record_total_times:
            status_text.set_text("No samples were captured; start recording first")
            return

        stamp = (record_started_at or datetime.now()).strftime("%Y%m%d_%H%M%S")
        image_path = output.parent / f"thrust_plot_{stamp}.png"
        plot_count = len(sources) * 2 + 1
        saved_fig = Figure(figsize=(12, 3.2 * plot_count), dpi=150)
        FigureCanvasAgg(saved_fig)
        saved_axes = saved_fig.subplots(plot_count, 1, squeeze=False).ravel()
        axis_index = 0
        for source in sources:
            for cell_index in range(2):
                saved_ax = saved_axes[axis_index]
                axis_index += 1
                saved_ax.plot(source.record_cell_times[cell_index],
                              source.record_cells[cell_index],
                              color=source.color, linewidth=0.8, alpha=0.35,
                              label="Raw")
                saved_ax.plot(source.record_cell_times[cell_index],
                              source.record_filtered_cells[cell_index],
                              color=source.color, linewidth=2.0, label="Filtered")
                saved_ax.axhline(0, color="black", linewidth=0.7)
                saved_ax.set_title(f"{source.label} — Load cell {cell_index + 1}")
                saved_ax.set_ylabel(source.cell_units[cell_index] or "Thrust (lbf)")
                saved_ax.grid(True, alpha=0.25)
                saved_ax.legend(loc="upper left")
        saved_total_ax = saved_axes[-1]
        saved_total_ax.plot(record_total_times, record_total_values,
                            color="#555555", linewidth=0.9, alpha=0.40,
                            label="Combined raw")
        saved_total_ax.plot(record_total_times, record_total_filtered,
                            color="#111111", linewidth=2.2,
                            label="Combined filtered")
        saved_total_ax.axhline(0, color="black", linewidth=0.7)
        saved_total_ax.set_title("Total thrust — all available load cells")
        saved_total_ax.set_xlabel("Computer time (s)")
        saved_total_ax.set_ylabel("Total thrust (lbf)")
        saved_total_ax.grid(True, alpha=0.25)
        saved_total_ax.legend(loc="upper left")
        recorded_values = [v for v in record_total_filtered if math.isfinite(v)]
        if recorded_values:
            saved_total_ax.text(0.99, 0.98, f"Peak: {max(recorded_values):.3f} lbf",
                                transform=saved_total_ax.transAxes,
                                ha="right", va="top")
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
                newest_status = f"ESP {len(sources)} connected: {item[2]}"
                continue

            _, source, host_mono, sample = item
            t = host_mono - plot_started
            force_values = [float(sample["cell1_lbf"]),
                            float(sample["cell2_lbf"])]
            raw_counts = [float(sample["raw1"]), float(sample["raw2"])]
            cell_values = [math.nan, math.nan]
            filtered_values = [math.nan, math.nan]
            for cell_index, force_value in enumerate(force_values):
                if source.cell_units[cell_index] is None:
                    if math.isfinite(force_value):
                        source.cell_units[cell_index] = "Thrust (lbf)"
                    elif raw_counts[cell_index] != 0.0:
                        source.cell_units[cell_index] = "ADC counts (uncalibrated)"
                        newest_status = (f"{source.label} cell {cell_index + 1}: "
                                         "ADC data received; calibration required for lbf")
                    if source.cell_units[cell_index]:
                        cell_axes[source.number - 1][cell_index].set_ylabel(
                            source.cell_units[cell_index])

                if source.cell_units[cell_index] == "Thrust (lbf)":
                    display_value = force_value
                elif source.cell_units[cell_index] == "ADC counts (uncalibrated)":
                    display_value = (raw_counts[cell_index]
                                     if raw_counts[cell_index] != 0.0 else math.nan)
                else:
                    display_value = math.nan
                cell_values[cell_index] = display_value

                if math.isfinite(display_value):
                    previous = source.filter_state[cell_index]
                    if math.isfinite(previous):
                        filtered_value = previous + EMA_ALPHA * (display_value - previous)
                    else:
                        filtered_value = display_value
                    source.filter_state[cell_index] = filtered_value
                    filtered_values[cell_index] = filtered_value

                if math.isfinite(force_value):
                    previous_force = source.force_filter_state[cell_index]
                    if math.isfinite(previous_force):
                        filtered_force = (previous_force +
                                          EMA_ALPHA * (force_value - previous_force))
                    else:
                        filtered_force = force_value
                    source.force_filter_state[cell_index] = filtered_force
                    source.latest_cells[cell_index] = force_value
                    source.latest_filtered_cells[cell_index] = filtered_force
                    source.latest_cell_times[cell_index] = host_mono
                if math.isfinite(display_value):
                    source.cell_times[cell_index].append(t)
                    source.cells[cell_index].append(display_value)
                    source.filtered_cells[cell_index].append(
                        filtered_values[cell_index])

            if recording:
                for cell_index in range(2):
                    if math.isfinite(cell_values[cell_index]):
                        source.record_cell_times[cell_index].append(
                            host_mono - record_started_mono)
                        source.record_cells[cell_index].append(
                            cell_values[cell_index])
                        source.record_filtered_cells[cell_index].append(
                            filtered_values[cell_index])

            active_raw: list[float] = []
            active_filtered: list[float] = []
            for active_source in sources:
                for cell_index in range(2):
                    fresh = (host_mono - active_source.latest_cell_times[cell_index]
                             <= TOTAL_FRESHNESS_SECONDS)
                    raw_value = active_source.latest_cells[cell_index]
                    filtered_value = active_source.latest_filtered_cells[cell_index]
                    if fresh and math.isfinite(raw_value):
                        active_raw.append(raw_value)
                    if fresh and math.isfinite(filtered_value):
                        active_filtered.append(filtered_value)
            combined_total = sum(active_raw) if active_raw else math.nan
            combined_filtered = sum(active_filtered) if active_filtered else math.nan
            total_times.append(t)
            total_values.append(combined_total)
            total_filtered.append(combined_filtered)
            if recording:
                record_total_times.append(host_mono - record_started_mono)
                record_total_values.append(combined_total)
                record_total_filtered.append(combined_filtered)
            if math.isfinite(combined_filtered):
                peak = max(peak, combined_filtered)

            for cell_index in range(2):
                while (source.cell_times[cell_index] and
                       t - source.cell_times[cell_index][0] > args.window):
                    source.cell_times[cell_index].popleft()
                    source.cells[cell_index].popleft()
                    source.filtered_cells[cell_index].popleft()
            while total_times and t - total_times[0] > args.window:
                total_times.popleft()
                total_values.popleft()
                total_filtered.popleft()

        if newest_status:
            status_text.set_text(newest_status)

        latest = 0.0
        artists: list[Any] = [peak_text, status_text,
                              total_raw_line, total_filtered_line]
        for source in sources:
            for cell_index in range(2):
                raw_line = source.raw_lines[cell_index]
                filtered_line = source.filtered_lines[cell_index]
                raw_line.set_data(source.cell_times[cell_index],
                                  source.cells[cell_index])
                filtered_line.set_data(source.cell_times[cell_index],
                                       source.filtered_cells[cell_index])
                artists.extend((raw_line, filtered_line))
                finite_cell = [
                    value
                    for series in (source.cells[cell_index],
                                   source.filtered_cells[cell_index])
                    for value in series if math.isfinite(value)
                ]
                if finite_cell:
                    low, high = min(finite_cell), max(finite_cell)
                    pad = max(0.5, (high - low) * 0.12)
                    cell_axes[source.number - 1][cell_index].set_ylim(
                        low - pad, high + pad)
                if source.cell_times[cell_index]:
                    latest = max(latest, source.cell_times[cell_index][-1])

        total_raw_line.set_data(total_times, total_values)
        total_filtered_line.set_data(total_times, total_filtered)
        if total_times:
            latest = max(latest, total_times[-1])
        finite_total = [value for series in (total_values, total_filtered)
                        for value in series if math.isfinite(value)]

        if latest:
            right = max(args.window, latest)
            left = max(0, right - args.window)
            total_ax.set_xlim(left, right)
            for axes in cell_axes:
                for cell_ax in axes:
                    if cell_ax.get_visible():
                        cell_ax.set_xlim(left, right)
        if finite_total:
            low, high = min(finite_total), max(finite_total)
            pad = max(0.5, (high - low) * 0.12)
            total_ax.set_ylim(low - pad, high + pad)
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
