#!/usr/bin/env python3
"""
Live PID tuner over a single serial connection.

Reads telemetry lines in the ">Label:value" format (same format used by the
Teleplot VS Code extension, so your firmware's existing Serial.print calls
work unchanged) and plots them live, while ALSO letting you type commands
(e.g. "p 1.5", "axis y", "show") that get sent straight to the board over
the same port — solving the port-exclusivity problem of using two separate
tools at once.

Usage:
    pip install pyserial matplotlib
    python pid_tuner.py                  # lists ports, prompts you to pick one
    python pid_tuner.py COM5             # Windows
    python pid_tuner.py /dev/ttyACM0     # Linux/Mac
    python pid_tuner.py COM5 --baud 57600 --window 300

Type commands at the "> " prompt in this terminal at any time; they're sent
to the board immediately. Type "quit" to exit (or just close the plot window).
"""

import argparse
import queue
import sys
import threading
import time
from collections import defaultdict, deque

import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


def parse_telemetry(line):
    """Parse a '>Label:value' line. Returns (label, float) or None."""
    line = line.strip()
    if not line.startswith(">") or ":" not in line:
        return None
    label, _, value = line[1:].partition(":")
    try:
        return label.strip(), float(value.strip())
    except ValueError:
        return None


def serial_reader(ser, data_queue, stop_event):
    """Background thread: read lines, push parsed telemetry, print anything else."""
    while not stop_event.is_set():
        try:
            raw = ser.readline()
        except serial.SerialException:
            print("[tuner] Serial connection lost.")
            stop_event.set()
            break
        if not raw:
            continue
        try:
            line = raw.decode("utf-8", errors="replace")
        except Exception:
            continue
        parsed = parse_telemetry(line)
        if parsed:
            data_queue.put(parsed)
        else:
            stripped = line.strip()
            if stripped:
                print(f"[MCU] {stripped}")


def command_sender(ser, stop_event):
    """Foreground-ish thread: read typed commands, forward to the board."""
    print("Type PID commands (e.g. 'axis r', 'p 1.5', 'i 0.04', 'd 18', "
          "'reset', 'show'). Type 'quit' to exit.")
    while not stop_event.is_set():
        try:
            cmd = input("> ")
        except EOFError:
            stop_event.set()
            break
        if cmd.strip().lower() == "quit":
            stop_event.set()
            break
        if cmd.strip():
            ser.write((cmd.strip() + "\n").encode("utf-8"))


def choose_port():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("No serial ports found.")
        sys.exit(1)
    print("Available ports:")
    for i, p in enumerate(ports):
        print(f"  [{i}] {p.device} - {p.description}")
    idx = int(input("Select port index: "))
    return ports[idx].device


def main():
    parser = argparse.ArgumentParser(description="Live PID tuner over serial.")
    parser.add_argument("port", nargs="?", default=None, help="Serial port (e.g. COM5, /dev/ttyACM0)")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (default 115200)")
    parser.add_argument("--window", type=int, default=200, help="Samples shown per trace (default 200)")
    args = parser.parse_args()

    port = args.port or choose_port()

    ser = serial.Serial(port, args.baud, timeout=1)
    time.sleep(2)  # let the board finish its post-reset boot (opening the port resets most Arduinos)
    ser.reset_input_buffer()

    data_queue = queue.Queue()
    stop_event = threading.Event()

    threading.Thread(target=serial_reader, args=(ser, data_queue, stop_event), daemon=True).start()
    threading.Thread(target=command_sender, args=(ser, stop_event), daemon=True).start()

    series = defaultdict(lambda: deque(maxlen=args.window))
    lines = {}

    fig, (ax_track, ax_out) = plt.subplots(2, 1, sharex=True, figsize=(9, 6))
    ax_track.set_ylabel("Setpoint / Actual")
    ax_out.set_ylabel("PID Output")
    ax_out.set_xlabel("Sample #")
    fig.suptitle(f"Live PID Tuner — {port} @ {args.baud} baud")

    def get_line(ax, label):
        if label not in lines:
            (line,) = ax.plot([], [], label=label)
            lines[label] = line
            ax.legend(loc="upper left")
        return lines[label]

    def update(_frame):
        changed = set()
        while not data_queue.empty():
            label, value = data_queue.get_nowait()
            series[label].append(value)
            changed.add(label)

        for label in changed:
            target_ax = ax_out if "output" in label.lower() else ax_track
            line = get_line(target_ax, label)
            ys = list(series[label])
            line.set_data(range(len(ys)), ys)

        for ax in (ax_track, ax_out):
            ax.relim()
            ax.autoscale_view()

        if stop_event.is_set():
            plt.close(fig)

        return list(lines.values())

    ani = FuncAnimation(fig, update, interval=100, cache_frame_data=False)
    plt.tight_layout()
    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        ser.close()


if __name__ == "__main__":
    main()
