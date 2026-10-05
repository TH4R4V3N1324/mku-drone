#!/usr/bin/env python3
"""Convert a binary flight log (FlightN.bin) to CSV.

Record layout (matches BinaryLogRecord in the Arduino sketch, 40 bytes,
little-endian, no padding on AVR):

    uint32 timestampUs
    float  pitch, roll, yaw
    float  accelX, accelY, accelZ
    float  gyroX, gyroY, gyroZ

Usage:
    python bin_to_csv.py Flight3.bin            # writes Flight3.csv
    python bin_to_csv.py Flight3.bin out.csv
"""
import csv
import struct
import sys
from pathlib import Path

RECORD = struct.Struct("<I9f")
assert RECORD.size == 40

COLUMNS = [
    "timestamp_us", "time_s",
    "pitch", "roll", "yaw",
    "accel_x", "accel_y", "accel_z",
    "gyro_x", "gyro_y", "gyro_z",
]


def convert(src: Path, dst: Path) -> None:
    data = src.read_bytes()
    count, leftover = divmod(len(data), RECORD.size)

    if leftover:
        print(f"Warning: ignoring {leftover} trailing bytes (partial record, "
              f"likely from power loss before a flush)")

    with dst.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(COLUMNS)

        t0 = None
        prev_ts = None
        rollovers = 0
        for i in range(count):
            ts, *values = RECORD.unpack_from(data, i * RECORD.size)

            # micros() wraps every ~71.6 minutes on a 32-bit counter
            if prev_ts is not None and ts < prev_ts:
                rollovers += 1
            prev_ts = ts

            full_ts = ts + rollovers * (1 << 32)
            if t0 is None:
                t0 = full_ts
            time_s = (full_ts - t0) / 1e6

            writer.writerow([full_ts, f"{time_s:.6f}", *values])

    print(f"Wrote {count} records to {dst}")


def main() -> None:
    if len(sys.argv) < 2:
        sys.exit(__doc__)

    src = Path(sys.argv[1])
    dst = Path(sys.argv[2]) if len(sys.argv) > 2 else src.with_suffix(".csv")
    convert(src, dst)


if __name__ == "__main__":
    main()
