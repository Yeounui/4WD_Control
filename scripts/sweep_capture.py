#!/usr/bin/env python3
"""Capture MOTOR_SWEEP_ON_BOOT telemetry over serial and fit duty<->RPM feedforward coefficients."""
import argparse
import csv
import re
from collections import defaultdict

import numpy as np
import serial

WHEEL_RE = re.compile(r"SWEEP_WHEEL=(\S+)")
DATA_RE = re.compile(r"SWEEP=([^,]+),(-?\d+),(-?\d+)")


def parse_args():
    parser = argparse.ArgumentParser(description="Capture and fit motor duty-sweep telemetry")
    parser.add_argument("--port", required=True, help="Serial port, e.g. /dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--out", default="sweep_log.csv")
    return parser.parse_args()


def capture(port, baud, out_path):
    rows = []
    current_wheel = None
    ser = serial.Serial(port, baud, timeout=1)
    try:
        while True:
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode(errors="replace").strip()
            if not line:
                continue
            print(line)

            if "MOTOR SWEEP END" in line:
                break

            wheel_match = WHEEL_RE.search(line)
            if wheel_match:
                current_wheel = wheel_match.group(1)
                continue

            data_match = DATA_RE.search(line)
            if data_match:
                wheel = data_match.group(1)
                duty = int(data_match.group(2))
                rpm = int(data_match.group(3)) / 1000.0
                rows.append((wheel, duty, rpm))
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()

    with open(out_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["wheel", "duty", "rpm"])
        writer.writerows(rows)

    return rows


def fit_and_report(rows):
    by_wheel = defaultdict(list)
    for wheel, duty, rpm in rows:
        by_wheel[wheel].append((duty, rpm))

    for wheel, points in by_wheel.items():
        moving = [(duty, rpm) for duty, rpm in points if rpm > 0.0]
        if len(moving) < 2:
            print(f"wheel={wheel} skipped (need >=2 moving points, got {len(moving)})")
            continue
        duty_arr = np.array([p[0] for p in moving], dtype=float)
        rpm_arr = np.array([p[1] for p in moving], dtype=float)
        gain, offset = np.polyfit(rpm_arr, duty_arr, 1)
        print(f"wheel={wheel} gain={gain:.3f} offset={offset:.3f}")


def main():
    args = parse_args()
    rows = capture(args.port, args.baud, args.out)
    print(f"\ncaptured {len(rows)} rows -> {args.out}\n")
    fit_and_report(rows)


if __name__ == "__main__":
    main()
