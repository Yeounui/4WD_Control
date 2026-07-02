#!/usr/bin/env python3
"""Capture Phase 4 SPD=/CNT= telemetry over serial and report per-wheel speed-tracking error."""
import argparse
import csv
import re
import time

import serial

DATA_RE = re.compile(
    r"SPD=(-?\d+),(-?\d+),(-?\d+),(-?\d+),(-?\d+),(-?\d+),(-?\d+),(-?\d+);"
    r"CNT=(\d+),(\d+),(\d+),(\d+)"
)

WHEELS = ("LF", "RF", "LR", "RR")


def parse_args():
    parser = argparse.ArgumentParser(description="Capture speed telemetry and compute RPM tracking error")
    parser.add_argument("--port", required=True, help="Serial port, e.g. COM8 or /dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--out", default="speed_log.csv")
    parser.add_argument("--duration", type=float, default=5.0, help="Capture window in seconds")
    return parser.parse_args()


def capture(port, baud, out_path, duration):
    rows = []
    ser = serial.Serial(port, baud, timeout=1)
    start = time.monotonic()
    try:
        while (time.monotonic() - start) < duration:
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode(errors="replace").strip()
            if not line:
                continue
            print(line)

            match = DATA_RE.search(line)
            if match:
                sLF, mLF, sRF, mRF, sLR, mLR, sRR, mRR, cLF, cRF, cLR, cRR = (
                    int(g) for g in match.groups()
                )
                rows.append({
                    "ts": time.monotonic(),
                    "sLF": sLF, "sRF": sRF, "sLR": sLR, "sRR": sRR,
                    "cLF": cLF, "cRF": cRF, "cLR": cLR, "cRR": cRR,
                })
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()

    with open(out_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["ts", "sLF", "sRF", "sLR", "sRR", "cLF", "cRF", "cLR", "cRR"])
        for row in rows:
            writer.writerow([row[k] for k in ("ts", "sLF", "sRF", "sLR", "sRR", "cLF", "cRF", "cLR", "cRR")])

    return rows


def report(rows):
    if len(rows) < 2:
        print(f"not enough samples to report (got {len(rows)})")
        return

    first, last = rows[0], rows[-1]
    dt = last["ts"] - first["ts"]

    for wheel in WHEELS:
        delta_count = last["c" + wheel] - first["c" + wheel]
        rpm_avg = (delta_count / dt) * 60.0 if dt > 0 else 0.0
        setpoint_rpm = last["s" + wheel] / 10.0
        error_pct = abs(rpm_avg - setpoint_rpm) / setpoint_rpm * 100.0 if setpoint_rpm != 0 else 0.0
        print(f"wheel={wheel} setpoint={setpoint_rpm:.1f} rpm_avg={rpm_avg:.2f} error={error_pct:.1f}%")


def main():
    args = parse_args()
    rows = capture(args.port, args.baud, args.out, args.duration)
    print(f"\ncaptured {len(rows)} rows -> {args.out}\n")
    report(rows)


if __name__ == "__main__":
    main()
