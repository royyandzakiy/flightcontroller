"""Decodes a blackbox log (LOGxxxxx.BBL) into CSV files.

Usage: python tools/bb_decode.py LOG00001.BBL [output_dir]

Writes <name>_loop.csv, <name>_events.csv and <name>_perf.csv. The record layout
must match lib/blackbox_format/bb_record.cpp.
"""

import csv
import os
import struct
import sys

# Same order as LogCode in src/system/event_log.h.
EVENT_NAMES = [
    "Boot", "WatchdogReset", "ImuFound", "ImuFault", "GyroCalibrated",
    "GyroCalibrationRetry", "AccelCalibrated", "Armed", "ArmBlocked", "Disarmed",
    "FlightMode", "FailsafeStarted", "FailsafeRecovered", "BatteryState", "RcLinkLost",
    "RcLinkRestored", "SdReady", "SdError", "BlackboxStarted", "BlackboxStopped",
    "BlackboxFull", "LoopOverrun", "PidFault", "ParamsLoaded", "ParamsDefaults",
    "ParamsSaved", "CliOpened", "CliClosed", "MotorTest",
]
LEVEL_NAMES = ["ERROR", "WARN", "INFO", "DEBUG"]

# Little endian, sizes exclude the tag byte.
LOOP_FORMAT = "<IH" + "h" * 18 + "H" * 4 + "h" * 4 + "hhH" + "BBB"
EVENT_FORMAT = "<IBBii"
PERF_FORMAT = "<IHHHHIIIIH"

LOOP_COLUMNS = (
    ["time_us", "loop_us"]
    + [f"gyro_raw_{a}" for a in "rpy"]
    + [f"gyro_{a}" for a in "rpy"]
    + [f"setpoint_{a}" for a in "rpy"]
    + [f"p_{a}" for a in "rpy"]
    + [f"i_{a}" for a in "rpy"]
    + [f"d_{a}" for a in "rpy"]
    + [f"motor_{n}" for n in range(1, 5)]
    + ["rc_roll", "rc_pitch", "rc_yaw", "rc_throttle"]
    + ["roll_deg", "pitch_deg", "battery_mv", "arm_state", "flight_mode", "aux_bits"]
)
# Divisor per column to undo the integer scaling.
LOOP_SCALE = (
    [1, 1]
    + [10] * 9
    + [10000] * 9
    + [1000] * 4
    + [1000] * 4
    + [10, 10, 1, 1, 1, 1]
)

PERF_COLUMNS = [
    "time_us", "loop_avg_us", "loop_max_us", "cpu_load_permille", "max_lateness_us",
    "overruns_total", "dropped_records", "sd_write_max_us", "free_heap", "rc_frames_per_s",
]


def read_header(data):
    """Returns (header dict, offset of the first record)."""
    header = {}
    offset = 0
    first = True
    while True:
        end = data.find(b"\n", offset)
        if end < 0:
            raise ValueError("header has no 'end' line")
        line = data[offset:end].decode("ascii", errors="replace")
        offset = end + 1
        if first:
            if not line.startswith("FCBB "):
                raise ValueError("not a blackbox file")
            header["format"] = line.split()[1]
            first = False
        elif line == "end":
            return header, offset
        elif "=" in line:
            key, value = line.split("=", 1)
            header[key] = value


def decode(path, out_dir):
    with open(path, "rb") as f:
        data = f.read()
    header, offset = read_header(data)

    loop_size = struct.calcsize(LOOP_FORMAT)
    event_size = struct.calcsize(EVENT_FORMAT)
    perf_size = struct.calcsize(PERF_FORMAT)

    loops, events, perfs = [], [], []
    while offset < len(data):
        tag = data[offset]
        offset += 1
        if tag == ord("L") and offset + loop_size <= len(data):
            raw = struct.unpack_from(LOOP_FORMAT, data, offset)
            loops.append([v / s if s != 1 else v for v, s in zip(raw, LOOP_SCALE)])
            offset += loop_size
        elif tag == ord("E") and offset + event_size <= len(data):
            time_us, level, code, a, b = struct.unpack_from(EVENT_FORMAT, data, offset)
            name = EVENT_NAMES[code] if code < len(EVENT_NAMES) else f"code{code}"
            level_name = LEVEL_NAMES[level] if level < len(LEVEL_NAMES) else str(level)
            events.append([time_us, level_name, name, a, b])
            offset += event_size
        elif tag == ord("P") and offset + perf_size <= len(data):
            perfs.append(list(struct.unpack_from(PERF_FORMAT, data, offset)))
            offset += perf_size
        else:
            # Zero padding or a cut-off record: the data ends here.
            break

    base = os.path.join(out_dir, os.path.splitext(os.path.basename(path))[0])
    write_csv(base + "_loop.csv", LOOP_COLUMNS, loops)
    write_csv(base + "_events.csv", ["time_us", "level", "event", "a", "b"], events)
    write_csv(base + "_perf.csv", PERF_COLUMNS, perfs)

    print(f"{path}: board {header.get('board')}, loop {header.get('loop_hz')} Hz, "
          f"log divider {header.get('log_div')}")
    print(f"  {len(loops)} loop, {len(events)} event, {len(perfs)} perf records -> {base}_*.csv")
    for event in events:
        print(f"  {event[0] / 1e6:10.3f} s  {event[1]:5}  {event[2]}  a={event[3]} b={event[4]}")


def write_csv(path, columns, rows):
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(columns)
        writer.writerows(rows)


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    decode(sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else os.path.dirname(sys.argv[1]) or ".")
