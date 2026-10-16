"""Standard tuning plots from a decoded blackbox log.

Usage: python tools/bb_plot.py LOG00001_loop.csv

Needs numpy and matplotlib (pip install numpy matplotlib).
"""

import sys

import matplotlib.pyplot as plt
import numpy as np


def main(path):
    data = np.genfromtxt(path, delimiter=",", names=True)
    t = (data["time_us"] - data["time_us"][0]) / 1e6

    fig, axes = plt.subplots(5, 1, sharex=True, figsize=(12, 14))
    fig.suptitle(path)

    # Gyro following the setpoint is what a good tune looks like.
    for ax, axis, name in zip(axes[:3], "rpy", ["roll", "pitch", "yaw"]):
        ax.plot(t, data[f"setpoint_{axis}"], label="setpoint", linewidth=1)
        ax.plot(t, data[f"gyro_{axis}"], label="gyro", linewidth=1)
        ax.plot(t, data[f"gyro_raw_{axis}"], label="gyro raw", linewidth=0.5, alpha=0.4)
        ax.set_ylabel(f"{name} deg/s")
        ax.legend(loc="upper right")
        ax.grid(True)
        twin = ax.twinx()
        twin.plot(t, data[f"p_{axis}"], "--", linewidth=0.7, label="P")
        twin.plot(t, data[f"i_{axis}"], "--", linewidth=0.7, label="I")
        twin.plot(t, data[f"d_{axis}"], "--", linewidth=0.7, label="D")
        twin.set_ylabel("PID terms")
        twin.legend(loc="lower right")

    for n in range(1, 5):
        axes[3].plot(t, data[f"motor_{n}"], label=f"motor {n}", linewidth=1)
    axes[3].plot(t, data["rc_throttle"], "k--", label="throttle", linewidth=1)
    axes[3].set_ylabel("0..1")
    axes[3].legend(loc="upper right")
    axes[3].grid(True)

    axes[4].plot(t, data["loop_us"], linewidth=1)
    axes[4].set_ylabel("loop us")
    axes[4].set_xlabel("time s")
    axes[4].grid(True)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    main(sys.argv[1])
