#!/usr/bin/env python3
# Copyright 2026 shoumexi-03, Apache License 2.0
"""Summarize a resource measurement: CSV, figures, Markdown table.

    python3 scripts/bench/report.py logs/resource_bench/<date>

Called at the end of scripts/bench_resources.sh. Inputs per condition directory: samples.csv,
topics.json, status.log. Outputs in the same directory:
  summary.csv           mean / max per condition
  fig1_infra_lidar.png  infrastructure LiDAR count vs resource usage (WHILL = 1)
  fig2_whill.png        WHILL count vs resource usage (infrastructure LiDAR = 1)
  results_table.md      table for the documentation

VRAM is the value of the AWSIM PID (nvidia-smi pmon), not of the whole GPU. The per-process GPU
utilization of NVML was unreliable (missing samples vary from run to run), so it is only kept in
the GPU_utilization_* columns; figures and table show the utilization of the whole GPU as a
reference, next to the value without AWSIM (gpu_baseline_no_awsim.csv).
"""

from __future__ import annotations

import csv
import json
import re
import sys
from pathlib import Path
from statistics import mean

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import yaml  # noqa: E402

STATUS_RE = re.compile(r"whill=(\d+) pos=\(([-\d.]+),([-\d.]+),([-\d.]+)\) "
                       r"speed=([-\d.]+) yaw_rate=([-\d.]+) yaw=([-\d.]+) camera_frames=(\d+)")
FPS_RE = re.compile(r"\[ResourceBenchmark\] fps value=([-\d.]+)")
STATUS_PERIOD_S = 5.0  # interval of the status lines of ResourceBenchmarkSpawner

COLOR_MEAN = "#2a78d6"
COLOR_MAX = "#eb6834"
INK = "#0b0b0b"
INK_2 = "#52514e"
GRID = "#e4e3df"


def column(rows, key):
    return [float(r[key]) for r in rows if r.get(key) not in ("", None)]


def summarize(cond_dir: Path, checks: dict) -> dict | None:
    m = re.match(r"infra(\d+)_whill(\d+)", cond_dir.name)
    samples = cond_dir / "samples.csv"
    if not m or not samples.exists():
        return None
    rows = list(csv.DictReader(open(samples)))
    if not rows:
        return None
    rss = column(rows, "rss_mib")
    fb = column(rows, "gpu_fb_mib")
    sm = column(rows, "gpu_sm_percent")
    cpu = column(rows, "cpu_percent")
    dev_util = column(rows, "device_gpu_util_percent")
    dev_mem = column(rows, "device_mem_used_mib")

    topics = {}
    if (cond_dir / "topics.json").exists() and (cond_dir / "topics.json").read_text().strip():
        topics = json.loads((cond_dir / "topics.json").read_text())
    lidar_hz = [v for k, v in topics.items() if k.endswith("/points")]
    camera_hz = [v for k, v in topics.items() if k.endswith("/image_raw")]

    # Motion of every WHILL during the recording (status every 5 s): share of samples above the
    # yaw-rate threshold, and the rendered camera rate from the frame counter.
    per_whill: dict[int, list[tuple[float, int]]] = {}
    status = cond_dir / "status.log"
    fps = []
    for line in (status.read_text().splitlines() if status.exists() else []):
        f = FPS_RE.search(line)
        if f:
            fps.append(float(f.group(1)))
        s = STATUS_RE.search(line)
        if s:
            per_whill.setdefault(int(s.group(1)), []).append((float(s.group(6)), int(s.group(8))))
    moving_ratio, cam_fps = [], []
    for entries in per_whill.values():
        rates = [e[0] for e in entries]
        moving_ratio.append(sum(v >= checks["min_whill_yaw_rate"] for v in rates) / len(rates))
        if len(entries) >= 2:
            cam_fps.append((entries[-1][1] - entries[0][1]) / (STATUS_PERIOD_S * (len(entries) - 1)))

    r1 = lambda x: round(x, 1) if x is not None else ""  # noqa: E731
    return {
        "infrastructure_lidar_count": int(m.group(1)),
        "whill_count": int(m.group(2)),
        "AWSIM_RAM_average_MiB": r1(mean(rss)),
        "AWSIM_RAM_max_MiB": r1(max(rss)),
        "AWSIM_GPU_memory_average_MiB": r1(mean(fb)) if fb else "",
        "AWSIM_GPU_memory_max_MiB": r1(max(fb)) if fb else "",
        "GPU_utilization_average_percent": r1(mean(sm)) if sm else "",
        "GPU_utilization_max_percent": r1(max(sm)) if sm else "",
        "GPU_utilization_valid_samples": f"{len(sm)}/{len(rows)}",
        "AWSIM_CPU_average_percent": r1(mean(cpu)),
        "AWSIM_CPU_max_percent": r1(max(cpu)),
        # frame rate of AWSIM (mean of the 5 s windows of the recording)
        "AWSIM_FPS_average": r1(mean(fps)) if fps else "",
        "AWSIM_FPS_min": r1(min(fps)) if fps else "",
        "samples": len(rows),
        "lidar_topics_ok": f"{sum(h >= checks['min_lidar_hz'] for h in lidar_hz)}/{int(m.group(1))}",
        "lidar_hz_min": r1(min(lidar_hz)) if lidar_hz else "",
        "camera_topics_ok": f"{sum(h >= checks['min_camera_hz'] for h in camera_hz)}/{int(m.group(2))}",
        "camera_hz_min": r1(min(camera_hz)) if camera_hz else "",
        "camera_render_fps_min": r1(min(cam_fps)) if cam_fps else "",
        "whills_moving": f"{sum(r >= 0.8 for r in moving_ratio)}/{int(m.group(2))}",
        "whill_moving_ratio_min": round(min(moving_ratio), 2) if moving_ratio else "",
        # reference only: the whole GPU, including other processes
        "reference_device_gpu_util_average_percent": r1(mean(dev_util)) if dev_util else "",
        "reference_device_gpu_util_max_percent": r1(max(dev_util)) if dev_util else "",
        "reference_device_mem_used_average_MiB": r1(mean(dev_mem)) if dev_mem else "",
    }


def style(ax, title, ylabel):
    ax.set_title(title, loc="left", fontsize=11, color=INK)
    ax.set_ylabel(ylabel, color=INK_2)
    ax.grid(axis="y", color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(INK_2)
    ax.tick_params(colors=INK_2)


def panel(ax, xs, means, other, title, ylabel, fmt, other_label="max (60 s)", baseline=None):
    ax.plot(xs, means, color=COLOR_MEAN, linewidth=2, marker="o", markersize=8,
            markeredgecolor="white", markeredgewidth=2, label="average (60 s)", zorder=3)
    if other is not None:
        # max (min for FPS) is close to the mean: markers only
        ax.scatter(xs, other, color=COLOR_MAX, marker="_", s=260, linewidths=2,
                   label=other_label, zorder=4)
    # label every mean; the last one also with the change from the first
    ax.annotate(fmt(means[0]), (xs[0], means[0]), textcoords="offset points",
                xytext=(6, 12), ha="left", fontsize=8.5, color=INK)
    for x, m in zip(xs[1:-1], means[1:-1]):
        # labels of the middle points below the line
        ax.annotate(fmt(m), (x, m), textcoords="offset points",
                    xytext=(0, -18), ha="center", fontsize=8.5, color=INK)
    d = means[-1] - means[0]
    # label of the last point to its right
    ax.annotate(f"{fmt(means[-1])}\n({'+' if d >= 0 else '-'}{fmt(abs(d))} vs {xs[0]})",
                (xs[-1], means[-1]), textcoords="offset points",
                xytext=(9, 0), ha="left", va="center", fontsize=8.5, color=INK)
    ax.set_xscale("log", base=2)
    ax.set_xticks(xs)
    ax.set_xticklabels([str(x) for x in xs])
    ax.set_xlim(xs[0] / 1.25, xs[-1] * 2.6)
    ax.minorticks_off()
    if baseline is not None:
        ax.axhline(baseline, color=INK_2, linewidth=1, linestyle=":", zorder=1,
                   label=f"AWSIM not running ({baseline:.1f}%)")
    ax.set_ylim(bottom=0, top=max(means + (other or [])) * 1.35)
    style(ax, title, ylabel)


def figure(rows, key, xlabel, fixed, title, command, sensor, gpu_baseline, out: Path, vehicle_lidar=""):
    """sensor: "lidar" | "camera": output rate of the multiplied sensor in the last panel."""
    rows = sorted(rows, key=lambda r: r[key])
    xs = [r[key] for r in rows]
    gib = lambda v: v / 1024  # noqa: E731
    val = lambda k: [float(r[k]) for r in rows]  # noqa: E731
    fig, axes = plt.subplots(2, 3, figsize=(16, 9.2))
    a = axes.ravel()
    panel(a[0], xs, [gib(v) for v in val("AWSIM_RAM_average_MiB")],
          [gib(v) for v in val("AWSIM_RAM_max_MiB")],
          "(a) RAM (RSS)", "GiB", lambda v: f"{v:.2f} GiB")
    panel(a[1], xs, [gib(v) for v in val("AWSIM_GPU_memory_average_MiB")],
          [gib(v) for v in val("AWSIM_GPU_memory_max_MiB")],
          "(b) GPU memory (VRAM)", "GiB", lambda v: f"{v:.2f} GiB")
    # GPU utilization of AWSIM alone is not available reliably: whole GPU, marked as reference
    panel(a[2], xs, val("reference_device_gpu_util_average_percent"),
          val("reference_device_gpu_util_max_percent"),
          "(c) GPU utilization: WHOLE GPU (reference)\n"
          "incl. other processes / AWSIM-only: n/a",
          "% of GPU", lambda v: f"{v:.1f}%", baseline=gpu_baseline)
    panel(a[3], xs, val("AWSIM_CPU_average_percent"), val("AWSIM_CPU_max_percent"),
          "(d) CPU usage", "% (100% = 1 core)", lambda v: f"{v:.0f}%")
    panel(a[4], xs, val("AWSIM_FPS_average"), val("AWSIM_FPS_min"),
          "(e) Simulation frame rate", "FPS", lambda v: f"{v:.1f}", other_label="min (5 s window)")
    if sensor == "lidar":
        panel(a[5], xs, val("lidar_hz_min"), None,
              "(f) LiDAR output rate (slowest unit)", "Hz", lambda v: f"{v:.1f} Hz")
    else:
        panel(a[5], xs, val("camera_hz_min"), None,
              "(f) Camera output rate (slowest unit)", "Hz", lambda v: f"{v:.1f} Hz")
    for ax in a:
        ax.set_xlabel(xlabel, color=INK_2)
    # one legend for the figure
    handles, labels = [], []
    for ax in (a[0], a[4], a[2]):
        for h, l in zip(*ax.get_legend_handles_labels()):
            if l not in labels:
                handles.append(h)
                labels.append(l)
    fig.legend(handles, labels, loc="upper right", bbox_to_anchor=(0.99, 0.925), ncol=2,
               frameon=False, fontsize=9)
    fig.suptitle(title, x=0.01, ha="left", fontsize=14, color=INK, fontweight="bold")
    fig.text(0.01, 0.912, fixed + "   |   (a)(b)(d)(e) are for the AWSIM process only; "
             "(c) is the whole GPU\n"
             f"Every WHILL carries a 1920x1080 camera (10 Hz){vehicle_lidar} and receives the same command "
             f"(v={command['linear']} m/s, ω={command['angular']} rad/s, spin in place)",
             fontsize=9.5, color=INK_2)
    fig.tight_layout(rect=(0, 0, 1, 0.89), h_pad=2.5)
    fig.savefig(out, dpi=150, facecolor="white")
    plt.close(fig)


def main() -> None:
    out = Path(sys.argv[1])
    cfg = yaml.safe_load(open(out / "config.yaml"))["bench"]
    gpu_baseline = None
    if (out / "gpu_baseline_no_awsim.csv").exists():
        base = column(list(csv.DictReader(open(out / "gpu_baseline_no_awsim.csv"))),
                      "device_gpu_util_percent")
        gpu_baseline = mean(base) if base else None
    rows = [s for d in sorted(out.iterdir()) if d.is_dir()
            for s in [summarize(d, cfg["checks"])] if s]
    if not rows:
        raise SystemExit(f"no results in {out}")
    rows.sort(key=lambda r: (r["infrastructure_lidar_count"], r["whill_count"]))

    with open(out / "summary.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    vl = " and a VLP16 LiDAR" if cfg.get("vehicle_lidar", True) else " (vehicle LiDAR off)"
    a = [r for r in rows if r["whill_count"] == 1]
    b = [r for r in rows if r["infrastructure_lidar_count"] == 1]
    if len(a) >= 2:
        figure(a, "infrastructure_lidar_count", "Infrastructure LiDAR Count",
               "Condition: WHILL Model CR = 1",
               "Infrastructure LiDAR Count vs AWSIM Resource Usage", cfg["command"], "lidar", gpu_baseline,
               out / "fig1_infra_lidar.png", vl)
    if len(b) >= 2:
        figure(b, "whill_count", "WHILL Model CR Count",
               "Condition: Infrastructure LiDAR = 1",
               "WHILL Count vs AWSIM Resource Usage", cfg["command"], "camera", gpu_baseline,
               out / "fig2_whill.png", vl)

    def g(v):
        return f"{v / 1024:.2f}" if v != "" else "n/a"

    lines = [
        "| Series | LiDARs | WHILLs | RAM mean / max [GiB] | VRAM mean / max [GiB] "
        "| GPU util. (whole GPU, reference) mean / max [%] | CPU mean / max [%] | FPS mean / min "
        "| LiDAR output | Camera output | WHILLs turning |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---|---|---|",
    ]
    for r in rows:
        exp = ("A, B" if r["infrastructure_lidar_count"] == 1 and r["whill_count"] == 1
               else "A" if r["whill_count"] == 1 else "B" if r["infrastructure_lidar_count"] == 1
               else "-")
        na = lambda v: v if v != "" else "n/a"  # noqa: E731
        lines.append(
            f"| {exp} | {r['infrastructure_lidar_count']} | {r['whill_count']} "
            f"| {g(r['AWSIM_RAM_average_MiB'])} / {g(r['AWSIM_RAM_max_MiB'])} "
            f"| {g(r['AWSIM_GPU_memory_average_MiB'])} / {g(r['AWSIM_GPU_memory_max_MiB'])} "
            f"| {na(r['reference_device_gpu_util_average_percent'])} / "
            f"{na(r['reference_device_gpu_util_max_percent'])} "
            f"| {r['AWSIM_CPU_average_percent']:.0f} / {r['AWSIM_CPU_max_percent']:.0f} "
            f"| {na(r['AWSIM_FPS_average'])} / {na(r['AWSIM_FPS_min'])} "
            f"| {r['lidar_topics_ok']} (min {r['lidar_hz_min']} Hz) "
            f"| {r['camera_topics_ok']} (min {r['camera_hz_min']} Hz) "
            f"| {r['whills_moving']} |")
    (out / "results_table.md").write_text("\n".join(lines) + "\n")
    print("\n".join(lines))


if __name__ == "__main__":
    main()
