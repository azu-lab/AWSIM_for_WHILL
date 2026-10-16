#!/usr/bin/env python3
# Copyright 2026 shoumexi-03, Apache License 2.0
"""Record the resource usage of the AWSIM process only, every interval seconds (run on the host).

    python3 scripts/bench/sample.py --pid 12345 --duration 60 --interval 1 --out samples.csv

The totals of free / nvidia-smi include the desktop, browsers and other ROS 2 processes; the
question is what AWSIM itself needs, so the values are taken for its PID.

Columns:
  rss_mib         RSS of /proc/<pid> (psutil), plus children if any (normally none)
  cpu_percent     CPU usage of the process (1 core = 100 %)
  gpu_fb_mib      fb column of `nvidia-smi pmon -s um` for the process (= NVML per-process VRAM)
  gpu_sm_percent  per-process SM utilization from NVML (nvmlDeviceGetProcessUtilization); often
                  missing, empty when not available
  device_*        values of the whole GPU, NOT of AWSIM; kept as a reference
"""

from __future__ import annotations

import argparse
import csv
import subprocess
import threading
import time

import ctypes

import psutil


class PmonReader(threading.Thread):
    """Keeps nvidia-smi pmon running and the latest line of the target PID."""

    def __init__(self, pid: int):
        super().__init__(daemon=True)
        self.pid = pid
        self.latest: dict | None = None
        self.proc = subprocess.Popen(
            ["nvidia-smi", "pmon", "-s", "um", "-d", "1"],
            stdout=subprocess.PIPE, text=True, bufsize=1)

    def run(self) -> None:
        header: list[str] = []
        for line in self.proc.stdout:
            if line.startswith("#"):
                cols = line.lstrip("#").split()
                if "pid" in cols:
                    header = cols
                continue
            parts = line.split()
            if not header or len(parts) < len(header) - 1:
                continue
            row = dict(zip(header, parts))
            if row.get("pid") == str(self.pid):
                self.latest = {"t": time.monotonic(), **row}

    def stop(self) -> None:
        self.proc.terminate()


class NvmlProcessUtil:
    """Per-process SM utilization from NVML, every sample since the previous call."""

    class _Sample(ctypes.Structure):
        _fields_ = [("pid", ctypes.c_uint), ("timeStamp", ctypes.c_ulonglong),
                    ("smUtil", ctypes.c_uint), ("memUtil", ctypes.c_uint),
                    ("encUtil", ctypes.c_uint), ("decUtil", ctypes.c_uint)]

    def __init__(self, pid: int, index: int = 0):
        self.pid = pid
        self.nvml = ctypes.CDLL("libnvidia-ml.so.1")
        if self.nvml.nvmlInit_v2() != 0:
            raise RuntimeError("nvmlInit failed")
        self.dev = ctypes.c_void_p()
        self.nvml.nvmlDeviceGetHandleByIndex_v2(index, ctypes.byref(self.dev))
        self.last = ctypes.c_ulonglong(0)

    def poll(self) -> float | None:
        n = ctypes.c_uint(0)
        self.nvml.nvmlDeviceGetProcessUtilization(self.dev, None, ctypes.byref(n), self.last)
        if n.value == 0:
            return None
        buf = (self._Sample * n.value)()
        if self.nvml.nvmlDeviceGetProcessUtilization(
                self.dev, buf, ctypes.byref(n), self.last) != 0:
            return None
        samples = buf[:n.value]
        self.last = ctypes.c_ulonglong(max(s.timeStamp for s in samples))
        mine = [s.smUtil for s in samples if s.pid == self.pid]
        return sum(mine) / len(mine) if mine else None


def num(value: str | None) -> float | None:
    try:
        return float(value)
    except (TypeError, ValueError):
        return None  # pmon prints "-" for missing values


def device_stats() -> tuple[float | None, float | None]:
    out = subprocess.run(
        ["nvidia-smi", "--query-gpu=utilization.gpu,memory.used",
         "--format=csv,noheader,nounits"], capture_output=True, text=True).stdout
    parts = [p.strip() for p in out.strip().splitlines()[0].split(",")] if out.strip() else []
    return (num(parts[0]), num(parts[1])) if len(parts) == 2 else (None, None)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--pid", type=int, required=True)
    ap.add_argument("--duration", type=float, required=True)
    ap.add_argument("--interval", type=float, default=1.0)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    proc = psutil.Process(args.pid)
    procs = [proc] + proc.children(recursive=True)
    for p in procs:
        p.cpu_percent(None)  # the first call returns 0

    pmon = PmonReader(args.pid)
    pmon.start()
    util = NvmlProcessUtil(args.pid)
    util.poll()  # drop older samples
    time.sleep(2.5)  # until the first pmon line

    fields = ["t", "rss_mib", "cpu_percent", "gpu_fb_mib", "gpu_sm_percent",
              "device_gpu_util_percent", "device_mem_used_mib"]
    t0 = time.monotonic()
    with open(args.out, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        n = 0
        while True:
            target = t0 + n * args.interval
            if target - t0 > args.duration:
                break
            time.sleep(max(0.0, target - time.monotonic()))
            if not proc.is_running():
                raise SystemExit(f"AWSIM (pid {args.pid}) exited during the measurement")
            rss = 0
            cpu = 0.0
            for p in procs:
                try:
                    rss += p.memory_info().rss
                    cpu += p.cpu_percent(None)
                except psutil.NoSuchProcess:
                    pass
            g = pmon.latest if pmon.latest and time.monotonic() - pmon.latest["t"] < 3 else {}
            sm = util.poll()
            dev_util, dev_mem = device_stats()
            w.writerow({
                "t": round(time.monotonic() - t0, 2),
                "rss_mib": round(rss / 2**20, 1),
                "cpu_percent": round(cpu, 1),
                "gpu_fb_mib": num(g.get("fb")),
                "gpu_sm_percent": round(sm, 1) if sm is not None else None,
                "device_gpu_util_percent": dev_util,
                "device_mem_used_mib": dev_mem,
            })
            f.flush()
            n += 1
    pmon.stop()


if __name__ == "__main__":
    main()
