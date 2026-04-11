#!/usr/bin/env python3
"""
Throughput benchmark for cpp-sensor-processor.

Runs the pipeline with 1, 2, 4, and 8 threads, captures the reported
throughput from each run, and prints a Markdown table you can paste
into README.md.

Usage (from project root):
    python benchmarks/run_benchmark.py
    python benchmarks/run_benchmark.py --binary build/Release/sensor_proc.exe
    python benchmarks/run_benchmark.py --file data/mock_sensor_xyz.csv --warmup 2 --runs 5
"""

import subprocess
import re
import sys
import os
import statistics
import argparse
import platform

# ---------------------------------------------------------------------------
# Defaults
# ---------------------------------------------------------------------------
DEFAULT_WARMUP = 3
DEFAULT_RUNS   = 5
THREAD_COUNTS  = [1, 2, 4, 8]

def find_binary():
    """Locate the sensor_proc binary relative to the project root."""
    candidates = [
        os.path.join("build", "Release", "sensor_proc.exe"),  # Windows MSVC
        os.path.join("build", "sensor_proc"),                  # Unix
        os.path.join("build", "Release", "sensor_proc"),       # Unix Release
    ]
    for c in candidates:
        if os.path.isfile(c):
            return c
    return None

def run_once(binary, data_file, threads, rgb):
    """Run the pipeline once and return (time_s, throughput_pts_per_sec)."""
    cmd = [binary, "--file", data_file, "--threads", str(threads),
           "--output", os.devnull if platform.system() != "Windows" else "NUL"]
    if rgb:
        cmd.append("--rgb")

    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=120)
        output = result.stdout + result.stderr
    except subprocess.TimeoutExpired:
        print(f"  [TIMEOUT] threads={threads}", file=sys.stderr)
        return None, None

    time_match       = re.search(r"Time:\s*([\d.]+)s", output)
    throughput_match = re.search(r"Throughput:\s*([\d.eE+\-]+)", output)

    if not time_match or not throughput_match:
        print(f"  [PARSE ERROR] Could not extract metrics. Output was:\n{output}", file=sys.stderr)
        return None, None

    return float(time_match.group(1)), float(throughput_match.group(1))

def benchmark(binary, data_file, rgb, warmup, runs):
    """Return list of {threads, median_time, median_throughput} dicts."""
    results = []
    for t in THREAD_COUNTS:
        print(f"  threads={t}: ", end="", flush=True)

        # Warm-up passes (not measured)
        for _ in range(warmup):
            run_once(binary, data_file, t, rgb)
            print(".", end="", flush=True)

        # Timed passes
        times, throughputs = [], []
        for _ in range(runs):
            elapsed, tp = run_once(binary, data_file, t, rgb)
            if elapsed is None:
                continue
            times.append(elapsed)
            throughputs.append(tp)
            print("T", end="", flush=True)

        print()

        if not times:
            results.append({"threads": t, "time": None, "throughput": None})
        else:
            results.append({
                "threads": t,
                "time": statistics.median(times),
                "throughput": statistics.median(throughputs),
            })

    return results

def print_markdown_table(results):
    base_tp = None
    for r in results:
        if r["threads"] == 1 and r["throughput"]:
            base_tp = r["throughput"]
            break

    print()
    print("| Threads | Time (s) | Throughput         | Speedup vs 1T |")
    print("|---------|----------|--------------------|---------------|")
    for r in results:
        t  = r["threads"]
        if r["time"] is None:
            print(f"| {t:<7} | {'—':<8} | {'—':<18} | {'—':<13} |")
            continue

        elapsed = f"{r['time']:.3f}"
        tp_raw  = r["throughput"]
        tp_fmt  = f"{tp_raw:>14,.0f} pts/sec"
        if base_tp and base_tp > 0:
            speedup = f"{tp_raw / base_tp:.2f}×"
        else:
            speedup = "—"
        print(f"| {t:<7} | {elapsed:<8} | {tp_fmt:<18} | {speedup:<13} |")

def main():
    parser = argparse.ArgumentParser(description="cpp-sensor-processor throughput benchmark")
    parser.add_argument("--binary", default=None,
                        help="Path to sensor_proc binary (auto-detected if omitted)")
    parser.add_argument("--file",   default="data/mock_sensor_xyz.csv",
                        help="Input CSV file")
    parser.add_argument("--rgb",    action="store_true",
                        help="Pass --rgb to the binary (use an XYZRGB file)")
    parser.add_argument("--warmup", type=int, default=DEFAULT_WARMUP,
                        help=f"Warm-up runs per thread count (default {DEFAULT_WARMUP})")
    parser.add_argument("--runs",   type=int, default=DEFAULT_RUNS,
                        help=f"Timed runs per thread count (default {DEFAULT_RUNS})")
    args = parser.parse_args()

    # Locate binary
    binary = args.binary or find_binary()
    if not binary or not os.path.isfile(binary):
        print("ERROR: sensor_proc binary not found. Build the project first:", file=sys.stderr)
        print("  cmake -B build -DCMAKE_BUILD_TYPE=Release", file=sys.stderr)
        print("  cmake --build build --config Release --parallel", file=sys.stderr)
        sys.exit(1)

    if not os.path.isfile(args.file):
        print(f"ERROR: data file not found: {args.file}", file=sys.stderr)
        sys.exit(1)

    print(f"Binary : {binary}")
    print(f"File   : {args.file}")
    print(f"Warmup : {args.warmup} runs  |  Timed: {args.runs} runs  |  Metric: median")
    print()

    results = benchmark(binary, args.file, args.rgb, args.warmup, args.runs)
    print_markdown_table(results)
    print()

if __name__ == "__main__":
    main()
