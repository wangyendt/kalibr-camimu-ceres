#!/usr/bin/env python3
"""Sweep production sessions with arm64 Kalibr Docker + native Ceres, tally wall clock.

For every session this runs two calibrations on the same ``cam_imu`` inputs:

* Kalibr baseline via ``tools/run_kalibr_docker.py`` using the arm64 Docker image
  (``--platform linux/arm64``); the upstream image now ships an arm64 manifest so
  this is native on Apple Silicon.
* Ceres ``calibrate_cam_imu`` built locally under ``build/`` (native binary, no Docker).

Each run is timed with a monotonic clock. The script prints a per-session table and
the aggregate wall clock (per tool and combined), and writes ``summary.csv`` /
``summary.json`` under the output root.
"""

import argparse
import csv
import datetime as _datetime
import json
import pathlib
import shlex
import subprocess
import sys
import time


DATA_ROOT = pathlib.Path(
    "/Users/wayne/Documents/work/code/project/ffalcon/production_calibration/data"
)
SESSIONS = [
    "2025_03_14_00_10_18",
    "2025_03_14_00_34_14",
    "2025_03_14_00_50_37",
    "2025_03_14_02_13_45",
    "2025_03_14_02_21_41",
    "2025_03_14_10_23_35",
    "2025_04_19_18_43_05",
    "2025_04_19_19_03_03",
    "2025_04_19_19_20_46",
    "2025_04_19_19_35_04",
    "2025_04_19_19_55_25",
    "2025_04_19_20_21_09",
]


def repo_dir() -> pathlib.Path:
    return pathlib.Path(__file__).resolve().parents[1]


def timestamp() -> str:
    return _datetime.datetime.now().strftime("%Y%m%d_%H%M%S")


def quote_command(command) -> str:
    return " ".join(shlex.quote(str(part)) for part in command)


def require_file(path: pathlib.Path, label: str) -> pathlib.Path:
    if not path.is_file():
        raise FileNotFoundError(f"{label} not found: {path}")
    return path


def run_logged(command, run_dir: pathlib.Path, label: str, print_only: bool):
    """Run ``command``, stream output to a log, return (return_code, elapsed_s)."""
    run_dir.mkdir(parents=True, exist_ok=True)
    (run_dir / f"{label}.command.txt").write_text(
        quote_command(command) + "\n", encoding="utf-8"
    )
    if print_only:
        print(quote_command(command), flush=True)
        return 0, 0.0

    log_path = run_dir / f"{label}.log"
    start = time.monotonic()
    proc = subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )
    assert proc.stdout is not None
    with log_path.open("w", encoding="utf-8") as log:
        for line in proc.stdout:
            log.write(line)
            # Kalibr prints a noisy carriage-return progress bar; keep it in the
            # log file but skip it on the console.
            if "Progress" not in line:
                sys.stdout.write(line)
                sys.stdout.flush()
    return_code = proc.wait()
    elapsed_s = time.monotonic() - start
    return return_code, elapsed_s


def kalibr_command(args, dataset: pathlib.Path, session: str, run_dir: pathlib.Path):
    return [
        sys.executable,
        str(repo_dir() / "tools" / "run_kalibr_docker.py"),
        "--dataset",
        str(dataset),
        "--image",
        args.kalibr_image,
        "--run-name",
        f"kalibr_{session}",
        "--out-root",
        str(run_dir),
        "--max-iter",
        str(args.kalibr_max_iter),
        "--platform",
        args.platform,
    ]


def ceres_command(args, dataset: pathlib.Path, result_path: pathlib.Path):
    return [
        str(args.ceres_bin),
        "--corner-defaults",
        "--cam",
        str(dataset / "cam0-camchain-640x400.yaml"),
        "--imu",
        str(dataset / "imu.yaml"),
        "--target",
        str(dataset / "aprilgrid.yaml"),
        "--imu-data",
        str(dataset / "data1.csv"),
        "--corners",
        str(dataset / "cam0_640x400_corners.csv"),
        "--corner-poses",
        str(dataset / "cam0_640x400_corner_poses.csv"),
        "--estimate-time-shift-prior",
        "--estimate-orientation-gravity-prior",
        "--pose-fit-motion-lambda",
        "0.0001",
        "--pose-fit-boundary-anchors",
        "--time-shift-prior-sigma",
        "0.0001",
        "--pose-motion-prior",
        "--pose-motion-translation-variance",
        "10",
        "--pose-motion-rotation-variance",
        "1",
        "--max-iterations",
        str(args.ceres_max_iterations),
        "--solver-max-trust-region-radius",
        "10000000",
        "--output-result",
        str(result_path),
    ]


def ceres_inputs(dataset: pathlib.Path):
    return [
        (dataset / "cam0-camchain-640x400.yaml", "camera chain yaml"),
        (dataset / "imu.yaml", "imu yaml"),
        (dataset / "aprilgrid.yaml", "target yaml"),
        (dataset / "data1.csv", "imu data"),
        (dataset / "cam0_640x400_corners.csv", "corners csv"),
        (dataset / "cam0_640x400_corner_poses.csv", "corner poses csv"),
    ]


def run_session(args, session: str, out_root: pathlib.Path):
    dataset = (args.data_root / session / "cam_imu").resolve()
    case_dir = out_root / session
    row = {"session": session, "dataset": str(dataset)}

    if not args.skip_kalibr:
        print(f"[{session}] Kalibr (arm64 docker)", flush=True)
        k_dir = case_dir / "kalibr"
        k_code, k_elapsed = run_logged(
            kalibr_command(args, dataset, session, k_dir),
            k_dir,
            "kalibr",
            args.print_only,
        )
        row["kalibr_return_code"] = k_code
        row["kalibr_wall_s"] = round(k_elapsed, 3)

    if not args.skip_ceres:
        if not args.print_only:
            for path, label in ceres_inputs(dataset):
                require_file(path, f"Ceres {label}")
        print(f"[{session}] Ceres (native)", flush=True)
        c_dir = case_dir / "ceres"
        c_dir.mkdir(parents=True, exist_ok=True)
        c_code, c_elapsed = run_logged(
            ceres_command(args, dataset, c_dir / "result.yaml"),
            c_dir,
            "ceres",
            args.print_only,
        )
        row["ceres_return_code"] = c_code
        row["ceres_wall_s"] = round(c_elapsed, 3)

    row["combined_wall_s"] = round(
        row.get("kalibr_wall_s", 0.0) + row.get("ceres_wall_s", 0.0), 3
    )
    ok = all(
        row.get(key, 0) == 0
        for key in ("kalibr_return_code", "ceres_return_code")
        if key in row
    )
    row["status"] = "ok" if ok else "FAIL"
    return row


def fmt_seconds(value: float) -> str:
    return f"{value:8.1f}"


def print_table(rows, args):
    header = f"{'session':<22} {'kalibr_s':>10} {'ceres_s':>10} {'combined_s':>11}  status"
    print("\n" + header, flush=True)
    print("-" * len(header), flush=True)
    for row in rows:
        k = fmt_seconds(row.get("kalibr_wall_s", 0.0)) if not args.skip_kalibr else f"{'-':>8}"
        c = fmt_seconds(row.get("ceres_wall_s", 0.0)) if not args.skip_ceres else f"{'-':>8}"
        combined = fmt_seconds(row.get("combined_wall_s", 0.0))
        print(
            f"{row['session']:<22} {k:>10} {c:>10} {combined:>11}  {row['status']}",
            flush=True,
        )
    print("-" * len(header), flush=True)


def print_totals(rows, args):
    count = len(rows)
    kalibr_total = sum(r.get("kalibr_wall_s", 0.0) for r in rows)
    ceres_total = sum(r.get("ceres_wall_s", 0.0) for r in rows)
    combined_total = sum(r.get("combined_wall_s", 0.0) for r in rows)
    failures = [r["session"] for r in rows if r["status"] != "ok"]

    print(f"\nsessions: {count}", flush=True)
    if not args.skip_kalibr:
        print(
            f"kalibr   wall clock: total {kalibr_total:9.1f}s  "
            f"mean {kalibr_total / count if count else 0:7.1f}s",
            flush=True,
        )
    if not args.skip_ceres:
        print(
            f"ceres    wall clock: total {ceres_total:9.1f}s  "
            f"mean {ceres_total / count if count else 0:7.1f}s",
            flush=True,
        )
    print(
        f"combined wall clock: total {combined_total:9.1f}s  "
        f"mean {combined_total / count if count else 0:7.1f}s",
        flush=True,
    )
    if failures:
        print(f"FAILED sessions: {', '.join(failures)}", flush=True)


def write_summary(out_root: pathlib.Path, rows):
    if not rows:
        return
    keys = [
        "session",
        "status",
        "kalibr_wall_s",
        "ceres_wall_s",
        "combined_wall_s",
        "kalibr_return_code",
        "ceres_return_code",
        "dataset",
    ]
    present = [k for k in keys if any(k in row for row in rows)]
    out_root.mkdir(parents=True, exist_ok=True)
    csv_path = out_root / "summary.csv"
    with csv_path.open("w", encoding="utf-8", newline="") as output:
        writer = csv.DictWriter(output, fieldnames=present)
        writer.writeheader()
        for row in rows:
            writer.writerow({k: row.get(k, "") for k in present})
    (out_root / "summary.json").write_text(
        json.dumps(rows, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(f"\nsummary: {csv_path}", flush=True)


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Sweep sessions with arm64 Kalibr Docker + native Ceres and tally wall clock."
        )
    )
    parser.add_argument("--data-root", type=pathlib.Path, default=DATA_ROOT)
    parser.add_argument(
        "--session",
        action="append",
        default=[],
        dest="sessions",
        help="Session name. Repeat to run a subset; default runs all 12.",
    )
    parser.add_argument(
        "--out-root",
        type=pathlib.Path,
        default=repo_dir() / "out" / "wallclock_sweep" / timestamp(),
    )
    parser.add_argument("--platform", default="linux/arm64",
                        help="Docker platform for the Kalibr baseline (default arm64).")
    parser.add_argument("--kalibr-image", default="kalibr-camera-calibration:20.04-arm64",
                        help="Kalibr Docker image tag matching --platform (default arm64 tag).")
    parser.add_argument("--ceres-bin", type=pathlib.Path,
                        default=repo_dir() / "build" / "calibrate_cam_imu")
    parser.add_argument("--kalibr-max-iter", type=int, default=30)
    parser.add_argument("--ceres-max-iterations", type=int, default=150)
    parser.add_argument("--skip-kalibr", action="store_true")
    parser.add_argument("--skip-ceres", action="store_true")
    parser.add_argument("--print-only", action="store_true",
                        help="Print the commands without running them.")
    return parser.parse_args()


def main():
    args = parse_args()
    args.data_root = args.data_root.expanduser().resolve()
    args.out_root = args.out_root.expanduser().resolve()
    args.ceres_bin = args.ceres_bin.expanduser().resolve()
    if args.skip_kalibr and args.skip_ceres:
        raise SystemExit("nothing to do: both --skip-kalibr and --skip-ceres set")
    if not args.skip_ceres and not args.print_only:
        require_file(args.ceres_bin, "native Ceres binary")
    sessions = args.sessions or SESSIONS

    rows = []
    for session in sessions:
        rows.append(run_session(args, session, args.out_root))
        write_summary(args.out_root, rows)

    print_table(rows, args)
    print_totals(rows, args)
    write_summary(args.out_root, rows)
    return 0 if all(row["status"] == "ok" for row in rows) else 1


if __name__ == "__main__":
    raise SystemExit(main())
