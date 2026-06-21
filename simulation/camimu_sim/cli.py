import argparse
import json
from pathlib import Path

from .simulator import run_simulation


def main() -> int:
    parser = argparse.ArgumentParser(description="Generate synthetic cam-IMU calibration datasets.")
    parser.add_argument("config", type=Path, help="absolute or relative path to the simulation YAML")
    parser.add_argument("--output-dir", type=Path, default=None, help="override paths.output_dir")
    args = parser.parse_args()

    manifest = run_simulation(args.config, args.output_dir)
    print(json.dumps(manifest, indent=2))
    return 0
